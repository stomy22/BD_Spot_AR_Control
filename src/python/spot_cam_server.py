import time

import socket
from urllib import request
import cv2
import numpy as np
import threading
import struct

from bosdyn.api import image_pb2
import bosdyn.client
from bosdyn.client.time_sync import TimedOutError
import bosdyn.client.util
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.api import image_pb2
from scipy import ndimage


ROTATION_ANGLE = {
    'back_fisheye_image': 0,
    'frontleft_fisheye_image': -78,
    'frontright_fisheye_image': -102,
    'left_fisheye_image': 0,
    'right_fisheye_image': 180,
    'hand_color_image': 0
}

STREAM_DICT = {
    "front_left":  {"id": 3,
                    "port_hololens": 62600,
                    "port_nuk": 62610,
                    "image_source": "frontright_fisheye_image"},
    "front_right": {"id": 4,
                    "port_hololens": 62601,
                    "port_nuk": 62611,
                    "image_source": "frontleft_fisheye_image"},
    "left":        {"id": 2,
                    "port_hololens": 62602,
                    "port_nuk": 62612,
                    "image_source": "left_fisheye_image"},
    "arm":         {"id": 1,
                    "port_hololens": 62603,
                    "port_nuk": 62613,
                    "image_source": "hand_color_image"},
    "right":       {"id": 5,
                    "port_hololens": 62604,
                    "port_nuk": 62614,
                    "image_source": "right_fisheye_image"},
    "back":        {"id": 6,
                    "port_hololens": 62605,
                    "port_nuk": 62615,
                    "image_source": "back_fisheye_image"}
    }


def streamid_to_name(id):

    if id == 1:
        return "arm"
    elif id == 2:
        return "left"
    elif id == 3:
        return "front_left"
    elif id == 4:
        return "front_right"
    elif id == 5:
        return "right"
    else:
        return "back"


def pixel_format_type_strings():
    names = image_pb2.Image.PixelFormat.keys()
    return names[1:]


def pixel_format_string_to_enum(enum_string):
    return dict(image_pb2.Image.PixelFormat.items()).get(enum_string)


class SpotStream:
    '''SpotCam class provides mapping between Spot Robot Cams and Hololens
        Attributes:
            image_sources: spot api image sources list
            port_hololens: Port on Hololens 
            port_nuk: Port on Intel Nuk
            spot_user: username
            spot_password: password
            ip_spot: ip of spot
            ip_hololens: ip of Hololens Computer
            simulation: if true a simulation stream would be used instead of Spot Stream
            sim_img_path: path to simulation images
            stream_width: how many pixels horizontally
            stream_height: how many pixels vertically
            udpsocket: udp socket server'''

    def __init__(self, stream_names, spot_user, spot_password, ip_spot, ip_hololens, jpg_quality, pixel_format_string):
        self.stream_names = stream_names
        self.stream_source_names = [STREAM_DICT[stream]["image_source"] for stream in self.stream_names]

        self.spot_user = spot_user
        self.spot_password = spot_password
        self.ip_spot = ip_spot
        self.ip_hololens = ip_hololens

        self.datagram_size = 60000
        self.jpg_quality = jpg_quality
        self.pixel_format_string = pixel_format_string
        self.pixel_format = pixel_format_string_to_enum(pixel_format_string)

        self.stream_targeted = 1
        self.stream_scheduling = []
        self.parallel_stream = 2

        self.blocks_per_stream = 10
        self.current_block = 0
       
        self.update_stream_scheduling()
        self.create_udp_server()
        self.auth_robot()

        eye_tracking_thread = threading.Thread(target=self.receiving_user_looking_at)
        eye_tracking_thread.start()

    def create_udp_server(self):
        for stream in self.stream_names:
            udpsocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udpsocket.bind(("0.0.0.0", STREAM_DICT[stream]['port_nuk']))
            STREAM_DICT[stream]['udp_socket'] = udpsocket
    
    def receiving_user_looking_at(self):
        eye_tracking_socket = udpsocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        eye_tracking_socket.bind(("0.0.0.0", 62630))
        while True:
            data, _ = eye_tracking_socket.recvfrom(1024)
            if data[0] < 8:
                self.stream_targeted = data[0]
            print("stream targeted: ", self.stream_targeted)
            self.update_stream_scheduling()

    def auth_robot(self):
        sdk = bosdyn.client.create_standard_sdk('image_capture')
        self.robot = sdk.create_robot(self.ip_spot)
        self.robot.authenticate(self.spot_user, self.spot_password)
        self.robot.sync_with_directory()
        self.robot.time_sync.wait_for_sync()
        self.image_client = self.robot.ensure_client(ImageClient.default_service_name)
        self.timeout_count_before_reset = 0
    
    def reset_image_client(self):
        del self.robot.service_clients_by_name['image']
        del self.robot.channels_by_authority['api.spot.robot']
        return self.robot.ensure_client('image')

    def get_prio(self):
        streams_available = [1, 2, 3, 4, 5, 6]
        prio = []
        prio.append(self.stream_targeted)
        streams_available.pop(streams_available.index(self.stream_targeted))
        if self.stream_targeted == 1:
            prio.append([self.stream_targeted + 1])
            streams_available.pop(streams_available.index(self.stream_targeted + 1))
        elif self.stream_targeted == 6:
            prio.append([self.stream_targeted - 1])
            streams_available.pop(streams_available.index(self.stream_targeted - 1))
        else:
            prio.append([self.stream_targeted - 1, self.stream_targeted + 1])
            streams_available.pop(streams_available.index(self.stream_targeted - 1))
            streams_available.pop(streams_available.index(self.stream_targeted + 1))
        prio.append(streams_available)
        return prio

    def update_current_block(self):
        self.current_block += 1
        if self.current_block >= self.blocks_per_stream:
            self.current_block = 0
            self.update_stream_scheduling()

    def update_stream_scheduling(self):
        self.stream_scheduling.clear()
        prio = self.get_prio()
        """ 
        prio format:
            default: [3, [2,4], [1,5,6]]
            edge   : [1, [2], [3,4,5,6]]
        """

        for i in range(self.parallel_stream):
            self.stream_scheduling.append([])

        for _ in range(self.blocks_per_stream):
            self.stream_scheduling[0].append(prio[0])

        if self.parallel_stream > 1:
            for i in range(0, self.blocks_per_stream):
                for k in range(1, self.parallel_stream):
                    j = i + k - 1
                    if j < len(prio[2]):
                        self.stream_scheduling[k].append(prio[2][j])
                    else:
                        self.stream_scheduling[k].append(prio[1][j % len(prio[1])])
    
    def get_image_sources_and_reorder_stream_names(self):
        self.stream_names.clear()
        image_sources = []
        for stream in self.stream_scheduling:
            stream_name = streamid_to_name(stream[self.current_block])
            image_sources.append(STREAM_DICT[stream_name]["image_source"])
            self.stream_names.append(stream_name)
        return image_sources

    def get_images_from_spot(self):
        image_sources = self.get_image_sources_and_reorder_stream_names()
        self.update_current_block()

        requests = [
            build_image_request(source, quality_percent=self.jpg_quality, pixel_format=self.pixel_format) # image_format=jpeg
            for source in image_sources
        ]
        try:
            images_response = self.image_client.get_image(requests, timeout=2)
            return images_response

        except TimedOutError as time_err:
            if self.timeout_count_before_reset == 5:
                self.image_client = self.reset_image_client()
                self.timeout_count_before_reset = 0
            else:
                self.timeout_count_before_reset += 1
        except Exception as err:
            raise Exception(err)

    def image_to_opencv(self, image, auto_rotate=True):
        img = np.frombuffer(image.shot.image.data, dtype=np.uint8)
        img = cv2.imdecode(img, -1)

        if auto_rotate:
            img = ndimage.rotate(img, ROTATION_ANGLE[image.source.name], order=0)
        return img

    def send_image(self, img, stream):

        img = self.image_to_opencv(img)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpg_quality]
        _, img = cv2.imencode('.jpg', img, encode_param)

        udp_socket = STREAM_DICT[stream]['udp_socket']
        port_hololens = STREAM_DICT[stream]['port_hololens']

        img_len = len(img)
        img = bytearray(img)
        data_chunks = [img[_:_+self.datagram_size] for _ in range(0, img_len, self.datagram_size)]

        for slice_index, chunk in enumerate(data_chunks):
            udp_socket.sendto(struct.pack("I", slice_index) + struct.pack("I", img_len)
                              + chunk, (self.ip_hololens, port_hololens))
            time.sleep(0.005)

    def run(self):
        while True:
            try:
                images = self.get_images_from_spot()
            except Exception as err:
                print(err)
                continue

            for i, img in enumerate(images):
                stream = self.stream_names[i]
                stream_thread = threading.Thread(target=self.send_image, args=(img, stream, ))
                stream_thread.start()

def main(options):

        stream = SpotStream(options.stream, options.spot_user, options.spot_password, options.ip_spot, options.ip_hololens, 
                            options.quality, options.pixel_format)
        stream.run()


def add_stream_args(parser):
    parser.add_argument('--stream', help='choose the stream which should be started', choices=STREAM_DICT.keys(),
         nargs='+')
    parser.add_argument('--ip-hololens', help='set IP of Hololens', default="192.168.2.4")
    parser.add_argument('--quality', help='JPG Quality', default=50, type=int)
    parser.add_argument('--spot-user', help='username of spot')
    parser.add_argument('--spot-password', help='password of spot')
    parser.add_argument('--ip-spot', help='ip of spot', default="192.168.80.3")
    parser.add_argument('--pixel-format', choices=pixel_format_type_strings(),
        help='Requested pixel format of image. If supplied, will be used for all sources.')


if __name__ == "__main__":
    import sys, argparse
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    add_stream_args(parser)
    options = parser.parse_args(sys.argv)
    if options.stream is not None:
        main(options)
