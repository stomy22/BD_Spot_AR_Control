import spot_cam_server as scs
import spot_controller as sctl

import bosdyn.client.util
import argparse
import threading

def main(argv):
    """Parses command line args.
    Args:
        argv: List of command-line arguments.
    """

    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter, description=('''
        Use this script to control the Spot robot from an Xbox control. Press the Back
        control button to safely power off the robot. Note that the example needs the E-Stop
        to be released. The estop_gui script from the estop SDK example can be used to release
        the E-Stop. Press ctrl-c at any time to safely power off the robot.

        Use this script for starting the camera streams aswell
        '''))

    bosdyn.client.util.add_base_arguments(parser)
    scs.add_stream_args(parser)
    sctl.add_ctl_args(parser)
    parser.add_argument('--control', help='start Spot Control', action='store_true')

    options = parser.parse_args(argv)

    if options.stream is not None:
        cam_thread = threading.Thread(target=scs.main, args=(options,))
        cam_thread.start()

    if options.control:
        sctl.main(options)

if __name__ == "__main__":
    import sys
    main(sys.argv)