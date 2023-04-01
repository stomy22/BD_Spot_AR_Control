using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;

using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;

using UnityEngine;
using UnityEngine.UI;
using TMPro;

using System.Net;
using System.Net.Sockets;
using System.Threading;


public class Eyetracking : MonoBehaviour
{
    private IMixedRealityEyeGazeProvider EyeTrackingProvider => eyeTrackingProvider ?? (eyeTrackingProvider = CoreServices.InputSystem?.EyeGazeProvider);
    private IMixedRealityEyeGazeProvider eyeTrackingProvider = null;
    private Vector3? plausibleLocation;

    public GameObject eye_cube;
    int old_stream_id;
    UdpClient client;
    IPEndPoint serverEndPoint;


    public byte[] createMessage(int messageSize)
    {
        // creates a payload with zeros for initial message
        byte[] payload = new byte[messageSize];
        for (int k = 0; k < payload.Length; k++)
        {
            payload[k] = (byte)'0';
        }
        return payload;
    }

    // Start is called before the first frame update
    void Start(){
        client = new UdpClient(62629);
        serverEndPoint = new IPEndPoint(IPAddress.Parse("192.168.2.5"), 62630);
        client.Connect(serverEndPoint);

        int messageSize = 1;
        var message = createMessage(messageSize);

        // Send message required by Hololens to receive packets
        client.Send(message, messageSize);
    }
    // Update is called once per frame
    void Update() {
    GameObject target_object = CoreServices.InputSystem.EyeGazeProvider.GazeTarget;

    if (target_object){
        int stream_id;
        switch(target_object.name){
        case "PanelArm": stream_id = 1; break;
        case "TitleBarArm": stream_id = 1; break;
        case "PanelLeft": stream_id = 2; break;
        case "TitleBarLeft": stream_id = 2; break;
        case "PanelFrontLeft": stream_id = 3; break;
        case "TitleBarFrontLeft": stream_id = 3; break;
        case "PanelFrontRight": stream_id = 4; break;
        case "TitleBarFrontRight": stream_id = 4; break;
        case "PanelRight": stream_id = 5; break;
        case "TitleBarRight": stream_id = 5; break;
        case "PanelBack": stream_id = 6; break;
        case "TitleBarBack": stream_id = 6; break;
        default: stream_id = 0; break;
        }

        if (stream_id != old_stream_id && stream_id != 0){
            Vector3 position = CoreServices.InputSystem.EyeGazeProvider.GazeTarget.transform.position;
            eye_cube.transform.position = new Vector3(position[0], (float)-0.4, position[2]);
            
            old_stream_id = stream_id;
            int mess_len = 4;
            byte[] message = new byte[mess_len];
            byte[] stream_id_bytes = BitConverter.GetBytes(stream_id);
            
            System.Buffer.BlockCopy(stream_id_bytes, 0, message, 0, 4);
            client.Send(message, mess_len);
            UnityEngine.Debug.Log(stream_id);
        }
     }
    }
}
