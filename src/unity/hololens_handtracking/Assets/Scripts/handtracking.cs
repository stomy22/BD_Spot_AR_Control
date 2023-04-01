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


public class Handtracking : MonoBehaviour
{
    public TextMeshPro value_text;
    public GameObject feedback_cube;
    Renderer feedback_cube_renderer;

    MixedRealityPose pose_ri; // right indextip
    MixedRealityPose pose_rt; // right thumbtip


    MixedRealityPose pose_rik; // right Index knuckle
    MixedRealityPose pose_rpk; // right Pinky knuckle

    Vector3 start_position_right, current_position_right; 
    Vector3 airtap_vector_right;
    Vector3 knuckle_vector_right;

    Vector3 feedback_cube_position;
    Vector3 location;

    bool airtap_right = false;
    bool reset = false;

    String text_right, text_left, text_middle;

    // Communication
    UdpClient client;
    IPEndPoint serverEndPoint;

    // Start is called before the first frame update

    float scalarMultiply(Vector3 vector1, Vector3 vector2){
        return (vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[2] * vector2[2]);
    }

    Vector3 transform_relative_to_camera(Vector3 vector){
        // get Rotation between Camera forward vector and z-Axis
        Quaternion rotation = Quaternion.LookRotation(Camera.main.transform.forward, Vector3.up);

        // change to counter clockwise rotation
        transformed_vector = Quaternion.AngleAxis(360 - rotation.eulerAngles[1], Vector3.up) * vector;
        return transformed_vector;
    }

    bool checkJointPose(){
        bool jointPose = true;
        if !(HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexTip, Handedness.Right, out pose_ri)){
            jointPose = false;}
        if !(HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbTip, Handedness.Right, out pose_rt)){
            jointPose = false;}
        if !(HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexKnuckle, Handedness.Right, out pose_rik)){
            jointPose = false;}
        if !(HandJointUtils.TryGetJointPose(TrackedHandJoint.PinkyKnuckle, Handedness.Right, out pose_rpk)){
            jointPose = false;}
        if(!jointPose){
            airtap_right = false;
        }
        return jointPose;
    }

    bool isGestureCorrectRecognised(){
        knuckle_vector_right = pose_rik.Position - pose_rpk.Position;
        // get angle between Camera forward vector and y-Axis plane and the knuckle vector 
        
        /*  y|
            |
            |____x
            /
        z */

        // xy_norm_vector = Camera.main.transform.forward x (1, 0, 1)^-1
        Vector3 xy_norm_vector = new Vector3(-Camera.main.transform.forward[2], 0 , Camera.main.transform.forward[0]);
        
        // rotation of the knuckle vector around the x-Axis is not tracked -> y=0
        Vector3 knuckle_vector = new Vector3(knuckle_vector_right[0], 0, knuckle_vector_right[2]);
        double knuckle_head_angle = Math.Acos(scalarMultiply(knuckle_vector, xy_norm_vector) / (knuckle_vector.magnitude * xy_norm_vector.magnitude)) * 180 / Math.PI;
        
        if (Math.Abs(90 - knuckle_head_angle) < 60){
            return true;}
        
        airtap_right = false;
        return false;
    }

    bool isAirtap(){
        airtap_vector_right = pose_ri.Position - pose_rt.Position;
        feedback_cube.transform.position = pose_ri.Position;

        if (airtap_vector_right.magnitude < 0.015 && !airtap_right){
            airtap_right = true;
            start_position_right = pose_rik.Position;
            feedback_cube.SetActive(true);
        }
        if (airtap_vector_right.magnitude > 0.02 && airtap_right){
            airtap_right = false;
        }
    }

    void resetLocation(){
        location = new Vector3(0,0,0);
        feedback_cube.SetActive(false);
    }

    void send_trajectory(){
        while (true){
            int mess_len = 12;
            byte[] message = new byte[mess_len];

            byte[] x = BitConverter.GetBytes(location[0]);
            byte[] y = BitConverter.GetBytes(location[1]);
            byte[] z = BitConverter.GetBytes(location[2]);

            System.Buffer.BlockCopy(x, 0, message, 0, 4);
            System.Buffer.BlockCopy(y, 0, message, 4, 4);
            System.Buffer.BlockCopy(z, 0, message, 8, 4);

            client.Send(message, mess_len);
            if(!reset){reset = true;}
            Thread.Sleep(33);  
        }
    }

    void createUDPClient(){
        client = new UdpClient(62619);
        serverEndPoint = new IPEndPoint(IPAddress.Parse("192.168.2.5"), 62620);
        client.Connect(serverEndPoint);
        int messageSize = 1;
        var message = createMessage(messageSize);
        client.Send(message, messageSize);
    }
    
    public byte[] createMessage(int messageSize){
        // creates a payload with zeros for initial message
        byte[] payload = new byte[messageSize];
        for (int k = 0; k < payload.Length; k++)
        {
            payload[k] = (byte)'0';
        }
        return payload;
    }

    void Start(){
        text_left = "handtracking script started...";
        text_middle = "";
        text_right = "";

        feedback_cube_position = feedback_cube.transform.position;
        feedback_cube_renderer = feedback_cube.GetComponent<Renderer>();
        feedback_cube_renderer.material.SetColor("_Color", Color.green);

        // Turn off all hand rays
        PointerUtils.SetHandRayPointerBehavior(PointerBehavior.AlwaysOff);
        
        createUDPClient();

        Thread t_send = new Thread(send_trajectory);
        t_send.IsBackground = true;
        t_send.Start();
    }
    void Update(){
        text_right = location.ToString();
        if(checkJointPose() and isGestureCorrectRecognised() and isAirtap()){ 
            location = pose_rik.Position - start_position_right;
            location = transform_relative_to_camera(location);

            if(reset){
                feedback_cube_position = feedback_cube.transform.position;
                start_position_right = pose_rik.Position;
                reset = false;
            }
        }
        else{
            resetLocation();
        }
    }
}
