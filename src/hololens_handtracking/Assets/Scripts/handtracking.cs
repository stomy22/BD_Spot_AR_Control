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
    public GameObject cube;
    public GameObject arm_cube;
    Renderer cube_renderer;

    /* MixedRealityPose 
     * https://docs.microsoft.com/de-DE/dotnet/api/microsoft.mixedreality.toolkit.utilities.mixedrealitypose?view=mixed-reality-toolkit-unity-2020-dotnet-2.7.0
     * Handtracking
     * https://xrlab.dev/custom-hand-joint-tracking-for-hololens/
     * Camera
     * https://dev.bostondynamics.com/python/examples/gripper_camera_params/readme
     */

     // send packet every 10ms

    MixedRealityPose pose_ri; // right indextip
    MixedRealityPose pose_rt; // right thumbtip
    MixedRealityPose pose_li; // left indextip
    MixedRealityPose pose_lt; // left thumbtip
    MixedRealityPose pose_lwrist; // left wrist
    MixedRealityPose pose_rwrist; // right wrist

    MixedRealityPose pose_rik; // right Index knuckle
    MixedRealityPose pose_rpk; // right Pinky knuckle
    MixedRealityPose pose_lik; // right Index knuckle
    MixedRealityPose pose_lpk; // right Pinky knuckle

    Vector3 start_position_right, current_position_right; //start of gesture right
    Vector3 start_position_left, current_position_left; //start of gesture
    Vector3 airtap_vector_right; // distance between index and thumb tip
    Vector3 airtap_vector_left;

    Vector3 knuckle_vector_right;
    Vector3 knuckle_vector_left;

    Vector3 wrist_vector;
    Vector3 cube_position;
    Vector3 cube_arm_position;

    Vector3 location;

    bool airtap_left = false;
    bool airtap_right = false;
    bool grab = false;
    bool reset = false;
    String text_right, text_left, text_middle;

    // Communication
    UdpClient client;
    IPEndPoint serverEndPoint;

    byte rot = 0;

    // Start is called before the first frame update
    void Start()
    {
        text_left = "handtracking script started...";
        text_middle = "";
        text_right = "";
        cube_position = cube.transform.position;
        cube_arm_position = arm_cube.transform.position;
        cube_renderer = cube.GetComponent<Renderer>();
        cube_renderer.material.SetColor("_Color", Color.green);
        //cube.SetActive(false); // Hide Cube
        //arm_cube.SetActive(false);
        // Turn off all hand rays
        PointerUtils.SetHandRayPointerBehavior(PointerBehavior.AlwaysOff);
        
        // create Communication Channel
        client = new UdpClient(62619);
        serverEndPoint = new IPEndPoint(IPAddress.Parse("192.168.2.5"), 62620);
        client.Connect(serverEndPoint);
        int messageSize = 1;
        var message = createMessage(messageSize);
        // Send message required by Hololens to receive packets
        client.Send(message, messageSize);
        
        Thread t_send = new Thread(send);
        t_send.IsBackground = true;
        t_send.Start();
    }

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
    float scalarMultiply(Vector3 vector1, Vector3 vector2){
        return (vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[2] * vector2[2]);
    }
    Vector3 transform_relative_to_camera(Vector3 vector){
        // get Rotation between Camera forward vector and z-Axis
        Quaternion rotation = Quaternion.LookRotation(Camera.main.transform.forward, Vector3.up);
        // change to counter clockwise rotation
        vector = Quaternion.AngleAxis(360 - rotation.eulerAngles[1], Vector3.up) * vector;
        return vector;
    }
    // Update is called once per frame
    void Update()
    {
        text_right = location.ToString();
        // right hand
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexTip, Handedness.Right, out pose_ri))
        {
           if (HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbTip, Handedness.Right, out pose_rt))
           {
               if (HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexKnuckle, Handedness.Right, out pose_rik))
                {
                    if (HandJointUtils.TryGetJointPose(TrackedHandJoint.PinkyKnuckle, Handedness.Right, out pose_rpk))
                    {
                        // get vector from indexknuckle to pinkyknuckle
                        knuckle_vector_right = pose_rik.Position - pose_rpk.Position;
                        // get angle between Camera forward vector and y-Axis plane and the knuckle vector 
                        // rotation of the knuckle vector around the x-Axis is not tracked -> y=0

                      /*  y|
                           |
                           |____x
                          /
                      z */

                        // xy_norm_vector = Camera.main.transform.forward x (1, 0, 1)^-1
                        Vector3 xy_norm_vector = new Vector3(-Camera.main.transform.forward[2], 0 , Camera.main.transform.forward[0]);
                        Vector3 knuckle_vector = new Vector3(knuckle_vector_right[0], 0, knuckle_vector_right[2]);
                        double knuckle_head_angle = Math.Acos(scalarMultiply(knuckle_vector, xy_norm_vector) / (knuckle_vector.magnitude * xy_norm_vector.magnitude)) * 180 / Math.PI;
                        if(Math.Abs(90 - knuckle_head_angle) < 60){

                            //text_right = "true";
                            //text_right = location[0].ToString();
                            //text_middle = knuckle_vector_right.ToString();

                            airtap_vector_right = pose_ri.Position - pose_rt.Position;
                            cube.transform.position = pose_ri.Position;

                            if (airtap_vector_right.magnitude < 0.015 && !airtap_right){
                                airtap_right = true;
                                start_position_right = pose_rik.Position;
                                cube.SetActive(true);
                            }
                            if (airtap_vector_right.magnitude > 0.02 && airtap_right){
                                airtap_right = false;
                               //cube_arm_position = arm_cube.transform.position;
                            }

                            if(airtap_right){
                                location = pose_rik.Position - start_position_right;
                                location = transform_relative_to_camera(location);
                               // arm_cube.transform.position = cube_arm_position + location;
                                if(reset){
                                    cube_position = cube.transform.position;
                                   // cube_arm_position = arm_cube.transform.position;
                                    start_position_right = pose_rik.Position;
                                    reset = false;
                                }
                            }
                            else{
                                //cube_arm_position = arm_cube.transform.position;
                                location = new Vector3(0,0,0);
                                cube.SetActive(false);
                            }
                        }
                        else{
                            //text_right = "false";
                            //cube_arm_position = arm_cube.transform.position;
                            airtap_right = false;
                            cube.SetActive(false);
                            location = new Vector3(0,0,0);
                            
                        }
                    }
                }
                
           }
           else{
                //cube_arm_position = arm_cube.transform.position;
                airtap_right = false;
                cube.SetActive(false);
               location = new Vector3(0,0,0);
               text_right = "move Hand in Front of Hololens!";}
        }
        else{
            //cube_arm_position = arm_cube.transform.position;
            airtap_right = false;
            cube.SetActive(false);
            location = new Vector3(0,0,0);
            text_right = "move Hand in Front of Hololens!";}

        // left hand
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexTip, Handedness.Left, out pose_li))
        {
           if (HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbTip, Handedness.Left, out pose_lt))
           {
               if (HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexKnuckle, Handedness.Left, out pose_lik))
                {
                    if (HandJointUtils.TryGetJointPose(TrackedHandJoint.PinkyKnuckle, Handedness.Left, out pose_lpk))
                    {
                        // get vector from indexknuckle to pinkyknuckle
                        knuckle_vector_left = pose_lik.Position - pose_lpk.Position;
                        if(Math.Abs(knuckle_vector_left[0]) < 0.03 && Math.Abs(knuckle_vector_left[1]) < 0.03){
                            text_left = "true";
                            //text_middle = knuckle_vector_left.ToString();

                            airtap_vector_left = pose_li.Position - pose_lt.Position;

                            if (airtap_vector_left.magnitude < 0.02 && !airtap_left){
                                airtap_left = true;
                                start_position_left = pose_li.Position;
                            }
                            else if (airtap_vector_left.magnitude > 0.04 && airtap_left){
                                airtap_left = false;
                            }

                            else if(airtap_left){
                                double rotation = pose_li.Position[0] - start_position_left[0];
                                double addit = 0;
                                if (rotation < -0.05){
                                    //text_middle = "left";
                                    rot = 1;
                                    addit = 0.02;
                                }
                                else if (rotation > 0.05){
                                   // text_middle = "right";
                                    addit = -0.02;
                                    rot = 2;
                                }
                                else{
                                    //text_middle = "stand";
                                    rot = 0;
                                }
                                cube.transform.rotation = new Quaternion(cube.transform.rotation[0], cube.transform.rotation[1], (float)(cube.transform.rotation[2] + addit), cube.transform.rotation[3]);
                            }
                        }
                        else{
                            text_left = "false";
                            airtap_left = false;
                        }
                    }
                }
           }
        }
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.Wrist, Handedness.Left, out pose_lwrist))
        {
           if (HandJointUtils.TryGetJointPose(TrackedHandJoint.Wrist, Handedness.Right, out pose_rwrist))
           {
               if(!airtap_left && !airtap_right){
                   wrist_vector = pose_lwrist.Position - pose_rwrist.Position;
                   if(wrist_vector.magnitude < 0.1){
                       cube_renderer.material.SetColor("_Color", Color.red);
                       grab = true;
                   }
                   else{
                   cube_renderer.material.SetColor("_Color", Color.green);
                   grab = false;
                    }
               }
               else{
                   cube_renderer.material.SetColor("_Color", Color.green);
                   grab = false;
               }
           }
        }
       value_text.SetText(text_left + " | " + text_middle + " | " + text_right);
    }

    void send(){
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
            if(!reset){
                reset = true;
            }
            Thread.Sleep(33);
            
        }
    }
}
