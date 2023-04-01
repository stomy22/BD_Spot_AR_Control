using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Diagnostics;

using UnityEngine;
using UnityEngine.UI;
using TMPro;


public class Live : MonoBehaviour
{
    class StreamInfo
    {
        public int portNuk { get; set; }
        public int portHololens { get; set; }
    }
    
    Dictionary<String, StreamInfo> StreamDict = new Dictionary<String, StreamInfo>()
    {
        { "front_left", new StreamInfo {portNuk = 62610, portHololens=62600} },
        { "front_right", new StreamInfo {portNuk = 62611, portHololens=62601} },
        { "left", new StreamInfo {portNuk = 62612, portHololens=62602} },
        { "arm", new StreamInfo {portNuk = 62613, portHololens=62603} },
        { "right", new StreamInfo {portNuk = 62614, portHololens=62604} },
        { "back", new StreamInfo {portNuk = 62615, portHololens=62605} }
    };

    public String StreamName;
    public String ipNuk;

    private int portNuk;
    private int portHololens;

    public RawImage image;
    public TextMeshPro fps_text;
    private Texture2D tex;

    private byte[] imageBuffer = new byte[1000000];
    private byte[] readyImg = new byte[1000000];

    private int dgSize = 60000;
    private int headerSize = 8;

    bool thread_running = false;
    bool updateImg = false;

    UdpClient client;
    IPEndPoint serverEndPoint;
    IPEndPoint remoteEndPoint = null;

    Stopwatch sw2 = new Stopwatch();
    private int[] fps = new int[20];

    private void initSocket(){
        client = new UdpClient(portHololens);
        serverEndPoint = new IPEndPoint(IPAddress.Parse(ipNuk), portNuk);
        client.Connect(serverEndPoint);
        var message = new byte[]{ 0x00, 0x00};
        client.Send(message, message.Length);
    }

    private void startReceivingThread(){
        thread_running = true;
        Thread receivingThread = new Thread(receive);
        receivingThread.IsBackground = true;
        receivingThread.Start();
    }
    private void receive(){
        byte[] bytes = new byte[dgSize + headerSize];
        int imgSize = 0;
        int sliceNum;
        int ImgSliceSize;
        int offset;

        while (thread_running){
            bytes = client.Receive(ref remoteEndPoint);
            sliceNum = BitConverter.ToInt32(bytes, 0);
            ImgSliceSize = BitConverter.ToInt32(bytes, 4);
            
            if(sliceNum == 0){
               Array.Clear(readyImg, 0, readyImg.Length);
               Array.Copy(imageBuffer, 0, readyImg, 0, imgSize);
               imgSize = ImgSliceSize;
               updateImg = true;
            }
            offset = dgSize * sliceNum;
            Array.Copy(bytes, headerSize, imageBuffer, sliceNum, bytes.Length-headerSize);
        }
    }
    public int getAverage(int[] values){
        int sum = 0;
        foreach (int item in values)
        {
            sum += item;
        }
        return sum/values.Length;
    }
    public void insertNewValue(int item){
        item = 1000/item;
        for(int i=0; i<fps.Length-1; i++){
            fps[i] = fps[i+1];
        }
        fps[fps.Length-1] = item;
    }

    public void Start()
    {
        tex = new Texture2D(2, 2);
    
        image.texture = tex;
        portNuk = StreamDict[StreamName].portNuk; 
        portHololens = StreamDict[StreamName].portHololens;

        initSocket();
        startReceivingThread(); 

        UnityEngine.Debug.Log(StreamName + " started");
        sw2.Start();
    }
    private void Update()
    {
        if(updateImg){
            tex.LoadImage(readyImg);
            tex.Apply();
            updateImg = false;
            insertNewValue((int)sw2.ElapsedMilliseconds);
            fps_text.SetText("FPS: " + getAverage(fps).ToString());
            sw2.Restart();
        }
    }
}