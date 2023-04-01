using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class Buttons : MonoBehaviour
{
    public GameObject panel;
    private bool state = true;

private void Start(){
   panel.SetActive(state);
}  
 public void toggle_disp(){
        state = !state;
        panel.SetActive(state);   
    } 
}


