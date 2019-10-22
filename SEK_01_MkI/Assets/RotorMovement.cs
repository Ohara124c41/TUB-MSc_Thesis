//=========================================================================================

// Class Name: RotorMovement
// Project: ROSBookWorld
// Author: Elena Peña Tapia
// Purpose: Simulate rotor movement for UAVs
// Script Interactions: none
// GameObject Interactions: 
// GameObject Attached to: 

//=========================================================================================

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotorMovement : MonoBehaviour {

    public float rotorSpeed;
    private float step;
    private Vector3 point;

    void Start () {

        rotorSpeed = 3000;
    }
	
	void FixedUpdate () {

        step = -rotorSpeed * Time.deltaTime;

        Transform rotor2 = transform.Find("Rotor2");

        rotor2.Rotate(Vector3.up, step, Space.Self);

        Transform rotor1 = transform.Find("Rotor1");
      
		rotor1.Rotate(Vector3.up, step, Space.Self);

        Transform rotor3 = transform.Find("Rotor3");

		rotor3.Rotate(Vector3.up, step, Space.Self);

        Transform rotor4 = transform.Find("Rotor4");

		rotor4.Rotate(Vector3.up, step, Space.Self);      
    }
}
