using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/**
 * 
 * Controller for fleet.
 * 
 * Juan Jesús Roldán: jj.roldan@upm.es
 * 
 **/


public class Robot {
	public int id;
	public Vector3 position;
	public Vector3 rotation;
	public float time;
	public int spawn;
	public int check;
	public GameObject model;

	public Robot()
	{
		id = 0;
		position = new Vector3 ();
		rotation = new Vector3 ();
		time = Time.time;
		spawn = 0;
		check = 0;
		model = null;
	}
}

public class Target{
	public int id;
	public Vector3 position;
	public Vector3 rotation;
	public float time;
	public int check;

	public Target()
	{
		id = 0;
		position = new Vector3 ();
		rotation = new Vector3 ();
		time = Time.time;
		check = 1;
	}
}

public class FleetController : MonoBehaviour {

	public GameObject groundRobot;
	public GameObject aerialRobot;

	public GameObject selection;

	public GameObject selectedRobot;

	public Robot[] robots;

	public Target[] targets;

	public int destroyTime;

	void Start () {

		robots = new Robot[10];

		for (int i = 0; i < robots.Length; i++) {
			robots [i] = new Robot ();
		}

		targets = new Target[10];

		for (int i = 0; i < targets.Length; i++) {
			targets [i] = new Target ();
		}
	}
	
	void Update () {

		for (int i = 0; i < robots.Length; i++) {
			
			if (robots [i].id > 0) {
				// Robot?
			
				// Spawn robots.

				if (robots [i].spawn == 0) {
					// New robot?

					GameObject clone;

					if (robots [i].id < 100) {
						// Ground robot?
						clone = Instantiate (groundRobot, robots [i].position, Quaternion.Euler(robots [i].rotation));
					} else {
						// Aerial robot?
						clone = Instantiate (aerialRobot, robots [i].position, Quaternion.Euler(robots [i].rotation));
					}

					robots [i].model = clone;

					Debug.Log ("ROBOT Spawned: ID-" + robots[i].id);

					robots [i].check = 1;
					robots [i].spawn = 1;
				}

				// Move robots.

				if (robots [i].check == 0) {

					if (Random.Range (0, 10) > 5) {
						robots [i].model.GetComponent<Rigidbody> ().MovePosition (robots [i].position);
					} else {
						robots [i].model.transform.Rotate(new Vector3(0,90,0) - robots [i].rotation - robots[i].model.transform.localRotation.eulerAngles);
					}

					robots [i].check = 1;
				}

				// Remove robots.

				if (Time.time - robots [i].time > destroyTime) {
					// Inactive robot?

					Destroy (robots [i].model, 1.0f);

					Debug.Log ("ROBOT Destroyed: ID-" + robots[i].id);

					robots [i] = new Robot ();
				}
			}

			if ((selectedRobot != null)&&(robots [i].model == selectedRobot)) {

				selection.transform.Translate(selectedRobot.transform.position - selection.transform.position);
			}
		}
	}
}
