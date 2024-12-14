using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AngryBirds : MonoBehaviour
{
    public float angleDegrees = 30;
    public float speed = 100;
    public float startHeight = 1;

    public GameObject projectileToCopy;


    // Update is called once per frame
    void Update()
    {
        Vector3 launchVelocity = new Vector3(angleDegrees, speed);
        Vector3 startPosition = new Vector3(0, startHeight, 0);


        if (Input.GetKeyDown(KeyCode.Space)) // if space  is pressed
        {
            Debug.Log("Launched!");
            GameObject newObject = Instantiate(projectileToCopy);
            PhysicsObject physicsObject = newObject.GetComponent<PhysicsObject>();


            physicsObject.velocity = launchVelocity;

            physicsObject.transform.position = startPosition;

        }
        Debug.DrawLine(startPosition, startPosition + launchVelocity, Color.red, 2);
    }
}
