using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class PhysicsEngine : MonoBehaviour
{
    public struct CollisionInfo
    {
        public bool didCollide;
        public Vector3 normal;

        public CollisionInfo(bool didCollide, Vector3 normal)
        {
            this.didCollide = didCollide;
            this.normal = normal;
        }
    }


    static PhysicsEngine instance = null;
    public static PhysicsEngine Instance
    {
        get
        {
            if (instance == null)
            {
                instance = FindFirstObjectByType<PhysicsEngine>();
            }
            return instance;
        }
    }

    public List<PhysicsObject> Objects = new List<PhysicsObject>();
    public float dt = 0.02f;
    public Vector3 gravityAcceleration = new Vector3(0, -10, 0);

    public Vector3 GetGravityForce(PhysicsObject objekt)
    {
        return gravityAcceleration * objekt.gravityScale * objekt.mass;
    }

    private void KinematicUpdate()

    {
        Debug.Log("Other Hi");
        foreach (PhysicsObject objekt in Objects)
        {
            Vector3 Fg = GetGravityForce(objekt);
            objekt.netForce += Fg;

            //position
            if (!objekt.isStatic) // isStatic is a boolean used to allow objects to move by kinematic or not
            {

                //Vector3 vSquared = objekt.velocity.normalized * objekt.velocity.magnitude * objekt.velocity.magnitude;
                //Vector3 dragForce = -objekt.drag * vSquared; // -c * V^2
                // Add drag
                //objekt.netForce += dragForce;

                // velocity update according to gravity acceleration
                Vector3 accelerationOnThisFrame = objekt.netForce / objekt.mass;
                // a = F/m


                //Apply acceleration
                objekt.velocity += accelerationOnThisFrame * dt;
                Vector3 newPos = objekt.transform.position + objekt.velocity * dt;
                objekt.transform.position = newPos;

                //debug drawing
                //Velocity
                Debug.DrawRay(objekt.transform.position, objekt.velocity, Color.red);
                // Force of Gravity
                Debug.DrawRay(objekt.transform.position, Fg, new Color(0.5f, 0.0f, 0.4f));
            }

            objekt.netForce = Vector3.zero;
        }
    }

    private void CollisionUpdate()
    {
 
        for (int iA = 0; iA < Objects.Count; iA++) // N 
        {
            PhysicsObject objectA = Objects[iA];

            for (int iB = iA + 1; iB < Objects.Count; iB++) // N - ???
            {
                
                PhysicsObject objectB = Objects[iB];

                PhysicsObject objectCorrectedA = objectA;
                PhysicsObject objectCorrectedB = objectB;

                //Check for collisions between objects

                CollisionInfo collisioninfo = new CollisionInfo(false, Vector3.zero);

                if (objectA.shape.GetShape() == FizziksShape.Shape.Sphere && objectB.shape.GetShape() == FizziksShape.Shape.Sphere)
                {
                    collisioninfo = CollisionResponseSphereSphere((FizziksShapeSphere)objectA.shape, (FizziksShapeSphere)objectB.shape);

                }
                else if (objectA.shape.GetShape() == FizziksShape.Shape.Sphere && objectB.shape.GetShape() == FizziksShape.Shape.Plane)
                {
                    collisioninfo = CollisionResponseSpherePlane((FizziksShapeSphere)objectA.shape, (FizziksShapePlane)objectB.shape);
                }
                else if (objectA.shape.GetShape() == FizziksShape.Shape.Plane && objectB.shape.GetShape() == FizziksShape.Shape.Sphere)
                {

                    objectCorrectedA = objectB;
                    objectCorrectedB = objectA;
                    collisioninfo = CollisionResponseSpherePlane((FizziksShapeSphere)objectCorrectedA.shape, (FizziksShapePlane)objectCorrectedB.shape);
                }

                if (collisioninfo.didCollide)
                {
                    //colliding!!
                    //change color of objects that collide to red
                    //objectB.GetComponent<Renderer>().material.color = Color.red;
                    //objectA.GetComponent<Renderer>().material.color = Color.red;

                    //Calculate gravity force
                    Vector3 Fg = GetGravityForce(objectCorrectedA);
                    //Calculate prependicular components of gravity by vector projection of gravity force onto normal
                    float gravityDotNormal = Vector3.Dot(Fg, collisioninfo.normal);
                    Vector3 gravityProjectedNormal = collisioninfo.normal * gravityDotNormal;

                    //Add normal force due to gravity
                    Vector3 Fn = -gravityProjectedNormal;
                    Debug.DrawRay(objectA.transform.position, Fn, Color.green);

                    objectA.netForce += Fn;
                    objectB.netForce -= Fn;

                    //Calculate relative velocity to determine if we should apply kinetic friction or not
                    Vector3 velARelativeToB = objectCorrectedA.velocity - objectCorrectedB.velocity;
                    //project relative velocity onto the surface (e.g. subtract perpendicular component to a plane)
                    float velDotNormal = Vector3.Dot(velARelativeToB, collisioninfo.normal);
                    Vector3 velProjectedNormal = collisioninfo.normal * velDotNormal; // part of velocity aligned along the normal axis

                    // Subtract the part of velocity aligned with the normal axis to be left with the velocity projected along the plane perpendicular to that collision normal
                    Vector3 velARelativeToBProjectedOntoPlane = velARelativeToB - velProjectedNormal; // in-plane relative motion between A and B


                    if (velARelativeToBProjectedOntoPlane.sqrMagnitude > 0.00001)// if not zero.... do friction
                    {
                        //magnitude of friction is coeffient of friction times normal force magnitude
                        float coefficientOfFriction = Mathf.Clamp01(objectCorrectedA.grippiness * objectCorrectedB.grippiness);
                        float frictionMagnitude = Fn.magnitude * coefficientOfFriction;
                        Vector3 Ff = -velARelativeToBProjectedOntoPlane.normalized * frictionMagnitude;

                        Debug.DrawRay(objectA.transform.position, Ff, new Color(0.8f, 0.6f, 0.0f));

                        objectCorrectedA.netForce += Ff;
                        objectCorrectedB.netForce -= Ff;
                        //Ff = u * ||Fn|| * -(vYouRelativeToBus/ ||vYouRelativeToBus||
                    }

                    //Bouncing/Applying impulse from collision
                    if (velDotNormal < 0) // if they are moving toward each other, do something else don't (From notes step 2)
                    {
                        //Bounce!
                        //Determine coefficient of restitution
                        float restitution = 0.8f;

                        if (velDotNormal > -1)//If the relative velocity is really small, even though they are moving toward each other, let's help dampen the motion 
                        {
                            restitution = 0;
                        }
                        else 
                        {
                            restitution = (objectCorrectedA.Bounciness * objectCorrectedB.Bounciness);
                        }


                        float deltaV1D = (1.0f + restitution) * velDotNormal;

                        //From notes step 4: Impulse = (1 + restitution) * Dot(vlRe12, N) * m1 * m2/ (m1 + m2)
                        float impulse1D = deltaV1D * objectCorrectedA.mass * objectCorrectedB.mass / (objectCorrectedA.mass + objectCorrectedB.mass);

                        //Impulse is in the direction of the collision normal
                        Vector3 impulse3D = collisioninfo.normal * impulse1D;

                        Debug.DrawRay(objectCorrectedA.transform.position, impulse3D, Color.cyan, 0.2f, false);

                        //Apply change in velocity based on impulse, but in the opposite direction for each objects
                        objectCorrectedA.velocity += -impulse3D / objectCorrectedA.mass;
                        objectCorrectedB.velocity += impulse3D / objectCorrectedB.mass;
                        Debug.Log("HI");
                    }

                }

            }
        }
    }

    void FixedUpdate()
    {
        //foreach (PhysiqsObject objekt in Objects)
        //{
        //    objekt.GetComponent<Renderer>().material.color = Color.white;
        //}
        CollisionUpdate();
        KinematicUpdate();


    }

    public static CollisionInfo CollisionResponseSphereSphere(FizziksShapeSphere sphereA, FizziksShapeSphere sphereB)
    {
        Vector3 Displacement_BtoA = sphereA.transform.position - sphereB.transform.position;
        float distance = Displacement_BtoA.magnitude;
        float overlap = (sphereA.radius + sphereB.radius) - distance;

        if (overlap < 0.0f) // if not overlapping
        {
            return new CollisionInfo(false, Vector3.zero); //Exit early
        }


        Vector3 collisionNormal_BToA;

        if (distance <= 0.000001f)
        {
            collisionNormal_BToA = Vector3.up;
        }
        else
        {
            collisionNormal_BToA = Displacement_BtoA / distance;
        }

        Vector3 mtv = collisionNormal_BToA * overlap;
        // points from B to A
        sphereA.transform.position += mtv * 0.5f;
        sphereB.transform.position -= mtv * 0.5f;

        return new CollisionInfo(true, collisionNormal_BToA);

    }

    public static CollisionInfo CollisionResponseSpherePlane(FizziksShapeSphere sphere, FizziksShapePlane plane)
    {
        Vector3 fromPointToSphere = sphere.transform.position - plane.transform.position;
        float positionAlongNormal = Vector3.Dot(fromPointToSphere, plane.Normal());

        float distanceToPlane = Mathf.Abs(positionAlongNormal);

        float overlap = sphere.radius - distanceToPlane;

        if (overlap < 0.0f) // if not overlapping
        {
            return new CollisionInfo(false, Vector3.zero); //Exit early
        }

        sphere.transform.position += plane.Normal() * (sphere.radius - positionAlongNormal);
        return new CollisionInfo(true, plane.Normal()); ;


    }


    public static bool IsOverLappingSpheres(PhysicsObject sphereA, PhysicsObject sphereB)
    {
        Vector3 Displacement = sphereA.transform.position - sphereB.transform.position;
        float distance = Displacement.magnitude;
        float radiusA = ((FizziksShapeSphere)sphereA.shape).radius;
        float radiusB = ((FizziksShapeSphere)sphereB.shape).radius;
        return distance < radiusA + radiusB;
    }

    public static bool IsOverLappingSpherePlane(FizziksShapeSphere sphere, FizziksShapePlane plane)
    {
        // subtract plane point from sphere center to get displacement
        Vector3 planeToSphere = sphere.transform.position - plane.transform.position;
        //Take a Dot product of normal and displacement to get position of sphere center along 
        float positionAlongNormal = Vector3.Dot(planeToSphere, plane.Normal());
        // if the absolute value of this dot product is less than the radius of the sphere, then 
        float distanceToPlane = Mathf.Abs(positionAlongNormal);
        return distanceToPlane < sphere.radius;
    }

}