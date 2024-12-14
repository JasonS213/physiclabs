using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsObject : MonoBehaviour
{
    public FizziksShape shape;

    [Range(0.001f, 100000.0f)]
    public float mass = 1.0f;
    [Range(0.0f, 1.0f)]
    public float drag = 0.1f;
    [Range(0.0f, 1.0f)]
    public float grippiness = 0.5f;

    public float gravityScale = 1;
    public Vector3 velocity = Vector3.zero;
    public Vector3 netForce = Vector3.zero;

    public bool isStatic = false;
    
    void Start()
    {
        shape = GetComponent<FizziksShape>();
        PhysicsEngine.Instance.Objects.Add(this);
    }
}
