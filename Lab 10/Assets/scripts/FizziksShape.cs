using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class FizziksShape : MonoBehaviour
{
    public enum Shape
    {
        Sphere,
        Plane,
        HalfSpace
    }

    public abstract Shape GetShape();
}
