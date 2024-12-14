using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FizziksShapePlane : FizziksShape
{
    public float size = 1.0f;
    public override Shape GetShape()
    {
        return Shape.Plane;
    }

    public Vector3 GetPosition()
    {
        return transform.position;
    }

    public Vector3 Normal()
    {
        return transform.up;
    }

    private void UpdateScale()
    {
        transform.localScale = new Vector3(size, size, size) * 2f;

    }

}
