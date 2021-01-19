using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ConstantRotation : MonoBehaviour
{
    public Vector3 angularVel;

    // Start is called before the first frame update
    void FixedUpdate()
    {
        GetComponent<Rigidbody>().angularVelocity = angularVel;
    }
}
