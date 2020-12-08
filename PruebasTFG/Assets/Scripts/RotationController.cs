using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class RotationController : MonoBehaviour
{
    public float turnSpeed; 

    public PID anglePID;
    public PID velocityPID;

    public Vector3 _targetRot;
    private Vector3 _torque;

    private Rigidbody _rb;

    private void OnValidate() {
        var PIDs = GetComponents<PID>();
        for (int i = 0; i < 2 - PIDs.Length; i++) {
            gameObject.AddComponent<PID>();
        }
        PIDs = GetComponents<PID>();

        anglePID = PIDs[0];
        anglePID.name = "AngleController";

        velocityPID = PIDs[1];
        velocityPID.name = "VelocityController";
    }

    void Start() {
        _targetRot = transform.localEulerAngles;
        _rb = GetComponent<Rigidbody>();
        _rb.maxAngularVelocity = 50;
    }

    void FixedUpdate() {
        float dt = Time.fixedDeltaTime;

        float angleError = Quaternion.Angle(Quaternion.Euler(transform.localEulerAngles), Quaternion.Euler(_targetRot));
        float torqueCorrectionForAngle = anglePID.GetOutput(angleError, dt);

        float angularVelocityError = _rb.angularVelocity.magnitude;
        float torqueCorrectionForAngularVelocity = velocityPID.GetOutput(angularVelocityError, dt);

        _torque = (_targetRot - transform.localEulerAngles).normalized * (torqueCorrectionForAngle + torqueCorrectionForAngularVelocity);
        _rb.AddRelativeTorque(_torque);
    }
}
