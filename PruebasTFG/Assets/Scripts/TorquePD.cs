using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TorquePD : MonoBehaviour {
    public bool localRotation = true;
    public Quaternion targetRot;

    public float rotP = 1000, rotI = 0, rotD = 1500, rotIRate = 0;
    public float velP = 0.05f, velI = 0, velD = 0.001f, velIRate = 0;

    private Vector3 rotPreviousError, rotIntegral = Vector3.zero;
    private Vector3 velPreviousError, velIntegral = Vector3.zero;

    public float maxTorque = 500;

    private Rigidbody _rb;
    private Quaternion _fixedTargetRot;

    private void Start() {
        _rb = GetComponent<Rigidbody>();
    }

    public void FixedUpdate() {
        Vector3 rotCorrection = RotationPID();
        Vector3 velCorrection = AngularVelocityPID();
        Vector3 finalTorque = rotCorrection + velCorrection;
        finalTorque *= Time.fixedDeltaTime;

        _rb.AddTorque(Vector3.ClampMagnitude(rotCorrection + velCorrection, maxTorque));
    }

    private Vector3 RotationPID() {
        _fixedTargetRot = localRotation ? transform.parent.rotation * targetRot : targetRot;

        Quaternion q = _fixedTargetRot * Quaternion.Inverse(transform.rotation);
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (q.w < 0) {
            // Convert the quaterion to eqivalent "short way around" quaterion
            q.x = -q.x;
            q.y = -q.y;
            q.z = -q.z;
            q.w = -q.w;
        }
        q.ToAngleAxis(out float xMag, out Vector3 error);
        error.Normalize();
        error *= Mathf.Deg2Rad;
        error *= xMag;
        rotIntegral += error * rotIRate;
        Vector3 pidv = rotP * error + rotI * rotIntegral + rotD * (error - rotPreviousError);


        Quaternion rotInertia2World = _rb.inertiaTensorRotation * _rb.rotation;
        pidv = Quaternion.Inverse(rotInertia2World) * pidv;
        pidv.Scale(_rb.inertiaTensor);
        pidv = rotInertia2World * pidv;

        rotPreviousError = error;

        return pidv;
    }

    private Vector3 AngularVelocityPID() {
        Vector3 error = -_rb.angularVelocity;
        velIntegral += error * velIRate;
        Vector3 angularVelCorrection = velP * error + velI * velIntegral + velD * (error - velPreviousError);

        velPreviousError = error;
        return angularVelCorrection;
    }
}