using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TorquePD : MonoBehaviour
{
    public float frequency = 6, damping = 1, maxTorque = 500;
    public bool localRotation = true;
    public Quaternion targetRot;

    private Rigidbody _rb;
    private float _kp = 1.0f, _kd = 0.1f;
    private Quaternion _fixedTargetRot;

    private void Start() {
        _rb = GetComponent<Rigidbody>();
    }

    public void FixedUpdate() {
        _fixedTargetRot = localRotation ? transform.parent.rotation * targetRot : targetRot;

        _kp = (6f * frequency) * (6f * frequency) * 0.25f;
        _kd = 4.5f * frequency * damping;
        float dt = Time.fixedDeltaTime;
        float g = 1 / (1 + _kd * dt + _kp * dt * dt);
        float ksg = _kp * g;
        float kdg = (_kd + _kp * dt) * g;
        Vector3 x;
        float xMag;

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
        q.ToAngleAxis(out xMag, out x);
        x.Normalize();
        x *= Mathf.Deg2Rad;
        Vector3 pidv = _kp * x * xMag - _kd * _rb.angularVelocity;
        Quaternion rotInertia2World = _rb.inertiaTensorRotation * transform.rotation;
        pidv = Quaternion.Inverse(rotInertia2World) * pidv;
        pidv.Scale(_rb.inertiaTensor);
        pidv = rotInertia2World * pidv;

        _rb.AddTorque(Vector3.ClampMagnitude(pidv, maxTorque));
    }
}
