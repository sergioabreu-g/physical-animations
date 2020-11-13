using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BodyPart : MonoBehaviour
{
    public Rigidbody rb;
    public ConfigurableJoint joint;
    public Transform animatedPeer;

    public Quaternion initialRotation;
    public Vector3 initialPosition;
    public float initialAngularXDriveSpring;
    public float initialAngularYZDriveSpring;

    public bool touchingGround = false;
    private int touchCount = 0;

    private float _relativeStrength = 1;
    public float RelativeStrength { get { return _relativeStrength; } 
        set {
            if (_relativeStrength == value) return;
            _relativeStrength = value;
            SetJointRelativeStrength(value);
        } }

    public void Start() {
        initialRotation = transform.localRotation;
        initialPosition = transform.position;

        if (joint != null) {
            initialAngularXDriveSpring = joint.angularXDrive.positionSpring;
            initialAngularYZDriveSpring = joint.angularYZDrive.positionSpring;
        }
    }
    
    public void Reset() {
        rb.angularVelocity = Vector3.zero;
        rb.velocity = Vector3.zero;
        rb.transform.localRotation = initialRotation;
        rb.transform.position = initialPosition;

        if (joint != null) {
            joint.targetRotation = Quaternion.identity;
            SetJointRelativeStrength(1);
        }
    }

    private void SetJointRelativeStrength(float strength) {
        JointDrive temp = joint.angularXDrive;
        temp.positionSpring = strength * initialAngularXDriveSpring;
        joint.angularXDrive = temp;

        temp = joint.angularYZDrive;
        temp.positionSpring = strength * initialAngularYZDriveSpring;
        joint.angularYZDrive = temp;
    }

    /*
    public void SetTargetRotation(float x, float y, float z) {
        x = (x + 1f) * 0.5f;
        y = (y + 1f) * 0.5f;
        z = (z + 1f) * 0.5f;

        var xRot = Mathf.Lerp(joint.lowAngularXLimit.limit, joint.highAngularXLimit.limit, x);
        var yRot = Mathf.Lerp(-joint.angularYLimit.limit, joint.angularYLimit.limit, y);
        var zRot = Mathf.Lerp(-joint.angularZLimit.limit, joint.angularZLimit.limit, z);

        var targetRot = Quaternion.Euler(xRot, yRot, zRot);
        joint.targetRotation = targetRot;
    }*/

    void OnCollisionEnter(Collision collision) {
        if (!collision.transform.TryGetComponent<BodyPart>(out BodyPart bodypart)) {
            touchCount++;
            touchingGround = true;
        }
    }

    void OnCollisionExit(Collision collision) {
        if (!collision.transform.TryGetComponent<BodyPart>(out BodyPart bodypart)) {
            touchCount--;
            touchingGround = touchCount == 0;
        }
    }
}
