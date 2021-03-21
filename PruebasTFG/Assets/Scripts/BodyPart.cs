using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BodyPart : MonoBehaviour
{
    public Rigidbody rb;
    public Rigidbody animatedEquivalent;
    public ConfigurableJoint joint;
    public bool endEffector = false;
    public bool canTouchGround = false;

    [HideInInspector]
    public Quaternion initialRotation;
    [HideInInspector]
    public Vector3 initialPosition;
    [HideInInspector]
    public float initialAngularXDriveSpring;
    [HideInInspector]
    public float initialAngularYZDriveSpring;

    public bool touchingGround = false;
    private int touchCount = 0;

    public void Start() {
        initialRotation = transform.localRotation;
        initialPosition = transform.position;

        if (joint != null) {
            initialAngularXDriveSpring = joint.angularXDrive.positionSpring;
            initialAngularYZDriveSpring = joint.angularYZDrive.positionSpring;
        }
    }

    public void Reset(bool setToCurrentAnimationFrame = true) {
        rb.angularVelocity = Vector3.zero;
        rb.velocity = Vector3.zero;

        if (setToCurrentAnimationFrame) {
            transform.localRotation = animatedEquivalent.transform.localRotation;
            transform.position = animatedEquivalent.transform.position;

            if (joint != null) {
                SetTargetRotation(animatedEquivalent.transform.localRotation);
            }
        }
        else {
            transform.localRotation = initialRotation;
            transform.position = initialPosition;

            if (joint != null) {
                SetTargetRotation(initialRotation);
            }
        }
    }

    public void SetTargetRotation(float x, float y, float z) {
        x = (x + 1f) / 2;
        y = (y + 1f) / 2;
        z = (z + 1f) / 2;

        var xRot = Mathf.Lerp(joint.lowAngularXLimit.limit, joint.highAngularXLimit.limit, x);
        var yRot = Mathf.Lerp(-joint.angularYLimit.limit, joint.angularYLimit.limit, y);
        var zRot = Mathf.Lerp(-joint.angularZLimit.limit, joint.angularZLimit.limit, z);

        joint.targetRotation = Quaternion.Euler(xRot, yRot, zRot);
    }

    public void SetTargetRotation(Quaternion rotation) {
        ConfigurableJointExtensions.SetTargetRotationLocal(joint, rotation, initialRotation);
    }

    public Vector3 GetJointNormalizedRotation() {
        Vector3 jointRot = transform.localRotation.eulerAngles;

        if (joint == null)
            return jointRot;

        jointRot.x = Mathf.InverseLerp(joint.lowAngularXLimit.limit, joint.highAngularXLimit.limit, jointRot.x);
        jointRot.y = Mathf.InverseLerp(-joint.angularYLimit.limit, joint.angularYLimit.limit, jointRot.y);
        jointRot.z = Mathf.InverseLerp(-joint.angularZLimit.limit, joint.angularZLimit.limit, jointRot.z);

        return jointRot;
    }

    void OnCollisionEnter(Collision collision) {
        if (!collision.transform.TryGetComponent<BodyPart>(out BodyPart bodypart)) {
            touchCount++;
            touchingGround = true;
        }
    }

    void OnCollisionExit(Collision collision) {
        if (!collision.transform.TryGetComponent<BodyPart>(out BodyPart bodypart)) {
            touchCount--;
            touchingGround = touchCount > 0;
        }
    }
}
