using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(TorquePD))]
public class BodyPart : MonoBehaviour
{
    public Rigidbody rb;
    public ConfigurableJoint joint;
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

    private float _relativeStrength = 1;
    private TorquePD _torquePD;

    public void Start() {
        _torquePD = GetComponent<TorquePD>();

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
            SetTargetRotation(0, 0, 0);
        }

        touchCount = 0;
        touchingGround = false;
    }

    public void SetTargetRotation(float x, float y, float z) {
        x = (x + 1f) * 0.5f;
        y = (y + 1f) * 0.5f;
        z = (z + 1f) * 0.5f;

        var xRot = Mathf.Lerp(joint.lowAngularXLimit.limit, joint.highAngularXLimit.limit, x);
        var yRot = Mathf.Lerp(-joint.angularYLimit.limit, joint.angularYLimit.limit, y);
        var zRot = Mathf.Lerp(-joint.angularZLimit.limit, joint.angularZLimit.limit, z);

        if (joint.secondaryAxis == Vector3.zero || joint.axis == Vector3.zero)
            Debug.LogWarning("Joint axes cannot be zero.");

        Vector3 thirdAxis = Vector3.Cross(joint.axis, joint.secondaryAxis).normalized;
        Vector3 transformedRotation = joint.axis.normalized * xRot
                                    + joint.secondaryAxis.normalized * yRot
                                    + thirdAxis * zRot;

        _torquePD.targetRot = Quaternion.Euler(transformedRotation);
    }

    public void SetTargetRotation(Quaternion rotation) {
        _torquePD.targetRot = rotation;
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
