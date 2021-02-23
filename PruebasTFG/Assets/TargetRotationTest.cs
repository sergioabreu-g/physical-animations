using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TargetRotationTest : MonoBehaviour
{
    public Vector3 localTargetRot;
    private ConfigurableJoint joint;

    private void Start() {
        joint = GetComponent<ConfigurableJoint>();
    }

    void FixedUpdate()
    {
        float x = (localTargetRot.x + 1f) / 2;
        float y = (localTargetRot.y + 1f) / 2;
        float z = (localTargetRot.z + 1f) / 2;

        var xRot = Mathf.Lerp(joint.lowAngularXLimit.limit, joint.highAngularXLimit.limit, x);
        var yRot = Mathf.Lerp(-joint.angularYLimit.limit, joint.angularYLimit.limit, y);
        var zRot = Mathf.Lerp(-joint.angularZLimit.limit, joint.angularZLimit.limit, z);

        if (joint.secondaryAxis == Vector3.zero || joint.axis == Vector3.zero)
            Debug.LogWarning("Joint axes cannot be zero.");

        Debug.Log(Quaternion.Inverse(transform.parent.rotation) * transform.rotation);

        joint.targetRotation = Quaternion.Euler(xRot, yRot, zRot);
    }
}
