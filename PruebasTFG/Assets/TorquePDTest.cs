using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TorquePDTest : MonoBehaviour
{
    public Vector3 targetRelativeRotations;
    public bool useTarget = false;
    public Transform target;
    private TorquePD _torquePD;
    private ConfigurableJoint joint;

    // Start is called before the first frame update
    void Start()
    {
        _torquePD = GetComponent<TorquePD>();
        joint = GetComponent<ConfigurableJoint>();
    }

    // Update is called once per frame
    void Update()
    {
        if (useTarget) {
            _torquePD.targetRot = target.localRotation;
            return;
        }

        float x = (targetRelativeRotations.x + 1f) * 0.5f;
        float y = (targetRelativeRotations.y + 1f) * 0.5f;
        float z = (targetRelativeRotations.z + 1f) * 0.5f;

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
}
