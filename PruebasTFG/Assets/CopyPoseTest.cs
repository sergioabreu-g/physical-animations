using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CopyPoseTest : MonoBehaviour
{
    public CharacterAgent agent;
    public Transform[] targets;
    public BodyPart[] bodyParts;

    // Start is called before the first frame update
    void OnValidate()
    {
        bodyParts = GetComponentsInChildren<BodyPart>();
    }

    private void Start() {
        agent.enabled = false;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        for (int i = 0; i < bodyParts.Length; i++) {
            if (bodyParts[i].joint != null)
                bodyParts[i].SetTargetRotation(targets[i].localRotation);
        }
    }
}
