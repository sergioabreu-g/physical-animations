using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CopyPoseTest : MonoBehaviour
{
    public CharacterAgent agent;
    public BodyPart[] bodyParts;

    public float timeBetweenDebugs = 0.5f;
    private float _debugCounter = 0;

    // Start is called before the first frame update
    void OnValidate()
    {
        bodyParts = GetComponentsInChildren<BodyPart>();
        agent = GetComponent<CharacterAgent>();
    }

    private void Start() {
        agent.enabled = false;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        for (int i = 0; i < bodyParts.Length; i++) {
            if (bodyParts[i].joint != null)
                bodyParts[i].SetTargetRotation(bodyParts[i].animatedEquivalent.transform.localRotation);
        }

        if (_debugCounter >= timeBetweenDebugs)
        {
            Debug.Log(agent.CalculateTotalReward());
            _debugCounter = 0;
        }
        else
            _debugCounter += Time.fixedDeltaTime;
    }
}
