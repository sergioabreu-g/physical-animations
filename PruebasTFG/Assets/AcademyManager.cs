using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class AcademyManager : MonoBehaviour
{

    [SerializeField] private int _academyFrequency = 1;
    private int academyStepCount = 0;

    void Start()
    {
        Academy.Instance.AutomaticSteppingEnabled = false;
    }

    void FixedUpdate()
    {
        if (academyStepCount >= _academyFrequency)
        {
            Academy.Instance.EnvironmentStep();
            academyStepCount = 0;
        }
        else
            academyStepCount++;
    }
}
