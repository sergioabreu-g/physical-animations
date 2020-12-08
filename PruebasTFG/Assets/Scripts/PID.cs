using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PID : MonoBehaviour
{
    public string name;
    public float Kp = 1.0f, Ki = 0.0f, Kd = 0.1f;
    private float _P = 0, _I = 0, _D = 0;
    private float _previousError = 0;

    public float GetOutput(float currentError, float deltaTime) {
        _P = currentError;

        _I += _P * deltaTime;

        _D = (_P - _previousError) * deltaTime;

        _previousError = currentError;

        return _P * Kp + _I * Ki + _D * Kd;
    }
}
