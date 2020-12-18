using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TorquePDTest : MonoBehaviour
{
    public Transform target;
    private TorquePD _torquePD;
    // Start is called before the first frame update
    void Start()
    {
        _torquePD = GetComponent<TorquePD>();  
    }

    // Update is called once per frame
    void Update()
    {
        _torquePD.targetRot = target.localRotation;
    }
}
