using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Ball : MonoBehaviour
{
    public float lifeTime;
    // Start is called before the first frame update
    void Start()
    {
    }

    private void OnEnable()
    {
        Invoke("SetInactive", lifeTime);
    }

    void SetInactive() { gameObject.SetActive(false); }

}
