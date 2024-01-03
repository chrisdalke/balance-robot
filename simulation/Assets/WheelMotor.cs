using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WheelMotor : MonoBehaviour
{
    Rigidbody rb;
    public float targetRpm = 0f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        
    }

    void FixedUpdate()
    {
        rb.AddRelativeTorque(new Vector3(targetRpm, 0f, 0f), ForceMode.Force);
    }
}
