using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BalanceBot : MonoBehaviour
{
    public GameObject imuOrigin;
    public WheelMotor leftMotor;
    public WheelMotor rightMotor;

    public float targetPitch = 0.0f;
    public float steerPercentage = 0.0f;
    float pitch = 0.0f;
    float err = 0.0f;
    float errLast = 0.0f;
    float errD = 0.0f;
    float errI = 0.0f;
    public float errISaturation = 45.0f;

    public float kp = 0.0f;
    public float ki = 0.0f;
    public float kd = 0.0f;

    void Start()
    {
        
    }

    void FixedUpdate() {
        // Read pitch from "IMU" object
        pitch = Mathf.Asin(imuOrigin.transform.forward.y) * Mathf.Rad2Deg;

        // Run PID loop
        errLast = err;
        err = targetPitch - pitch;
        errD = (err - errLast) / Time.fixedDeltaTime;

        errI = Mathf.Clamp(errI + (err * Time.fixedDeltaTime), -errISaturation, errISaturation);
        float output = (err * kp) + (errD * kd) + (errI * ki);

        // Give up if pitch is too large
        if (pitch >= 45.0f || pitch <= -45.0f) {
            output = 0.0f;
        }

        // Set output value for virtual motors
        float outputLeft = output + (output * (steerPercentage * 0.01f));
        float outputRight = output + (output * (steerPercentage * -0.01f));

        leftMotor.targetRpm = outputLeft;
        rightMotor.targetRpm = outputRight;
    }

    void OnGUI()
    {
        GUIStyle style = new GUIStyle();
        style.fontSize = 24;
        GUI.Label(new Rect(10, 10, 0, 0), "Current pitch: " + pitch, style);
        GUI.Label(new Rect(10, 40, 0, 0), "Target pitch: " + targetPitch, style);
        GUI.Label(new Rect(10, 70, 0, 0), "Err: " + err, style);
        GUI.Label(new Rect(10, 100, 0, 0), "ErrD: " + errD, style);
        GUI.Label(new Rect(10, 130, 0, 0), "ErrI: " + errI, style);
    }
}
