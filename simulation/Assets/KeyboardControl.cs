using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KeyboardControl : MonoBehaviour
{
    public BalanceBot balanceBot;

    public float pitchStrength = 5.0f;
    public float steerStrength = 25.0f;

    float targetPitch = 0.0f;
    float steerPercentage = 0.0f;

    void Start()
    {
        
    }

    void FixedUpdate()
    {
        targetPitch = 0.0f;
        if (Input.GetKey("w")) {
            targetPitch = pitchStrength;
        } else if (Input.GetKey("s")) {
            targetPitch = -pitchStrength;
        }
        if (balanceBot.targetPitch < targetPitch) {
            balanceBot.targetPitch += 0.4f;
        } else if (balanceBot.targetPitch > targetPitch) {
            balanceBot.targetPitch -= 0.4f;
        }

        steerPercentage = 0.0f;
        if (Input.GetKey("a")) {
            steerPercentage -= steerStrength;
        }
        if (Input.GetKey("d")) {
            steerPercentage += steerStrength;
        }
        if (balanceBot.steerPercentage < steerPercentage) {
            balanceBot.steerPercentage += 5.0f;
        } else if (balanceBot.steerPercentage > steerPercentage) {
            balanceBot.steerPercentage -= 5.0f;
        }
    }
}
