using UnityEngine;
using System.Collections;
using System.Collections.Generic;

[System.Serializable]
public class AxleInfo {
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor;
    public bool steering;
}

public class RC : MonoBehaviour {
    public List<AxleInfo> axleInfos;
    public float maxMotorTorque;
    //public float maxSteeringAngle;

    Quaternion BASE_WHEEL_ROTATION = Quaternion.Euler(0, 0, 90);

    public void ApplyLocalPositionToVisuals(WheelCollider collider) {
        if (collider.transform.childCount == 0) {
            return;
        }

        Transform visualWheel = collider.transform.GetChild(0);

        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation * BASE_WHEEL_ROTATION;
    }
    private void colorWheel(WheelCollider wc, float torque) {
        var mr = wc.transform.GetComponentInChildren<MeshRenderer>();
        var color = Color.white;
        if (torque > 0) { color = Color.green; }
        if (torque < 0) { color = Color.red; }
        mr.material.color = color;
    }
    public void FixedUpdate() {
        float motor = maxMotorTorque * Input.GetAxis("Vertical");
        float steering = -maxMotorTorque * Input.GetAxis("Horizontal");

        foreach (AxleInfo axleInfo in axleInfos) {
            //if (axleInfo.steering) {
            //    axleInfo.leftWheel.steerAngle = steering;
            //    axleInfo.rightWheel.steerAngle = steering;
            //}
            if (axleInfo.motor) {
                axleInfo.leftWheel.motorTorque = Mathf.Clamp(motor + steering, -maxMotorTorque, maxMotorTorque);
                axleInfo.rightWheel.motorTorque = Mathf.Clamp(motor - steering, -maxMotorTorque, maxMotorTorque);
                colorWheel(axleInfo.leftWheel, axleInfo.leftWheel.motorTorque);
                colorWheel(axleInfo.rightWheel, axleInfo.rightWheel.motorTorque);

            }
            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        }
    }
}