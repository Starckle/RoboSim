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
public enum OperationalModes { Manual, Semi, Autonomous }
public class RC : MonoBehaviour {
    public List<AxleInfo> axleInfos;
    public UltraS[] sensors;
    public OperationalModes autonomous;
    public float maxMotorTorque;
    //public float maxSteeringAngle;

    public string ForwardAxis = "Vertical";
    public string RotateAxis = "Horizontal";
    public string StrafeAxis = "Yaw";
    public string FlickButton = "joystick button 0";
    public HingeJoint Flicker = null;

    Quaternion[] BASE_WHEEL_ROTATIONS = new[]{
        Quaternion.Euler(0, -45, 0),
        Quaternion.Euler(0, 45, 0)
    };

    public void ApplyLocalPositionToVisuals(WheelCollider collider, int index) {
        if (collider.transform.childCount == 0) {
            return;
        }

        Transform visualWheel = collider.transform.GetChild(0);

        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        visualWheel.transform.position = position;
        visualWheel.transform.rotation = BASE_WHEEL_ROTATIONS[index] * rotation;
    }
    private void colorWheel(WheelCollider wc, float torque) {
        var mr = wc.transform.GetComponentInChildren<MeshRenderer>();
        var color = new Color(1,1,0);
        if (torque > 0) { color.r = 1 - torque/ maxMotorTorque; }
        if (torque < 0) { color.g = 1 + torque/maxMotorTorque; }
        mr.material.color = color;
    }
    public void Update() {
        if (Input.GetKeyDown(FlickButton)) {
            Flicker.useSpring = true;
        }
        if (Input.GetKeyUp(FlickButton)) {
            Flicker.useSpring = false;
        }
    }
    float FORWARD_SPEED = 1f;
    float ROTATE_SPEED = 1f;
    float STRAFE_SPEED = 1.0f;
    public void FixedUpdate() {

        float forward = 0;
        float rotate = 0;
        float strafe = 0;
        if (autonomous != OperationalModes.Autonomous) {
            forward = maxMotorTorque * Input.GetAxis(ForwardAxis) * FORWARD_SPEED;
            rotate = -maxMotorTorque * Input.GetAxis(RotateAxis) * ROTATE_SPEED;
            strafe = maxMotorTorque * Input.GetAxis(StrafeAxis) * STRAFE_SPEED;
        }
        if (autonomous == OperationalModes.Autonomous) {
            Flicker.useSpring = true; 
            if (sensors.Length > 0) {
                forward = maxMotorTorque * 0.2f * FORWARD_SPEED;
                bool turn = sensors[2].distance < 2f;
                if (turn) {
                    forward = 0;
                    rotate = -maxMotorTorque * 1f * ROTATE_SPEED;
                }
            }
        }

        int axle_index = 0;
        var strafeScale = 1;
        foreach (AxleInfo axleInfo in axleInfos) {
            //if (axleInfo.steering) {
            //    axleInfo.leftWheel.steerAngle = steering;
            //    axleInfo.rightWheel.steerAngle = steering;
            //}
            if (axleInfo.motor) {
                axleInfo.leftWheel.motorTorque = Mathf.Clamp(forward + rotate + strafe * strafeScale, -maxMotorTorque, maxMotorTorque);
                axleInfo.rightWheel.motorTorque = Mathf.Clamp(forward - rotate + strafe * strafeScale, -maxMotorTorque, maxMotorTorque);
                colorWheel(axleInfo.leftWheel, axleInfo.leftWheel.motorTorque);
                colorWheel(axleInfo.rightWheel, axleInfo.rightWheel.motorTorque);

            }
            ApplyLocalPositionToVisuals(axleInfo.leftWheel, axle_index);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel, axle_index);

            axle_index++;
            strafeScale = strafeScale * -1;
        }
    }
}