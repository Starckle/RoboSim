using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;

[System.Serializable]
public class AxleInfo {
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor;
    public bool steering;
}
public enum OperationalModes { Manual, Semi, Autonomous }

public enum RoboStateName { Rest, Planning, Drive, Error }

public class RoboTransition {
    public RoboStateName from;
    public RoboStateName to;
    public Predicate<RC> condition;
    public Action<RC> action;
}
public class RoboState {
    public RoboStateName name;
    public RoboTransition[] transitions;
    public Action<RC> execution;
}
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

    public RoboStateName state = RoboStateName.Drive;
    public Vector3[] waypoints = new[] { Vector3.zero };
    static Quaternion fourtyfive = Quaternion.Euler(0, -45, 0);
    static float WAYPOINT_DETECTION_DISTANCE = 5/12f;
    static bool CheckIfAtWaypoint(RC self) {
        if (self.waypoints.Length > 0) {
            return (self.RoboTransform.position - self.waypoints[0]).magnitude < WAYPOINT_DETECTION_DISTANCE;
        } else {
            return false;
        }
    }
    static void NextWaypoint(RC self) {
        if (self.waypoints.Length > 0) {
            for( var i=1; i<self.waypoints.Length; i++) {
                self.waypoints[i - 1] = self.waypoints[i];
            }
            Array.Resize(ref self.waypoints, self.waypoints.Length - 1);
        }
    }
    static bool NoWaypoints(RC self) {
        return self.waypoints.Length == 0;
    }
    static bool HaveWaypoints(RC self) {
        return self.waypoints.Length > 0;
    }
    static bool AlwaysTrue(RC self) {
        return true;
    }
    static void DoDrive(RC self) {
        if(self.waypoints.Length == 0) {
            return;
        }
        
        Debug.DrawLine(self.RoboTransform.position, self.RoboTransform.position + self.RoboTransform.forward * 2, Color.green);
        Debug.DrawLine(self.RoboTransform.position, self.waypoints[0], Color.blue);
        // TODO: Implement this
        if (self.sensors.Min(s => s.distance) < 1.5f) {
            DoAvoidance(self);
        } else {
            var delta = self.waypoints[0] - self.RoboTransform.position;
            delta.y = 0;
            var angle = Vector3.SignedAngle(self.RoboTransform.forward, delta, Vector3.up);
            var dist = delta.magnitude;
            var deltaLocalNormalized = self.RoboTransform.InverseTransformVector(delta).normalized;
            //var deltaLocal = self.GetRotation()
            if (dist < 1) {
                self.CmdForward = deltaLocalNormalized.z;
                self.CmdStrafe = deltaLocalNormalized.x;
            } else {
                self.CmdRotate = angle / 180;
                self.CmdForward = 0.2f;
                if (Mathf.Abs(angle) < 10) {
                    self.CmdForward = 0.4f;
                }
            }

        }
    }
    static void DoNothing(RC self) {
        self.CmdForward = 0;
        self.CmdRotate = 0;
        self.CmdStrafe = 0;
    }
    static void GenerateWaypoints(RC self) {
        self.waypoints = new Vector3[1];
        self.waypoints[0].x = -5 + UnityEngine.Random.Range(0, 5) * 2;
        self.waypoints[0].z = -5 + UnityEngine.Random.Range(0, 5) * 2;
    }
    static bool ApproxEqual(float a, float b, float delta = 0.1f) {
        return Mathf.Abs(a - b) <= delta;
    }
    static void DoAvoidance(RC self) {
        float Freakiness = 0.3f;
        self.CmdForward = 0;
        self.CmdRotate = 0;
        self.CmdStrafe = 0;
        if (self.sensors.Length > 0) {
            self.CmdForward = 0.3f;
            bool collisionDetected = false;
            foreach (var sensor in self.sensors) {
                collisionDetected = collisionDetected || sensor.distance < 1.5f;

            }
            if (collisionDetected) {
                self.CmdForward = 0;
                // if (sensors.Length < 2.5f) {
                //      Flicker.useSpring = true;
                //  }
                //RaycastHit hit;
                //if (Physics.SphereCast(self.GetPosition(), .4f, self.transform.forward, out hit, 100)) {
                //    GameObject hitObject = hit.collider.gameObject;
                //    string hitObjectName = hitObject.name;

                //    if (hitObjectName == "Ball" || hitObjectName == "Robo") {
                //        self.CmdForward = self.maxMotorTorque * 1f * self.FORWARD_SPEED;
                //        if (self.sensors.Length < 4f) {
                //            self.Flicker.useSpring = true;
                //        }
                //        self.sensors[0].distance = 0;
                //        self.sensors[1].distance = 0;
                //        self.sensors[2].distance = 0;

                //    } else 
                if(self.sensors[1].distance <= 1f) {
                    self.CmdForward = -0.2f;
                }
                if (ApproxEqual(
                        self.sensors[0].distance - self.sensors[1].distance,
                        self.sensors[1].distance - self.sensors[2].distance)) {
                    // Probably facing wall
                    if (self.sensors[0].distance > self.sensors[2].distance) {
                        self.CmdRotate = -0.5f * Freakiness;
                    } else if (self.sensors[0].distance < self.sensors[2].distance) {
                        self.CmdRotate = 0.5f * Freakiness;
                    }
                } else if (ApproxEqual(self.sensors[0].distance , self.sensors[2].distance)){
                    self.CmdRotate = 2f * Freakiness;
                } else if (self.sensors[0].distance > self.sensors[2].distance) {
                    self.CmdRotate = -1f * Freakiness;
                } else if (self.sensors[0].distance < self.sensors[2].distance) {
                    self.CmdRotate = 1f * Freakiness;
                }
                //}
            }
        }
    }
    public Dictionary<RoboStateName, RoboState> states = new Dictionary<RoboStateName, RoboState>(){
        {
            RoboStateName.Drive,
            new RoboState {
                name = RoboStateName.Drive,
                transitions = new RoboTransition[] {
                    new RoboTransition {
                        from = RoboStateName.Drive,
                        to = RoboStateName.Drive,
                        condition = CheckIfAtWaypoint,
                        action = NextWaypoint
                    },
                    new RoboTransition {
                        from = RoboStateName.Drive,
                        to = RoboStateName.Rest,
                        condition = NoWaypoints,
                    }
                },
                execution = DoDrive
            }
        },
        {
            RoboStateName.Planning,
            new RoboState {
                name = RoboStateName.Planning,
                execution = GenerateWaypoints,
                transitions = new RoboTransition[] {
                    new RoboTransition{
                        from = RoboStateName.Planning,
                        to = RoboStateName.Drive,
                        condition = HaveWaypoints
                    }
                }
            }
        },
        {
            RoboStateName.Rest,
            new RoboState {
                name = RoboStateName.Rest,
                execution = DoNothing,
                transitions = new RoboTransition[] {
                    new RoboTransition {
                        from = RoboStateName.Rest,
                        to = RoboStateName.Planning,
                        condition = AlwaysTrue
                    }
                }
            }
        }
    };
    public Transform RoboTransform;
    //public Vector3 GetPosition() {
    //    if (RoboTransform) {
    //        return RoboTransform.position;
    //    } else {
    //        return Vector3.zero;
    //    }
    //}
    //public Quaternion GetRotation() {

    //    if (RoboTransform) {
    //        return fourtyfive * RoboTransform.rotation;
    //    } else {
    //        return Quaternion.identity;
    //    }
    //}
    //public Vector3 forward;
    //public Vector3 Forward() {
    //    if (RoboTransform) {
    //        RoboTransform.forward;
    //        return forward;
    //    } else {
    //        return Vector3.forward;
    //    }
    //}
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

    public float CmdForward = 0;
    public float CmdRotate = 0;
    public float CmdStrafe = 0;
    public void FixedUpdate() {
        if (autonomous != OperationalModes.Autonomous) {
            CmdForward = Input.GetAxis(ForwardAxis);
            CmdRotate = Input.GetAxis(RotateAxis);
            CmdStrafe = Input.GetAxis(StrafeAxis);
        }
        if (autonomous == OperationalModes.Autonomous) {
            // Execute State Machine
            var stateNode = states[state];
            if (stateNode.transitions != null) {
                foreach (var transition in stateNode.transitions) {
                    if (transition.condition(this)) {
                        transition.action?.Invoke(this);
                        state = transition.to;
                        Debug.Log("Transitioning from: " + transition.from + " to: " + transition.to);
                        break;
                    }
                }
            }
            stateNode.execution?.Invoke(this);
            // TODO: Sample Sensors, record obstacles
        }

        var ForwardTorque = CmdForward * maxMotorTorque * FORWARD_SPEED;
        var RotateTorque = CmdRotate * -maxMotorTorque * ROTATE_SPEED;
        var StrafeTorque = CmdStrafe * maxMotorTorque * STRAFE_SPEED;
        int axle_index = 0;
        var strafeScale = 1;
        foreach (AxleInfo axleInfo in axleInfos) {
            //if (axleInfo.steering) {
            //    axleInfo.leftWheel.steerAngle = steering;
            //    axleInfo.rightWheel.steerAngle = steering;
            //}
            if (axleInfo.motor) {
                axleInfo.leftWheel.motorTorque = Mathf.Clamp(ForwardTorque + RotateTorque + StrafeTorque * strafeScale, -maxMotorTorque, maxMotorTorque);
                axleInfo.rightWheel.motorTorque = Mathf.Clamp(ForwardTorque - RotateTorque + StrafeTorque * strafeScale, -maxMotorTorque, maxMotorTorque);
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