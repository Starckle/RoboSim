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
    public List<Vector2Int> waypoints = new List<Vector2Int>();
    public Vector2Int? Target = null;
    static Quaternion fourtyfive = Quaternion.Euler(0, -45, 0);
    static float WAYPOINT_DETECTION_DISTANCE = 1f;
    static float WAY_COUNT_X = 0;
    static float WAY_COUNT_Y = 0;
    static float WAY_COUNT = 0;

    static int BLOCKED = -1;
    public static int CLEAR_OF_OBSTACLES = 100;
    static float DRIVE_NO_AVOIDANCE_TIME = 0.5f;
    public static int GRID_WIDTH = 13; //6;
    public static int GRID_HEIGHT = 13; //6;
    public static float GRID_SIZE = 1f; //2; //2ft per cube
    public static float GRID_X_START = -6f;
    public static float GRID_Y_START = -6f;
    static int MAX_DIST = GRID_WIDTH * GRID_HEIGHT;
    public float[,] obstacles = new float[GRID_WIDTH, GRID_HEIGHT];
    public float[,] pathplanningtemp = new float[GRID_WIDTH, GRID_HEIGHT];
    public bool debug_lines = false;
    public bool show_waypoints = false;
    List<Transform> arrows = new List<Transform>();
    Transform targetArrow = null;
    public Transform arrowPrefab = null;
    public Transform targetPrefab = null;
    static Vector2Int[] DIRS = new Vector2Int[] {
        Vector2Int.up, Vector2Int.right, Vector2Int.down, Vector2Int.left
    };
    static Vector2Int[] CORNER_DIRS = new Vector2Int[] {
        //Vector2Int.up+Vector2Int.left, Vector2Int.up+Vector2Int.right, Vector2Int.down+Vector2Int.left, Vector2Int.down+Vector2Int.right
    };
    //static Vector2Int[] CORNER_DIRS = new Vector2Int[] {
    //    Vector2Int.up, Vector2Int.right, Vector2Int.down, Vector2Int.left
    //};
    public static Vector2Int PositionToIndex(Vector3 pos) {
        return new Vector2Int((int)Mathf.Round((pos.x - GRID_X_START) / GRID_SIZE), (int)Mathf.Round((pos.z - GRID_Y_START) / GRID_SIZE));
    }
    public static Vector3 IndexToPosition(Vector2Int idx) {
        return new Vector3(idx.x * GRID_SIZE + GRID_X_START, 0, idx.y * GRID_SIZE + GRID_Y_START);
    }
    static bool InGrid(Vector2Int idx) {
        return idx.x >= 0 && idx.x < GRID_WIDTH && idx.y >= 0 && idx.y < GRID_HEIGHT;
    }
    static Vector3[] PRESET_OBSTACLES = new[] {
        new Vector3( -2, 0,  0),
        new Vector3(  2, 0,  0),
        new Vector3(  0, 0, -2),
        new Vector3(  0, 0,  2),

        new Vector3( -2, 0, -2),
        new Vector3(  2, 0, -2),
        new Vector3( -2, 0,  2),
        new Vector3(  2, 0,  2),

        new Vector3( -4, 0, -2),
        new Vector3( -4, 0,  2),
        new Vector3(  4, 0, -2),
        new Vector3(  4, 0,  2),

        new Vector3( -2, 0, -4),
        new Vector3( -2, 0,  4),
        new Vector3(  2, 0, -4),
        new Vector3(  2, 0,  4),
    };
    public ObstacleViewer obs_view = null;
    public void SetWaypoints(List<Vector2Int> wps) {
        waypoints = wps;
        //regenerate arrows
        if (show_waypoints) {
            foreach (var arrow in arrows) {
                Destroy(arrow.gameObject);
            }
            arrows.Clear();
            for (var i = 0; i < waypoints.Count - 1; i++) {
                var pos = IndexToPosition(waypoints[i]);
                var next = IndexToPosition(waypoints[i + 1]);
                var forward = next - pos;
                var arrow = Instantiate(arrowPrefab, pos, Quaternion.identity, transform);
                arrows.Add(arrow);
                foreach (var mr in arrow.GetComponentsInChildren<MeshRenderer>()) {
                    mr.material = RoboTransform.GetComponent<MeshRenderer>().material;
                }
                arrow.forward = forward;
            }
        }
    }
    public void PrefillKnownObstacles() {
        foreach(var obs in PRESET_OBSTACLES) {
            var idx = PositionToIndex(obs);
            obstacles[idx.x, idx.y] = 1000000;
        }
    }
    public List<Vector2Int> GenerateWaypointsToTarget(Vector3 from, Vector3 to) {
        var from_idx = PositionToIndex(from);
        var to_idx = PositionToIndex(to);
        List<Vector2Int> waypoints = new List<Vector2Int>();
        for(var x =0; x<GRID_WIDTH; x++) {
            for(var y = 0; y<GRID_HEIGHT; y++) {
                if (obstacles[x, y] > CLEAR_OF_OBSTACLES) {
                    pathplanningtemp[x, y] = BLOCKED;
                } else {
                    pathplanningtemp[x, y] = MAX_DIST;
                }
            }
        }
        HashSet<Tuple<Vector2Int,float>> locs = new HashSet<Tuple<Vector2Int, float>>();
        locs.Add(Tuple.Create(to_idx,0f));
        bool graph_completed = false;
        while (locs.Any() && !graph_completed) {
            HashSet<Tuple<Vector2Int, float>> new_locs = new HashSet<Tuple<Vector2Int, float>>();
            foreach (var (loc,offset) in locs) {
                if ( obstacles[loc.x, loc.y] > CLEAR_OF_OBSTACLES) {
                    continue;
                }
                if (loc == from_idx) {
                    graph_completed = true;
                }
                if (offset < pathplanningtemp[loc.x, loc.y] ) {
                    pathplanningtemp[loc.x, loc.y] = offset;
                    foreach(var off in DIRS) {
                        var new_pos = loc + off;
                        if (InGrid(new_pos) && (offset + 1f) < pathplanningtemp[new_pos.x, new_pos.y]) {
                            new_locs.Add(Tuple.Create(new_pos,offset+1.0f));
                        }
                    }
                    foreach (var off in CORNER_DIRS) {
                        var new_pos = loc + off;
                        if (InGrid(new_pos) && (offset + 1.414f) < pathplanningtemp[new_pos.x, new_pos.y]) {
                            new_locs.Add(Tuple.Create(new_pos, offset + 1.414f));
                        }
                    }
                }
            }
            locs = new_locs;
        }

        if (graph_completed) {
            var curr_loc = from_idx;
            var curr_dist = pathplanningtemp[curr_loc.x, curr_loc.y];
            // TODO: This can loop forever. add safety
            while (curr_loc != to_idx) {
                var found = false;
                Vector2Int least_next = new Vector2Int();
                float least_next_dist = float.MaxValue;
                foreach (var dir in DIRS.Concat(CORNER_DIRS).ToArray()) {
                    var next = curr_loc + dir;
                    var next_dist = InGrid(next)?pathplanningtemp[next.x, next.y]:-1;
                    if (InGrid(next) && next_dist < curr_dist && next_dist != BLOCKED) {
                        if(next_dist< least_next_dist) {
                            least_next = next;
                            least_next_dist = next_dist;
                            found = true;
                        }
                        //curr_loc = next;
                        //curr_dist = pathplanningtemp[next.x, next.y];
                        //continue;
                    }
                }
                if (found) {
                    curr_loc = least_next;
                    curr_dist = least_next_dist;
                    waypoints.Add(curr_loc);
                    //Debug.Assert(found, "Oh God, we are stuck.");
                } else {
                    return new List<Vector2Int>();
                }
            }
        }
        return waypoints;
    }
    static bool CheckIfAtWaypoint(RC self) {
        foreach (var wp in self.waypoints) {
            if((self.RoboTransform.position - IndexToPosition(wp)).magnitude < WAYPOINT_DETECTION_DISTANCE) {
                return true;
            }
        }
        return false;
    }
    static void NextWaypoint(RC self) {
        var end = 0;
        for(var i=self.waypoints.Count-1; i>=0; i--) {
            if ((self.RoboTransform.position - IndexToPosition(self.waypoints[0])).magnitude < WAYPOINT_DETECTION_DISTANCE) {
                end = i;
                continue;
            }
        }
        if (self.waypoints.Count > 0) {
            self.waypoints.RemoveRange(0,end+1);
            self.SetWaypoints(self.waypoints);
            //for( var i=1; i<self.waypoints.Count; i++) {
            //    self.waypoints[i - 1] = self.waypoints[i];
            //}
            //Array.Resize(ref self.waypoints, self.waypoints.Length - 1);
        }
    }
    static bool NoWaypoints(RC self) {
        return self.waypoints.Count == 0;
    }
    static bool HaveWaypoints(RC self) {
        return self.waypoints.Count > 0;
    }
    static bool AlwaysTrue(RC self) {
        return true;
    }
    static bool IsBlocked(RC self) {
        return self.waypoints.Count > 0 && self.obstacles[self.waypoints[0].x, self.waypoints[0].y] > CLEAR_OF_OBSTACLES;
    }
    static bool IsTooFar(RC self) {
        return self.waypoints.Count > 0 && (IndexToPosition(self.waypoints[0]) - self.RoboTransform.position).magnitude > 2;
    }
    static void ClearWaypoints(RC self) {
        self.SetWaypoints(new List<Vector2Int>());
    }
    static void DoDrive(RC self) {
        if(self.waypoints.Count == 0) {
            return;
        }
        if (self.debug_lines) {
            //Debug.DrawLine(self.RoboTransform.position, self.RoboTransform.position + self.RoboTransform.forward * 2, Color.blue);
            var from = self.RoboTransform.position;
            foreach(var wp in self.waypoints) {
                var to = IndexToPosition(wp);
                Debug.DrawLine(from, to, Color.green);
                from = to;
            }
            if (self.Target.HasValue) {
                Debug.DrawLine(self.RoboTransform.position, IndexToPosition(self.Target.Value), Color.magenta);
            }
        }

        if ((Time.fixedTime - self.stateStartTime)>=DRIVE_NO_AVOIDANCE_TIME && self.sensors.Min(s => s.distance) < 1.5f) {
            DoAvoidance(self);
        } else {
            var delta = IndexToPosition(self.waypoints[0]) - self.RoboTransform.position;
            delta.y = 0;
            var angle = Vector3.SignedAngle(self.RoboTransform.forward, delta, Vector3.up);
            var dist = delta.magnitude;
            var deltaLocalNormalized = self.RoboTransform.InverseTransformVector(delta).normalized;
            //var deltaLocal = self.GetRotation()
            if (dist < 1.5) {
                self.CmdForward = deltaLocalNormalized.z / 2;
                self.CmdStrafe = deltaLocalNormalized.x / 2;
                self.CmdRotate = angle / 180;
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
        GenerateMainwaypoint(self);
    }
    static void GenerateWaypoints(RC self) {
        if (self.Target.HasValue) {
            var start = Time.realtimeSinceStartup;
            self.SetWaypoints(self.GenerateWaypointsToTarget(self.RoboTransform.position, IndexToPosition(self.Target.Value)));
            if (self.waypoints.Count == 0) {
                GenerateMainwaypoint(self);
            }
            var stop = Time.realtimeSinceStartup;
            Debug.Log("Created a " + self.waypoints.Count + " waypoint route in " + (stop - start) + " ms");
        }
 /* !!!! ----> */    //if (ApproxEqual(self.waypoints[0].x, self.Mainwaypoint[0].x)) {
            //self.Mainwaypoint = new Vector3[1];
            //self.Mainwaypoint[0].x = -5 + UnityEngine.Random.Range(0, 5) * 2;
            //self.Mainwaypoint[0].z = -5 + UnityEngine.Random.Range(0, 5) * 2;
      //  }
        //self.waypoints = new Vector3[1];
        //self.waypoints[0].x = -5 + UnityEngine.Random.Range(0, 5) * 2;
        //self.waypoints[0].z = -5 + UnityEngine.Random.Range(0, 5) * 2;
 // CONCEPT  
        // if (WAY_COUNT % 2 == 0)
            // Mainwaypoint[0].x - WAY_COUNT_X (WAY_COUNT_X = Mainwaypoint[0].x - 1)
            //WAY_COUNT_X = WAY_COUNT_X - 1
        //else
            // Mainwaypoint[0].z - WAY_COUNT_Z (WAY_COUNT_Z = Mainwaypoint[0].Z - 1)
            //WAY_COUNT_Z = WAY_COUNT_Z - 1
        // WAY_COUNT = WAY_COUNT + 1

    }
    static void DoPlanning(RC self) {

        self.CmdForward = 0;
        self.CmdRotate = 0;
        self.CmdStrafe = 0;
        GenerateWaypoints(self);
        // GenerateMainwaypoint(self);
    }
    static void GenerateMainwaypoint(RC self) {
        self.Target = null;
        while (!self.Target.HasValue) {
            var newTarget = new Vector2Int(UnityEngine.Random.Range(0, GRID_WIDTH-1), UnityEngine.Random.Range(0, GRID_HEIGHT-1));
            if (self.obstacles[newTarget.x, newTarget.y]<CLEAR_OF_OBSTACLES) {
                self.Target = newTarget;
            }
        }
        if (self.show_waypoints) {
            if (self.targetArrow) {
                Destroy(self.targetArrow.gameObject);
            }
            self.targetArrow = Instantiate(self.targetPrefab, IndexToPosition(self.Target.Value), Quaternion.identity);
            foreach(var mr in self.targetArrow.GetComponentsInChildren<MeshRenderer>()) {
                mr.material = self.RoboTransform.GetComponent<MeshRenderer>().material;
            }
        }
    }
    static bool ApproxEqual(float a, float b, float delta = 0.1f) {
        return Mathf.Abs(a - b) <= delta;
    }
    static void DoAvoidance(RC self) {
        float Freakiness = 0.8f;
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
                    self.CmdForward = -1f;
                    Debug.DrawLine(self.RoboTransform.position, self.RoboTransform.position + self.RoboTransform.forward * 3, Color.cyan);
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
                        to = RoboStateName.Planning,
                        condition = (RC rc) => (Time.fixedTime - rc.stateStartTime)>5
                    },
                    new RoboTransition {
                        from = RoboStateName.Drive,
                        to = RoboStateName.Rest,
                        condition = NoWaypoints,
                    },
                    new RoboTransition {
                        from = RoboStateName.Drive,
                        to = RoboStateName.Planning,
                        condition = IsBlocked,
                        action = ClearWaypoints,
                    },
                    new RoboTransition {
                        from = RoboStateName.Drive,
                        to = RoboStateName.Planning,
                        condition = IsTooFar,
                        action = ClearWaypoints,
                    }
                },
                execution = DoDrive
            }
        },
        {
            RoboStateName.Planning, 
            new RoboState {
                name = RoboStateName.Planning,
                execution = DoPlanning,
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
    public void Awake() {
        PrefillKnownObstacles();
    }
    public void Update() {
        if (Input.GetKeyDown(FlickButton)) {
            Flicker.useSpring = true;
        }
        if (Input.GetKeyUp(FlickButton)) {
            Flicker.useSpring = false;
        }
        if (obs_view != null) {
            obs_view.setObstacles(obstacles);
        }
    }
    float FORWARD_SPEED = 1f;
    float ROTATE_SPEED = 1f;
    float STRAFE_SPEED = 1.0f;

    public float CmdForward = 0;
    public float CmdRotate = 0;
    public float CmdStrafe = 0;

    public float stateStartTime = 0;
    public void FixedUpdate() {
        if (autonomous != OperationalModes.Autonomous) {
            CmdForward = Input.GetAxis(ForwardAxis);
            CmdRotate = Input.GetAxis(RotateAxis);
            CmdStrafe = Input.GetAxis(StrafeAxis);
        }
        if (autonomous == OperationalModes.Autonomous) {
            // Populate Obstacles from sensors
            foreach (var sensor in sensors) {
                var loc = sensor.transform.position + sensor.transform.forward * sensor.distance;
                var iloc = PositionToIndex(loc);
                if (InGrid(iloc) && obstacles[iloc.x, iloc.y]<=1000) {
                    obstacles[iloc.x, iloc.y] = Mathf.Min(1000,obstacles[iloc.x, iloc.y] + 3);
                }
            }
            //If you find yourself in a self made obstacle, clear it
            var iloc_self = PositionToIndex(RoboTransform.position);
            if(obstacles[iloc_self.x, iloc_self.y] <= 1000) {
                obstacles[iloc_self.x, iloc_self.y] = 0;
            }
            // Degrade Obstacles
            for (var x = 0; x<GRID_WIDTH; x++) {
                for(var y=0;y<GRID_HEIGHT; y++) {
                    obstacles[x, y] = Mathf.Max(0, obstacles[x, y]- 0.2f);
                }
            }
            // Execute State Machine
            var stateNode = states[state];
            if (stateNode.transitions != null) {
                foreach (var transition in stateNode.transitions) {
                    if (transition.condition(this)) {
                        transition.action?.Invoke(this);
                        state = transition.to;
                        stateStartTime = Time.fixedTime;
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