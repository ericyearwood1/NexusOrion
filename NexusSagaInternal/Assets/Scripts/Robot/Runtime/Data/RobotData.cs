using Robot.Runtime.Data.Robot;
using Robot.Runtime.View;
using UnityEngine;

namespace Robot.Runtime.Data
{
    public class RobotData
    {
        public RobotDisplay Display { get; set; }
        public Transform HomeContainer { get; set; }
        public bool IsTrackRobot { get; set; }
        public GripperState GripperState { get; set; }

        public Vector3 Velocity;
        public Vector3 Rotation;
        public Vector3 Position;
        public Vector3 EEPosition;
        public Quaternion EEOrientation;
        public YawPitchRoll EEYPR;
        
    }
}