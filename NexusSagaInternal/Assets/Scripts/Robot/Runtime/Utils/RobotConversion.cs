using Robot.Runtime.Data.Robot;
using UnityEngine;

namespace Robot.Runtime.Utils
{
    public static class RobotConversion
    {
        public static Quaternion RobotPoseToQuaternion(float[] pose)
        {
            return new Quaternion(pose[1], pose[2] * -1, pose[0] * -1, pose[3]);
        }

        public static Vector3 RobotPoseToVector3(float[] robotPose)
        {
            return new Vector3(
                robotPose[1] * -1,
                robotPose[2],
                robotPose[0]);
        }
        
        public static Vector3 RightHandToLeftHand(Vector3 robotPose)
        {
            return new Vector3(
                robotPose.y * -1,
                robotPose.z,
                robotPose.x);
        }
        
        public static YawPitchRoll RightHandToLeftHand(YawPitchRoll robotPose)
        {
            return new YawPitchRoll {Pitch = robotPose.Pitch,
                    Roll = robotPose.Roll * -1,
                    Yaw = robotPose.Yaw * -1}
                ;
        }
        
        public static Vector3 UnityToRobot(Vector3 v)
        {
            return new Vector3( v.z, v.x * -1, v.y );
        }
        
        public static Vector3 RobotToUnity(Vector3 v)
        {
            return new Vector3( v.y * -1, v.z, v.x );
        }

        public static float[] Vector2ToRobotPose(Vector2 v)
        {
            return new[] { v.y, v.x * -1, 0 };
        }
    }
}