using UnityEngine;

namespace SiroComms.Runtime.Services.Utils
{
    public static class RobotConversion
    {
        public static float[] QuaternionToRobotPose(Quaternion q)
        {
            return new [] { q.z, q.x * -1, q.y, q.w };
        }

        public static float[] Vector3ToRobotPose(Vector3 v)
        {
            return new[] { v.z, v.x * -1, v.y };
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

        public static Vector3 EulerRightToLeft(Vector3 v)
        {
            return new Vector3( v.y * -1, v.z, v.x );

        }


    }
}