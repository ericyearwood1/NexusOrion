namespace CTRL.Data
{
  /// <summary>
  /// A Quaternion in **CTRL-R's axis ordering**.
  /// Note that this is different from Unity's axis ordering - CTRL-R
  /// defines the X axis as forward, whereas Unity defines the Z axis as forward.
  /// <br /> To use this quaternion within Unity to rotate GameObjects,
  /// you should convert it to a Unity quaternion.
  /// </summary>
  [System.Serializable]
  public struct Quaternion
  {
#pragma warning disable IDE1006
    public double w;
    public double x;
    public double y;
    public double z;
#pragma warning restore IDE1006

    /// Convert to a Unity quaternion via overloaded cast.
    /// `var unity_quat = (UnityEngine.Quaternion)ctrl_qaut;`
    public static explicit operator UnityEngine.Quaternion(Quaternion ctrlq)
    {
      var thumb_to_top_wrist = ctrlq.x;
      var top_wrist_to_fingers = ctrlq.y;
      var fingers_to_thumb = ctrlq.z;

      // Map these to rotations from one axis to another in Unity's coordinate system
      var yz = -fingers_to_thumb;
      var zx = -top_wrist_to_fingers;
      var xy = -thumb_to_top_wrist;

      return new UnityEngine.Quaternion(
          (float)yz,
          (float)zx,
          (float)xy,
          (float)ctrlq.w);
    }

    /// Convert to a Unity quaternion.
    public UnityEngine.Quaternion ToUnityQuaternion() => (UnityEngine.Quaternion)this;

    /// Construct a CTRL-R quaternion from a Unity quaternion.
    /// Assumes the Unity quaternion is in Unity's default axis ordering (Z forward).
    public static Quaternion FromUnityQuaternion(UnityEngine.Quaternion quaternion)
    {
      return new Quaternion
      {
        x = -quaternion.z,
        y = -quaternion.y,
        z = -quaternion.x,
        w = quaternion.w,
      };
    }

    /// Return the identity quaternion (in CTRL-R's frame)
    public static Quaternion Identity() => new Quaternion { x = 0.0, y = 0.0, z = 0.0, w = 1.0 };
  }


  /// The current orientation and acceleration applied to the CTRL-kit Armband.
#pragma warning disable IDE1006
  [System.Serializable]
  public struct IMU
  {
    /// Note: this is the absolute orientation of the CTRL-kit armband in space.
    public CTRL.Data.Quaternion orientation;
    /// Has units of m/s^2
    public UnityEngine.Vector3 acceleration;
    /// Has units of rad/s^2
    public UnityEngine.Vector3 angular_rate;
    public bool calibrated;
  }
#pragma warning restore IDE1006

  /// A continous stream of the CTRL-kit Armband's rotation and acceleration.
  /// Generally the output of this stream isn't reliable until the IMU is
  /// calibrated,
  /// which happens automatically after a short period of wearing the CTRL-kit Armband
  /// in a variety of orientations.
  /// <br />
  public class IMUStream : OutputStreamHandle<IMU>
  {
    protected override string defaultStreamName => "raw_imu";
  }

}
