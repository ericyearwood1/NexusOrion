using UnityEngine;

using CTRL.Data;
using Quaternion = UnityEngine.Quaternion;

namespace CTRL.Samples
{
  [RequireComponent(typeof(IMUStream))]
  public class ImuVisualizer : ITK.DependBehavior
  {
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected IMUStream imu;

    [SerializeField]
    protected Transform orientationAxes;
    [SerializeField]
    protected TextMesh orientationText;

    [SerializeField]
    protected Transform accelerationArrow;
    [SerializeField]
    protected TextMesh accelerationText;

    [SerializeField]
    protected Transform angularRateArrow;
    [SerializeField]
    protected TextMesh angularRateText;

    private UnityEngine.Quaternion? startOrientation = null;

    protected void Update()
    {
      if (!imu.Latest.HasValue)
      {
        return;
      }

      var raw_imu = imu.Latest.Value.data;

      var imu_orientation = (UnityEngine.Quaternion)raw_imu.orientation;

      // The IMU's data is in real-world space, but not everyone is gonna be facing the
      // same cardinal direction when playing your game. To solve this, you can offset the
      // IMU data you get from the device by the direction the player is facing when the
      // game starts as shown below.
      if (!this.startOrientation.HasValue)
      {
        this.startOrientation = UnityEngine.Quaternion.Euler(0.0f, -imu_orientation.eulerAngles.y, 0.0f);
      }

      var orientation = this.startOrientation.Value * imu_orientation;
      var acceleration = this.startOrientation.Value * raw_imu.acceleration;
      var angularRate = this.startOrientation.Value * raw_imu.angular_rate;

      // Now we're going to visualize the data we obtained above with some arrows, as well
      // as print the numeric values on screen.
      this.orientationAxes.transform.localRotation = Quaternion.LookRotation(orientation * Vector3.forward, orientation * Vector3.right);
      var eulerAngles = orientation.eulerAngles;
      this.orientationText.text = string.Format(
          "x: {0:+00.00; -00.00}\ny: {1:+00.00; -00.00}\nz: {2:+00.00; -00.00}",
          eulerAngles.x, eulerAngles.y, eulerAngles.z);

      float accelScale = acceleration.magnitude / 9.8f;
      this.accelerationArrow.localScale = new Vector3(accelScale, accelScale, accelScale);
      if (acceleration != Vector3.zero)
      {
        this.accelerationArrow.transform.localRotation =
            UnityEngine.Quaternion.LookRotation(acceleration);
      }
      this.accelerationText.text = string.Format(
          "x: {0:+00.00; -00.00}\ny:{1:+00.00; -00.00}\nz:{2:+00.00; -00.00}",
          acceleration.x, acceleration.y, acceleration.z);

      float angularScale = angularRate.magnitude / 22.5f;
      this.angularRateArrow.localScale = new Vector3(angularScale, angularScale, angularScale);
      if (angularRate != Vector3.zero)
      {
        this.angularRateArrow.transform.localRotation =
            UnityEngine.Quaternion.LookRotation(angularRate);
      }
      this.angularRateText.text = string.Format(
          "x: {0:+00.00; -00.00}\ny:{1:+00.00; -00.00}\nz:{2:+00.00; -00.00}",
          angularRate.x, angularRate.y,
          angularRate.z);
    }
  }
}
