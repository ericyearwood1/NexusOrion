using UnityEngine;

namespace CTRL.Schemes.Samples
{
  public class DeltaToPos : ITK.DependBehavior
  {
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    [SerializeField]
    protected RelativeIMU pointer;

    [SerializeField]
    protected Vector2 scale;

    // Update is called once per frame
    protected void Update()
    {
      var delta = pointer.ReferenceDelta;
      
      transform.SetLocalPositionAndRotation(
        new Vector3(delta.x * scale.x, delta.y * scale.y, transform.localPosition.z),
        Quaternion.AngleAxis(pointer.TwistAngle * Mathf.Rad2Deg, new Vector3(0, 0, 1)));
    }
  }
}
