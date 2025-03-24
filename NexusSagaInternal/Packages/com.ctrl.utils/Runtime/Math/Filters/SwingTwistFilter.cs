using UnityEngine;
using System;

namespace CTRL.Math.Filters
{
  [Serializable]
  public struct SwingTwist
  {
    public Quaternion Swing;
    public Quaternion Twist;
    public float Roll;
  }

  [Serializable]
  public class SwingTwistFilter : IFilter<Quaternion, SwingTwist>
  {
    // Parameters
    [SerializeField]
    public Vector3 Forward = Vector3.forward;

    // The current neutral position
    private Quaternion? _reference;
    public Quaternion? Reference { get => _reference; }

    public SwingTwistFilter() { }

    public void Reset()
    {
      _reference = null;
    }

    public void Reset(Quaternion newReference)
    {
      _reference = newReference;
    }

    public SwingTwist Evaluate(Quaternion orientation)
    {
      // Ensure we have a value
      if (!_reference.HasValue)
      {
        _reference = orientation;
      }

      // Move the current orientation into the reference orientation
      Quaternion invReference = Quaternion.Inverse(_reference.Value);
      Quaternion orientationInReference = invReference * orientation;

      // Decompose swing and twist
      var result = new SwingTwist();
      DecomposeSwingTwist(orientationInReference, Forward, out result.Swing, out result.Twist);

      // Extract roll
      result.Twist.ToAngleAxis(out result.Roll, out Vector3 axis);

      // Sign of axis gives sign of angle
      result.Roll *= Vector3.Dot(Forward, axis) > 0 ? 1.0f : -1.0f;

      return result;
    }

    // http://allenchou.net/2018/05/game-math-swing-twist-interpolation-sterp/
    public static void DecomposeSwingTwist(
      Quaternion q,
      Vector3 twistAxis,
      out Quaternion swing,
      out Quaternion twist
    )
    {
      Vector3 r = new Vector3(q.x, q.y, q.z);
      Vector3 p = Vector3.Project(r, twistAxis);

      // A singularity happens if
      // a) 180 degree rotation: q.w = 0, and
      // b) no twist component, pure swing only: Proj_t(q.axis) = 0
      // Source: https://arxiv.org/pdf/1506.05481v1.pdf page 23 (penultimate eqn.)

      // The singularity results in a zero quaternion.
      // But we don't need to do anything about this issue, since
      // Quaternion.normalized sets small quaternions to Quaternion.identity, which
      // is the correct value for twist in this case.
      
      twist = new Quaternion(p.x, p.y, p.z, q.w).normalized;
    
      swing = q * Quaternion.Inverse(twist);
    }
  }
}
