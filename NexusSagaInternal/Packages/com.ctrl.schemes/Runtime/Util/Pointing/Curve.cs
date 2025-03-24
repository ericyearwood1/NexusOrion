using UnityEngine;

namespace CTRL.Utils.Pointing
{
  /// Renders a curve through the given points.
  [RequireComponent(typeof(LineRenderer))]
  public class Curve : MonoBehaviour
  {
    [SerializeField]
    protected int vertices = 16;

    protected LineRenderer lineRenderer;

    protected virtual void Start()
    {
      this.lineRenderer = this.GetComponent<LineRenderer>();
    }

    protected static Vector3 Bezier(Vector3 start, Vector3 handle, Vector3 end, float time)
    {
      return Vector3.Lerp(
          Vector3.Lerp(start, handle, time),
          Vector3.Lerp(handle, end, time),
          time);
    }

    /// Sets the start end and handle of the curve.
    public void SetPositions(Vector3 start, Vector3 handle, Vector3 end)
    {
      this.lineRenderer.positionCount = this.vertices;
      for (var i = 0; i < this.vertices; ++i)
      {
        var time = (float)i / (float)(this.vertices - 1);
        this.lineRenderer.SetPosition(i, Curve.Bezier(start, handle, end, time));
      }
    }
  }
}
