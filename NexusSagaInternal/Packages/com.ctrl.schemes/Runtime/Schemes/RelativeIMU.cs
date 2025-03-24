using UnityEngine;

using CTRL.Data;
using CTRL.Flow;
using CTRL.Math.Filters;
using ProjectionType = CTRL.Math.Filters.IDirectionToPlaneFilter.ProjectionType;

namespace CTRL.Schemes
{
  public class RelativeIMU : ITK.DependBehavior
  {
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected IMUStream imuStream;

    /// "Forward" direction down the forearm
    [SerializeField]
    protected Vector3 Forward = Vector3.forward;

    [SerializeField]
    protected ProjectionType Projection = ProjectionType.AzimuthalEquidistant;
    private ProjectionType _previousProjection;

    /// Filters
    protected IDirectionToPlaneFilter orientationFilter;
    protected SwingTwistFilter swingTwistFilter;

    /// Pipeline to extract IMU relative orientation
    protected Pipeline<Sample<IMU>, Vector3> imuToOrientation;
    public Pipe<Vector3> OnOrientation { get => imuToOrientation.Tail; }
    public Vector3 Orientation { get => OnOrientation.Latest; }

    /// Pipeline to extract IMU spherical offset
    protected Pipeline<Vector3, Vector2> orientationToDelta;
    public Vector3? ReferenceDirection { get => orientationFilter.ReferenceDirection; }
    public Pipe<Vector2> OnReferenceDelta { get => orientationToDelta.Tail; }
    public Vector2 ReferenceDelta { get => OnReferenceDelta.Latest; }

    /// Pipeline to extract roll (radians)
    protected Pipeline<Sample<IMU>, float> twistFilter;
    public Pipe<float> OnTwistAngle { get => twistFilter.Tail; }
    public float TwistAngle { get => twistFilter.Tail.Latest; }

    protected RelativeIMU()
    {
      _previousProjection = Projection;

      swingTwistFilter = new SwingTwistFilter() { Forward = this.Forward };

      twistFilter = new Pipe<Sample<IMU>>()
        .Chain((sample) => sample.data.orientation.ToUnityQuaternion())
        .Chain(swingTwistFilter.Evaluate)
        .Chain((data) => data.Roll)
        .Chain((data) => data * Mathf.Deg2Rad);
    }

    protected void InitializeOrientationFilter()
    {
      switch (Projection)
      {
        case ProjectionType.AzimuthalEquidistant:
          orientationFilter = new DirectionToPlaneFilterAzimuthalEquidistant();
          break;
        case ProjectionType.SphericalCoordinates:
          orientationFilter = new DirectionToPlaneFilterSpherical();
          break;
        case ProjectionType.Stereographic:
          orientationFilter = new DirectionToPlaneFilterStereographic();
          break;
        case ProjectionType.Raycast:
          orientationFilter = new DirectionToPlaneFilterRaycast();
          break;
        case ProjectionType.Euler:
          orientationFilter = new DirectionToPlaneFilterEuler();
          break;
        default:
          Debug.LogWarning($"Unknown projection type {Projection}. Using Azimuthal Equidistant.");
          orientationFilter = new DirectionToPlaneFilterAzimuthalEquidistant();
          break;
      }

      imuToOrientation = new Pipe<Sample<IMU>>()
        .Chain((sample) => sample.data.orientation.ToUnityQuaternion())
        .Chain((q) => q * Forward);

      orientationToDelta = imuToOrientation.Tail
        .Chain(orientationFilter.Evaluate);

      ResetReference();
      AddListeners();
    }

    protected virtual void OnEnable()
    {
      InitializeOrientationFilter();
    }

    protected virtual void OnDisable()
    {
      RemoveListeners();
    }

    protected virtual void AddListeners()
    {
      if (imuStream == null)
      {
        return;
      }
      RemoveListeners();
      imuStream.OnStreamLatestSample.AddListener(imuToOrientation);
      imuStream.OnStreamLatestSample.AddListener(twistFilter);
    }

    protected virtual void RemoveListeners()
    {
      if (imuStream == null)
      {
        return;
      }
      if (imuToOrientation != null)
      {
        imuStream.OnStreamLatestSample.RemoveListener(imuToOrientation);
      }
      imuStream.OnStreamLatestSample.RemoveListener(twistFilter);
    }

    public void ResetReference()
    {
      orientationFilter.Reset();
      swingTwistFilter.Reset();
    }

    protected void OnValidate()
    {
      if (Projection != _previousProjection)
      {
        InitializeOrientationFilter();
        _previousProjection = Projection;
      }
    }
  }
}
