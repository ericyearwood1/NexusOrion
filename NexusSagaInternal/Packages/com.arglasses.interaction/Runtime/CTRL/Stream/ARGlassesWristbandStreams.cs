using System;
using CTRL;
using CTRL.Data;
using UnityEngine;
using CTRL.Flow;
using CTRL.Math;
using CTRL.Math.Filters;
using Quaternion = UnityEngine.Quaternion;
using ProjectionType = CTRL.Math.Filters.IDirectionToPlaneFilter.ProjectionType;

namespace ARGlasses.Interaction
{
    public static class CTRLConversions
    {
        public static Quaternion ToUnityAxisOrientation(Quaternion deviceOrientation) =>
            Quaternion.LookRotation(deviceOrientation * Vector3.forward, deviceOrientation * Vector3.right);

        public static Vector3 ParseNostrandRawAccel(Vector3 deviceAcceleration) => new(deviceAcceleration.y, -deviceAcceleration.z, -deviceAcceleration.x);
        public static Vector3 ParsePOCRawAccel(Vector3 deviceAcceleration) => new(-deviceAcceleration.x, -deviceAcceleration.z, -deviceAcceleration.y);
    }

    public class ARGlassesWristbandStreams : MonoBehaviour
    {
        public enum RawStreamFrequency
        {
            Full,
            Half,
        }

        public const float DefaultImuSensitivity = 1f;
        [SerializeField, ReadOnly] private CTRLClient _ctrl;

        [SerializeField, ReadOnly] private OutputStreamHandle<IMU> _imuStream;
        [SerializeField, ReadOnly] private OutputStreamHandle<Vector3> _accelerationStream;

        [SerializeField, ReadOnly] private Quaternion _absoluteOrientation;
        public Quaternion AbsoluteOrientation => _absoluteOrientation;

        [SerializeField, ReadOnly] private Quaternion _referenceOrientationBaseline = Quaternion.identity;
        public Quaternion UserOrientation => Quaternion.Euler(0, -_referenceOrientationBaseline.eulerAngles.y, 0) * AbsoluteOrientation;

        [SerializeField, ReadOnly] private Vector3 _rawAcceleration;
        public Vector3 RawAcceleration => _rawAcceleration;

        [SerializeField, ReadOnly] private Vector3 _pitchedAcceleration;
        public Vector3 PitchedAcceleration => _pitchedAcceleration;

        public Vector3 LocalGravityDir => Quaternion.Inverse(AbsoluteOrientation) * Vector3.down;

        [SerializeField] private ProjectionType _projectionType = ProjectionType.Stereographic;

        /// Filters
        private IDirectionToPlaneFilter _projectionFilter;

        private SwingTwistFilter _swingTwistFilter;
        private Pipeline<Sample<IMU>, Vector3> _imuForward;
        public Vector3 ImuForward => _imuForward.Tail.Latest;

        /// Pipeline to extract IMU spherical offset
        private Pipeline<Vector3, Vector2> _projectedRadiansPipeline;

        public Vector3? ReferenceDirection => _projectionFilter.ReferenceDirection;

        private bool IsSimulatingMouseKeyboard => _ctrl.State == ConnectionState.Disconnected;
        private bool _referenceInitialized;

        public Vector2 ProjectedRadians
        {
            get
            {
                if (IsSimulatingMouseKeyboard) return _fakeRelativeDelta;
                if (!_referenceInitialized) return Vector2.zero;
                return _projectedRadiansPipeline.Tail.Latest;
            }
        }

        public double Timestamp
        {
            get
            {
                double t = -1;
                if (_imuStream && _imuStream.Latest.HasValue) t = t > _imuStream.Latest.Value.timestamp_s ? t : _imuStream.Latest.Value.timestamp_s;
                return t;
            }
        }

        /// Pipeline to extract roll (radians)
        protected Pipeline<Sample<IMU>, float> _twistFilterRadians;

        public float TwistRadians => _twistFilterRadians.Tail.Latest;

        private double _prevImuTimestamp = 0;
        private double _prevAccelerationTimestamp = 0;

        private bool IsPocr => !Application.isEditor;
        private bool IsBinaryStream => IsPocr;

        private RawStreamFrequency _imuFrequency = RawStreamFrequency.Full;
        private RawStreamFrequency _accelFrequency = RawStreamFrequency.Full;
        private Type PocrRawImuComponent => ImuFrequency == RawStreamFrequency.Half ? typeof(RawImuStream_POCR_60) : typeof(RawImuStream_POCR_120);
        private Type PocrRawAccelerationComponent => AccelFrequency == RawStreamFrequency.Half ? typeof(RawAccelerationStream_POCR_60) : typeof(RawAccelerationStream_POCR_120);

        public RawStreamFrequency ImuFrequency
        {
            get => _imuFrequency;
            set
            {
                var wasEnabled = enabled;
                enabled = false;
                _imuFrequency = value;
                enabled = wasEnabled;
            }
        }

        public RawStreamFrequency AccelFrequency
        {
            get => _accelFrequency;
            set
            {
                var wasEnabled = enabled;
                enabled = false;
                _accelFrequency = value;
                enabled = wasEnabled;
            }
        }

        protected void Awake()
        {
            this.Scene(ref _ctrl);
            // this.Ensure(ref _imuStream, defaultType: UsingPocr ? typeof(RawImuStream_60) : typeof(RawImuStream_120));

            if (IsBinaryStream)
            {
                Debug.Log("Switching CTRL Port to 9998 for POC-R Binary Stream...");
                _ctrl.Port = 9998;
            }

            _swingTwistFilter = new SwingTwistFilter() { Forward = Vector3.forward };
            _twistFilterRadians = new Pipe<Sample<IMU>>().Chain(s => s.data.orientation.ToUnityQuaternion()).Chain(_swingTwistFilter.Evaluate).Chain(data => data.Roll)
                .Chain(data => data * Mathf.Deg2Rad);

            switch (_projectionType)
            {
                case ProjectionType.AzimuthalEquidistant:
                    _projectionFilter = new DirectionToPlaneFilterAzimuthalEquidistant();
                    break;
                case ProjectionType.SphericalCoordinates:
                    _projectionFilter = new DirectionToPlaneFilterSpherical();
                    break;
                case ProjectionType.Stereographic:
                    _projectionFilter = new DirectionToPlaneFilterStereographic();
                    break;
                case ProjectionType.Raycast:
                    _projectionFilter = new DirectionToPlaneFilterRaycast();
                    break;
                default:
                    Debug.LogWarning($"Unknown projection type {_projectionType}. Using Azimuthal Equidistant.");
                    _projectionFilter = new DirectionToPlaneFilterAzimuthalEquidistant();
                    break;
            }

            _imuForward = new Pipe<Sample<IMU>>().Chain(s => s.data.orientation.ToUnityQuaternion()).Chain(q => q * Vector3.forward);
            _projectedRadiansPipeline = _imuForward.Tail.Chain(_projectionFilter.Evaluate);
        }

        protected virtual void OnEnable()
        {
            ResetReference();

            if (IsPocr)
            {
                _imuStream = gameObject.AddComponent(PocrRawImuComponent) as OutputStreamHandle<IMU>;
                _accelerationStream = gameObject.AddComponent(PocrRawAccelerationComponent) as OutputStreamHandle<Vector3>;
            }
            else
            {
                _imuStream = gameObject.AddComponent<RawImuStream_Nostrand>();
                _accelerationStream = null;
            }

            Debug.Log($"IMU Stream: {_imuStream}");
            Debug.Log($"Acceleration Stream: {_accelerationStream}");

            _imuStream.enabled = true;
            _imuStream.OnStreamLatestSample.AddListener(_imuForward);
            _imuStream.OnStreamLatestSample.AddListener(_twistFilterRadians);
            _imuStream.OnStreamSample.AddListener(ImuSampleHandler);
            if (_accelerationStream) _accelerationStream.OnStreamSample.AddListener(PocrAccelerationSampleHandler);
        }

        protected virtual void OnDisable()
        {
            _imuStream.enabled = false;
            _imuStream.OnStreamLatestSample.RemoveListener(_imuForward);
            _imuStream.OnStreamLatestSample.RemoveListener(_twistFilterRadians);
            _imuStream.OnStreamSample.RemoveListener(ImuSampleHandler);
            if (_accelerationStream) _accelerationStream.OnStreamSample.RemoveListener(PocrAccelerationSampleHandler);

            Destroy(_imuStream);
            if (_accelerationStream) Destroy(_accelerationStream);
        }

        public event Action PreResetReference = delegate { };
        public event Action PostResetReference = delegate { };

        public void ResetReference()
        {
            PreResetReference();

            _lastMousePosition = Input.mousePosition;
            _fakeRelativeDelta = default;
            _referenceInitialized = false;

            _projectionFilter?.Reset();
            _swingTwistFilter?.Reset();

            _referenceOrientationBaseline = AbsoluteOrientation;

            PostResetReference();
        }

        [SerializeField] private float _fakeImuScale = 0.001f;
        [SerializeField, ReadOnly] private Vector3 _lastMousePosition;
        [SerializeField, ReadOnly] private Vector3 _fakeRelativeDelta;

        private void Update()
        {
            var mouseDelta = new Vector3(Input.GetAxis("Mouse X"), Input.GetAxis("Mouse Y"));
            _fakeRelativeDelta += mouseDelta * _fakeImuScale;
            _lastMousePosition = Input.mousePosition;
        }

        public struct ContinuousSample<T> where T : struct
        {
            public float DeltaTime;
            public T Data;
        }

        private DeltaTimeBuffer _imuSampleTimeBuffer = new DeltaTimeBuffer(1);
        public float ImuSamplesPerSecond => _imuSampleTimeBuffer.Count;
        protected virtual void ImuSampleHandler(Sample<IMU> sample)
        {
            if (_prevImuTimestamp == 0) _prevImuTimestamp = sample.timestamp_s;
            var deltaTime = (float)(sample.timestamp_s - _prevImuTimestamp);
            if (deltaTime <= 0) return;
            _prevImuTimestamp = sample.timestamp_s;
            _imuSampleTimeBuffer.AddEvent(0, deltaTime);

            var orientation = sample.data.orientation;
            EmitOrientation(orientation, deltaTime);

            if (!_accelerationStream)
            {
                var acceleration = sample.data.acceleration;
                var rawAcceleration = IsPocr ? CTRLConversions.ParsePOCRawAccel(acceleration) : CTRLConversions.ParseNostrandRawAccel(acceleration);
                EmitAcceleration(rawAcceleration, deltaTime);
            }
        }

        public event Action<ContinuousSample<float>> WhenImuTwistRadiansSample = delegate { };
        public event Action<ContinuousSample<Quaternion>> WhenImuOrientationSample = delegate { };

        private void EmitOrientation(CTRL.Data.Quaternion orientation, float deltaTime)
        {
            _referenceInitialized = true;
            WhenImuTwistRadiansSample(new ContinuousSample<float>
            {
                DeltaTime = deltaTime,
                Data = TwistRadians
            });

            _absoluteOrientation = CTRLConversions.ToUnityAxisOrientation((Quaternion)orientation);

            WhenImuOrientationSample(new ContinuousSample<Quaternion>
                {
                    DeltaTime = deltaTime,
                    Data = _absoluteOrientation
                }
            );
        }

        private bool _accountForGravity = true;
        public event Action<ContinuousSample<Vector3>> WhenImuAccelerationSample = delegate { };

        private void EmitAcceleration(Vector3 rawAcceleration, float deltaTime)
        {
            _rawAcceleration = rawAcceleration;
            var deviceAcceleration = _accountForGravity ? rawAcceleration - LocalGravityDir * 9.8f : rawAcceleration;
            _pitchedAcceleration = Quaternion.Euler(-AbsoluteOrientation.Pitch(), 0, 0) * deviceAcceleration;

            WhenImuAccelerationSample(new ContinuousSample<Vector3>
            {
                DeltaTime = deltaTime,
                Data = _rawAcceleration
            });
        }

        private DeltaTimeBuffer _accelSampleTimeBuffer = new DeltaTimeBuffer(1);
        public float AccelSamplesPerSecond => _accelSampleTimeBuffer.Count;
        private void PocrAccelerationSampleHandler(Sample<Vector3> sample)
        {
            if (_prevAccelerationTimestamp == 0) _prevAccelerationTimestamp = sample.timestamp_s;
            float deltaTime = (float)(sample.timestamp_s - _prevAccelerationTimestamp);
            if (deltaTime <= 0) return;
            _prevAccelerationTimestamp = sample.timestamp_s;
            _accelSampleTimeBuffer.AddEvent(0, deltaTime);

            var rawAcceleration = CTRLConversions.ParsePOCRawAccel(sample.data);
            EmitAcceleration(rawAcceleration, deltaTime);
        }
    }
}
