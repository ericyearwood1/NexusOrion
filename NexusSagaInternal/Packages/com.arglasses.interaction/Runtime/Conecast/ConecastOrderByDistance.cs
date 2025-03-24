using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ARGlasses.Interaction
{
    /// <summary>
    /// Use the OrderedHits from GazeGrabConecaster to provide a depth-disambiguated target to GazeGrabInteractor
    /// </summary>
    public class ConecastOrderByDistance : MonoBehaviour
    {
        [SerializeField, ReadOnly] private ARGlassesRig _rig;
        public ARGlassesRig Rig => _rig;

        [SerializeField] private float _leashDistance = 0.05f;
        public void SetHandRange(float handRange) => _leashDistance = handRange;

        [SerializeField, ReadOnly] private float _zWristHeadspace;
        [SerializeField, ReadOnly] private float _zAnchor;
        [SerializeField, ReadOnly] private float _anchorToWrist;
        [SerializeField, ReadOnly] private float _wristZLerp;

        [SerializeField, ReadOnly] private float _minDist;
        [SerializeField, ReadOnly] private float _maxDist;

        [SerializeField] private bool _continuous;
        public void SetContinuous(bool active) => _continuous = active;
        [SerializeField] private float _scrollSpeed = 0.5f;
        [SerializeField, ReadOnly] private float _accumulatedZMotion = 0;
        [SerializeField, ReadOnly] private float _lastWristHeadspace;

        [SerializeField, ReadOnly] private Conecaster _conecaster;
        [SerializeField, ReadOnly] private ARGlassesWristband _wristband;

        public ConecastPrioritizer Prioritizer => _conecaster.Prioritizer;
        private IReadOnlyList<ConecastResult> OrderedHits => _conecaster.OrderedHits;

        private void Awake()
        {
            this.Ancestor(ref _conecaster);
            this.Scene(ref _rig);
            this.Scene(ref _wristband);
        }

        private void Start()
        {
            _wristband.WhenDPad += HandleDPad;
        }

        /// <summary>
        /// EMG ThumbSwipes can drive selection when using SteppedDistance
        /// </summary>
        /// <param name="dir">Normalized Up, Down, Left, Right pagination event</param>
        private void HandleDPad(DPad dPad)
        {
            if (dPad.IsUp()) Further();
            if (dPad.IsDown()) Closer();
        }

        private void Update()
        {
            if (_continuous) ContinuousDistance();
            else SteppedDistance();

            Prioritizer.HoverDistance += Input.GetAxis("Mouse ScrollWheel") * _scrollSpeed;
            if (Input.GetKeyDown(KeyCode.UpArrow)) Further();
            if (Input.GetKeyDown(KeyCode.DownArrow)) Closer();
        }

        void Cycle()
        {
            var index = (Prioritizer.ClosestIndex + 1) % _conecaster.OrderedHits.Count;
            if (index < _conecaster.OrderedHits.Count) Prioritizer.HoverDistance = OrderedHits[index].hitPointDistance;
        }

        void Further()
        {
            var index = Prioritizer.ClosestIndex + 1;
            if (index < _conecaster.OrderedHits.Count) Prioritizer.HoverDistance = OrderedHits[index].hitPointDistance;
        }

        void Closer()
        {
            var index = Prioritizer.ClosestIndex - 1;
            if (index > 0 && index < _conecaster.OrderedHits.Count) Prioritizer.HoverDistance = _conecaster.OrderedHits[index].hitPointDistance;
            else Prioritizer.HoverDistance = 0;
        }

        public Pose WristHeadspace => _rig.HeadPose.InverseTransform(_rig.RightHandSnapshot.Bones.Wrist);

        /// <summary>
        /// Z-Depth selection is continuous.  Called from Update.
        /// Maps the current position of CV Hand to min/max distance objects
        /// This is an alternative to using SteppedDistance()
        /// </summary>
        private void ContinuousDistance()
        {
            _zWristHeadspace = WristHeadspace.position.z;

            _anchorToWrist = _zWristHeadspace - _zAnchor;
            if (_anchorToWrist > _leashDistance) _zAnchor = _zWristHeadspace - _leashDistance;
            if (_anchorToWrist < -_leashDistance) _zAnchor = _zWristHeadspace + _leashDistance;

            _anchorToWrist = _zWristHeadspace - _zAnchor;
            _wristZLerp = (_anchorToWrist + _leashDistance) / (_leashDistance * 2);

            _minDist = _conecaster.OrderedHits.FirstOrDefault().hitPointDistance;
            _maxDist = _conecaster.OrderedHits.LastOrDefault().hitPointDistance;
            Prioritizer.HoverDistance = Mathf.Lerp(_minDist, _maxDist, _wristZLerp);

            // WhenLog($"Reach: {_anchorToWrist:+0.00; -0.00}\nFocus: {_wristZLerp:+0.00; -0.00}");
        }

        /// <summary>
        /// Discretely step between targets in Z-Depth.  Called from Update.
        /// Buffers motion of the CV Hands for a 'stepped' transition
        /// This is an alternative to using ContinuousDistance()
        /// </summary>
        private int _accumulatedZ;
        private void SteppedDistance()
        {
            var wristHeadspace = WristHeadspace.position.z;
            _accumulatedZMotion += wristHeadspace - _lastWristHeadspace;
            _lastWristHeadspace = wristHeadspace;

            if (Mathf.Abs(_accumulatedZMotion) > _leashDistance)
            {
                if (_accumulatedZMotion > 0)
                {
                    Further();
                    _accumulatedZ += 1;
                }
                else
                {
                    Closer();
                    _accumulatedZ -= 1;
                }
                _accumulatedZMotion = 0;
            }

            // WhenLog($"Steps: {_accumulatedZ}\nBuffer: {_accumulatedZMotion:+0.00; -0.00}");
        }
    }
}
