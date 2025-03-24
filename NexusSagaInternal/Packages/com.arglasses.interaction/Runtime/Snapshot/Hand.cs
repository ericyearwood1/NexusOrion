using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static partial class Snapshot
    {
        [Serializable]
        public struct Hand
        {
            [SerializeField, ReadOnly] private Side _side;
            [SerializeField, ReadOnly] private Pose _pinch;
            [SerializeField, ReadOnly] private Pose _wrist;
            [SerializeField, ReadOnly] private Pose _ray;
            [SerializeField, ReadOnly] private bool _rayActive;
            [SerializeField, ReadOnly] private float _indexStrength;
            [SerializeField, ReadOnly] private float _middleStrength;
            [SerializeField, ReadOnly] private bool _indexPinching;
            [SerializeField, ReadOnly] private bool _middlePinching;

            [SerializeField, ReadOnly] private HandBones _bones;
            [SerializeField, ReadOnly] private bool _isTracked;

            public Hand(Side side, Pose pinch, Pose wrist, Pose ray, bool rayActive, float indexStrength, float middleStrength, bool indexPinching, bool middlePinching, HandBones bones, bool isTracked)
            {
                _side = side;
                _pinch = pinch;
                _wrist = wrist;
                _ray = ray;
                _rayActive = rayActive;
                _indexStrength = indexStrength;
                _middleStrength = middleStrength;
                _indexPinching = indexPinching;
                _middlePinching = middlePinching;
                _bones = bones;
                _isTracked = isTracked;
            }

            public bool IsTracked
            {
                get => _isTracked;
                set => _isTracked = value;
            }

            public Side Side => _side;
            public Pose Pinch => _pinch;
            public Pose Wrist => _wrist;
            public Pose Ray => _ray;
            public bool RayActive => _rayActive;
            public float IndexStrength => _indexStrength;
            public float MiddleStrength => _middleStrength;
            public bool IndexPinching => _indexPinching;
            public bool MiddlePinching => _middlePinching;
            public HandBones Bones => _bones;
            public readonly bool IsPinching(in HandFinger finger) => (finger.IsMiddle() ? _middlePinching : _indexPinching);
        }

    }
}
