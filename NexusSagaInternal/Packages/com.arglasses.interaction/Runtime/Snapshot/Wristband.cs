using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static partial class Snapshot
    {
        [Serializable]
        public struct Wristband
        {
            [SerializeField, ReadOnly] private bool _isConnected;
            [SerializeField, ReadOnly] private bool _index;
            [SerializeField, ReadOnly] private bool _middle;
            [SerializeField, ReadOnly] private DPad _dPad;
            [SerializeField, ReadOnly] private Quaternion _orientationNotAlignedInYaw;
            [SerializeField, ReadOnly] private Vector2 _projectedRadians;
            [SerializeField, ReadOnly] private float _twistDegrees;
            [SerializeField, ReadOnly] private Side _side;
            [SerializeField, ReadOnly] private DiscreteHandMotion _handMotion;

            public Wristband(bool isPinchingIndex, bool isPinchingMiddle, DPad dPad, DiscreteHandMotion handMotion, Quaternion orientationNotAlignedInYaw, Vector2 projectedRadians, float twistDegrees, bool isConnected, Side side)
            {
                _index = isPinchingIndex;
                _middle = isPinchingMiddle;
                _dPad = dPad;
                _handMotion = handMotion;
                _orientationNotAlignedInYaw = orientationNotAlignedInYaw;
                _projectedRadians = projectedRadians;
                _twistDegrees = twistDegrees;
                _isConnected = isConnected;
                _side = side;
            }

            public bool IsConnected => _isConnected;
            public bool Index => _index;
            public bool Middle => _middle;
            public DPad DPad => _dPad;
            public DiscreteHandMotion DiscreteHandMotion => _handMotion;
            public Quaternion OrientationNotAlignedInYaw => _orientationNotAlignedInYaw;
            public Vector2 ProjectedRadians => _projectedRadians;
            public float TwistDegrees => _twistDegrees;
            public readonly Side Side => _side;

            public readonly bool IsPinching(in Side side, in HandFinger finger)
            {
                if (side != Side) return false;
                if (finger.IsIndex()) return _index;
                if (finger.IsMiddle()) return _middle;
                return false;
            }
        }
    }
}
