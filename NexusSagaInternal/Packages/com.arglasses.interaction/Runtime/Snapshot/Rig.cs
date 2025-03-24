using System;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public interface ISnapshotRigProvider
    {
        public Snapshot.Rig Snapshot { get; }
    }

    public static partial class Snapshot
    {
        [Serializable]
        public struct Rig
        {
            [SerializeField, ReadOnly] private Gaze _gaze;
            [SerializeField, ReadOnly] private Hand _leftHand;
            [SerializeField, ReadOnly] private Hand _rightHand;
            [SerializeField, ReadOnly] private Wristband _wristband;
            [SerializeField, ReadOnly] private bool _isUserInHmd;

            public Gaze Gaze => _gaze;
            public Hand LeftHand => _leftHand;
            public Hand RightHand => _rightHand;
            public Wristband Wristband => _wristband;
            public bool IsUserInHmd => _isUserInHmd;
            public IReadOnlyList<Hand> Hands { get; }

            public Pose FromHeadSpace(Pose pose) => _gaze.Head.Transform(pose);
            public Pose ToHeadSpace(Pose pose) => _gaze.Head.InverseTransform(pose);
            public Vector3 FromHeadSpace(Vector3 position) => _gaze.Head.TransformPoint(position);
            public Vector3 ToHeadSpace(Vector3 position) => _gaze.Head.InverseTransformPoint(position);

            public Rig(Gaze gaze, Hand left, Hand right, Wristband wristband, bool isUserInHmd)
            {
                _isUserInHmd = isUserInHmd;
                _gaze = gaze;
                _leftHand = left;
                _rightHand = right;
                _wristband = wristband;
                Hands = new List<Hand> { _leftHand, _rightHand };
            }

            public readonly Hand GetHand(in Side side) => side.IsLeft() ? _leftHand : _rightHand;

            public readonly Side GetPinchingSide(in HandFinger finger)
            {
                var left = _leftHand.IsPinching(finger);
                var right = _rightHand.IsPinching(finger);
                if (left && right) return Side.Both;
                if (left) return Side.Left;
                if (right) return Side.Right;
                return Side.None;
            }
        }
    }
}
