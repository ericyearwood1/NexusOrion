using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static partial class Snapshot
    {
        [Serializable]
        public struct Gaze
        {
            [SerializeField, ReadOnly] private Pose _eyes;
            [SerializeField, ReadOnly] private Pose _head;

            public Gaze(bool eyesTracked, Pose eyes, Pose head)
            {
                _eyes = eyes;
                _head = head;
            }

            public Pose Eyes => _eyes;
            public Pose Head => _head;
        }
    }
}
