using System;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class ARGlassesWristbandPushPullGraph : MonoBehaviour
    {
        [SerializeField] private DeltaTimeBufferGraph _graph;
        [SerializeField, ReadOnly] private ARGlassesWristbandPushPull _pushPull;

        public DeltaTimeBuffer _pushPullTags = new();
        public DeltaTimeBuffer PushPullTags => _pushPullTags;

        private void Awake()
        {
            this.Ensure(ref _graph);
            this.Scene(ref _pushPull);
        }

        private void Start()
        {
            _graph.AddBuffer(_pushPull.Detector.AccelerationBuffer);
            _graph.AddBuffer(_pushPullTags, Color.cyan);
            _graph.Tags = _pushPull.Detector.Tags;
        }
    }
}
