using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class ScrollModelVideo : ScrollModel
    {
        [SerializeField, ReadOnly] private VideoModel _videoModel;

        public float _scrollBufferTime = 0.2f;
        [SerializeField] private float _metersPerSecond = 1f;
        private float _lastScrollPositionUpdateTime;
        private float _pendingScrollPosition;
        private float _destinationTimestamp = 0;
        private bool _scrollDistanceDirty;

        [Tooltip("Pause for X seconds while scrubbing")] [SerializeField]
        private float _scrubbingPauseVideoDuration = 0.5f;

        public override bool IsHorizontal => true;

        private void Awake()
        {
            this.Descendant(ref _videoModel, allowSibling: true);
        }

        [SerializeField] private float _secondsPerLandmark = 15f;
        [SerializeField, ReadOnly] private bool _isGrabbed;
        public int LandmarkCount => (int)(_videoModel.VideoPlayer.length / _secondsPerLandmark) + 1;

        public override List<float> PaginationLandmarks => Enumerable.Range(0, LandmarkCount)
            .Select(n => n * _secondsPerLandmark * _metersPerSecond).ToList();


        public override float ContainerMax => _metersPerSecond * (float)_videoModel.VideoPlayer.length;

        protected override float GetMeters()
        {
            var useVideoTime = Time.time - _lastScrollPositionUpdateTime < _scrollBufferTime * 2;
            return (useVideoTime ? _pendingScrollPosition : (float)_videoModel.VideoPlayer.time) * _metersPerSecond;
        }

        protected override void SetMeters(float value)
        {
            _scrollDistanceDirty = true;
            _pendingScrollPosition = value / _metersPerSecond;
        }

        public override void GrabChanged(bool isGrabbed) => _isGrabbed = isGrabbed;

        private float VideoEndZone => (float)_videoModel.VideoPlayer.length - 0.01f;

        private void Update()
        {
            if (_destinationTimestamp != 0 && _destinationTimestamp < Time.time && _pendingScrollPosition < VideoEndZone)
            {
                _videoModel.IsPlaying = true;
            }

            if (!_scrollDistanceDirty || Time.time - _lastScrollPositionUpdateTime < _scrollBufferTime) return;

            var wasPlaying = _videoModel.IsPlaying;
            _videoModel.IsPlaying = false;
            _destinationTimestamp = wasPlaying ? Time.time + _scrubbingPauseVideoDuration : 0;
            _scrollDistanceDirty = false;
            _lastScrollPositionUpdateTime = Time.time;

            if (_pendingScrollPosition > VideoEndZone) _pendingScrollPosition = VideoEndZone;

            _videoModel.VideoPlayer.time = _pendingScrollPosition;
        }
    }
}
