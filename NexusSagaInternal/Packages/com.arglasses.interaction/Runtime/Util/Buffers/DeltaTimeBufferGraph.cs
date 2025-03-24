using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class DeltaTimeBufferGraph : MonoBehaviour
    {
        [SerializeField] private float _lineWidth = 0.0008f;
        [SerializeField] private bool _clampLines = true;

        [SerializeField] private float _duration = 3f;
        [SerializeField] private bool _updateDurationFromBuffers = true;

        [SerializeField] private float _yMagnitudeMin = 0.1f;

        [SerializeField] private float _rangeResetDelay = 1f;

        [SerializeField] private TextMeshProUGUI _topLeftText;
        [SerializeField] private TextMeshProUGUI _bottomLeftText;
        [SerializeField] private List<TextMeshProUGUI> _eventLabelPool = new ();

        [Serializable]
        private struct BufferEntry
        {
            public DeltaTimeBuffer Buffer;
            public Color Color;
            public float LineWidthScale;

            public BufferEntry(DeltaTimeBuffer buffer, Color color, float lineWidthScale = 1f)
            {
                Buffer = buffer;
                Color = color;
                LineWidthScale = lineWidthScale;
            }
        }

        [SerializeField, ReadOnly] private List<BufferEntry> _buffers = new();
        private float _currentYMax;
        private float _currentYMin;

        private void Awake()
        {
            this.Descendant(ref _topLeftText, nameStrict: nameof(_topLeftText), optional: true);
            this.Descendant(ref _bottomLeftText, nameStrict: nameof(_bottomLeftText), optional: true);


        }

        public void AddBuffer(DeltaTimeBufferMulti bufferMulti, float lineWidthScale=1) => AddBuffer(bufferMulti, Color.red, Color.green, Color.blue, lineWidthScale);

        public void AddBuffer(DeltaTimeBufferMulti bufferMulti, Color c0, Color c1, Color c2, float lineWidthScale=1)
        {
            AddBuffer(bufferMulti[0], c0, lineWidthScale);
            AddBuffer(bufferMulti[1], c1, lineWidthScale);
            AddBuffer(bufferMulti[2], c2, lineWidthScale);
        }

        public void AddBuffer(DeltaTimeBuffer buffer, Color c, float lineWidthScale = 1f)
        {
            _buffers.Add(new BufferEntry(buffer, c, lineWidthScale));
        }

        public void Clear()
        {
            _buffers.Clear();
        }

        private int _eventLabelIndex = 0;
        private void LateUpdate()
        {
            UpdateRange();

            var rectSize = transform.GetRectSize();
            var xScale = rectSize.x / Mathf.Max(_duration, 0.0001f);
            var yScale = rectSize.y / Mathf.Max(_currentYMax - _currentYMin, 0.0001f);
            var yZero = rectSize.y * (_currentYMax + _currentYMin) * 0.5f;

            _eventLabelIndex = 0;
            for (var i = 0; i < _eventLabelPool.Count; i++) _eventLabelPool[i].enabled = false;

            for (int i = 0; i < _buffers.Count; i++) DrawBuffer(_buffers[i], rectSize, xScale, yScale, yZero);

            var zeroLine = -yZero * yScale;
            var from = transform.TransformPoint(new Vector3(-rectSize.x * 0.5f, zeroLine));
            var to = transform.TransformPoint(new Vector3(rectSize.x * 0.5f, zeroLine));
            Line.Draw(from, to, Color.magenta, _lineWidth);
        }


        private float _lastRangeChangeTime;
        private void UpdateRange()
        {
            if (_updateDurationFromBuffers)
            {
                _duration = 0;
                for (var i = 0; i < _buffers.Count; i++) _duration = Mathf.Max(_buffers[i].Buffer.Duration, _duration);
            }

            var max = _yMagnitudeMin;
            var min = -_yMagnitudeMin;
            for (var i = 0; i < _buffers.Count; i++)
            {
                max = Mathf.Max(_buffers[i].Buffer.Max, max);
                min = Mathf.Min(_buffers[i].Buffer.Min, min);
            }

            var shouldResetRange = max > _currentYMax || min < _currentYMin || Time.time - _lastRangeChangeTime > _rangeResetDelay;
            if (shouldResetRange)
            {
                _lastRangeChangeTime = Time.time;
                _currentYMax = max;
                _currentYMin = min;

                if (_topLeftText) _topLeftText.text = $"{_currentYMax:+0.00; -0.00}";
                if (_bottomLeftText) _bottomLeftText.text = $"{_currentYMin:+0.00; -0.00}";
            }
        }

        public Dictionary<float, string> Tags = new();

        private void DrawBuffer(BufferEntry bufferEntry, Vector2 rectSize, float xScale, float yScale, float yZero = 0)
        {
            var buffer = bufferEntry.Buffer;
            var color = bufferEntry.Color;
            var lineWidth = _lineWidth * bufferEntry.LineWidthScale;

            Vector3 ToLocalSpace(DeltaTimeBuffer.Event e)
            {
                var dataStartsFromRightSide = rectSize.x * 0.5f;
                var localX = (e.BufferTime - buffer.AccumulatedTime) * xScale + dataStartsFromRightSide;
                var localY = (e.Val - yZero) * yScale;
                return new Vector3(localX, localY, 0);
            }

            for (var i = 0; i < buffer.Count -1; i++)
            {
                var fromEvent = buffer[i];
                var toEvent = buffer[i + 1];

                var fromLocal = ToLocalSpace(fromEvent);
                var toLocal = ToLocalSpace(toEvent);
                var leftCutoff = -rectSize.x;
                if (_clampLines && toLocal.x < leftCutoff) continue;

                var fromWorld = transform.TransformPoint(fromLocal);
                var toWorld = transform.TransformPoint(toLocal);

                if (Tags.TryGetValue(fromEvent.BufferTime, out var tag))
                {
                    var label = _eventLabelPool[_eventLabelIndex++ % _eventLabelPool.Count];
                    label.enabled = true;
                    label.text = $"{tag}";

                    if(tag.Equals("Push")) label.color = Color.cyan;
                    else if(tag.Equals("Pull")) label.color = Color.magenta;
                    else  label.color = Color.white;

                    label.transform.position = toWorld;
                    label.transform.localPosition += Vector3.up * label.rectTransform.rect.height * (fromEvent.Val > 0 ? 0.5f : -0.5f);
                }

                Line.Draw(fromWorld, toWorld, color, lineWidth);
            }
        }
    }
}
