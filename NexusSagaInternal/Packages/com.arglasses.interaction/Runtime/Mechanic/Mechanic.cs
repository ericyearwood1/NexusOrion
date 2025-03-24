using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static partial class Mechanic
    {
        public interface IPressDetector
        {
            public void Sync(bool isPinching, bool wasPinching);
        }

        public class PressDetector
        {
            public const float PressTimingThreshold = 0.5f;

            [Serializable]
            public class Short : IPressDetector
            {
                [SerializeField, ReadOnly] private bool _isShortPressing;
                public bool IsShortPressing => _isShortPressing;

                [SerializeField, ReadOnly] private float _lastPinchBeginTime;
                public event Action WhenShortPress = delegate { };

                public void Sync(bool isPinching, bool wasPinching)
                {
                    if (isPinching && !wasPinching)
                    {
                        _isShortPressing = true;
                        _lastPinchBeginTime = Time.time;
                    }

                    if (_isShortPressing && Time.time - _lastPinchBeginTime > PressTimingThreshold)
                    {
                        _isShortPressing = false;
                    }

                    if (!isPinching && wasPinching)
                    {
                        if (_isShortPressing) WhenShortPress?.Invoke();
                        _isShortPressing = false;
                    }
                }

                public void Clear()
                {
                    _isShortPressing = false;
                    _lastPinchBeginTime = default;
                }
            }

            [Serializable]
            public class Long : IPressDetector
            {
                [SerializeField, ReadOnly] private float _lastPressBeginTime;
                [SerializeField, ReadOnly] private bool _waiting;
                [SerializeField, ReadOnly] private bool _isLongPressing;
                public bool IsLongPressing => _isLongPressing;
                public event Action WhenLongPress = delegate { };

                public void Sync(bool isPinching, bool wasPinching)
                {
                    if (!isPinching && wasPinching)
                    {
                        _lastPressBeginTime = default;
                        _isLongPressing = false;
                    }

                    if (isPinching && !wasPinching)
                    {
                        _lastPressBeginTime = Time.time;
                        _isLongPressing = false;
                    }

                    var waitingForLongPinch = !_isLongPressing && _lastPressBeginTime != default;
                    var longPinchDurationExceeded = Time.time - _lastPressBeginTime > PressTimingThreshold;
                    if (waitingForLongPinch && longPinchDurationExceeded)
                    {
                        WhenLongPress?.Invoke();
                        _isLongPressing = true;
                    }
                }

                public void Clear()
                {
                    _lastPressBeginTime = default;
                    _isLongPressing = false;
                }
            }

            [Serializable]
            public class Double : IPressDetector
            {
                [SerializeField, ReadOnly] private bool _isDoublePressing;
                public bool IsDoublePressing => _isDoublePressing;

                [SerializeField, ReadOnly] private float _lastPressBeginTime = default;
                public event Action WhenDoublePress = delegate { };

                public void Sync(bool isPressing, bool wasPressing)
                {
                    if (isPressing && !wasPressing)
                    {
                        if (Time.time - _lastPressBeginTime < PressTimingThreshold)
                        {
                            WhenDoublePress?.Invoke();
                            _isDoublePressing = true;
                        }

                        _lastPressBeginTime = Time.time;
                    }

                    if (!isPressing && wasPressing) _isDoublePressing = false;
                }

                public void Clear()
                {
                    _lastPressBeginTime = default;
                    _isDoublePressing = false;
                }
            }
        }
    }
}
