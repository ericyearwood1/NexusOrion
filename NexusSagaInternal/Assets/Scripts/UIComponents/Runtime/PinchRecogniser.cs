using System;
using Oculus.Interaction.Input;
using UnityEngine;

namespace UIComponents.Runtime
{
    public class PinchRecogniser
    {
        public Action OnGesture;
        
        private HandRef _hand;
        private State _currentState;
        private float _gestureTime;
        private float _currentGestureTime;

        private enum State
        {
            None,
            Idle,
            Started,
            InProgress,
            GestureComplete,
            WaitingForReset,
            Ended
        }

        public bool IsGesture => _currentState == State.GestureComplete;

        public void Initialise(HandRef hand, float gestureTime = 0f)
        {
            _hand = hand;
            _gestureTime = gestureTime;
            _currentState = State.Idle;
        }

        public void Tick()
        {
            switch (_currentState)
            {
                case State.Idle :
                    if (_hand.GetIndexFingerIsPinching())
                    {
                        _currentState = State.Started;
                    }
                    break;
                case State.Started :
                    _currentGestureTime = 0f;
                    _currentState = _hand.GetIndexFingerIsPinching() ? State.InProgress : State.Ended;
                    break;
                case State.InProgress : 
                    if (_hand.GetIndexFingerIsPinching())
                    {
                        _currentGestureTime += Time.deltaTime;
                        if (_currentGestureTime >= _gestureTime)
                        {
                            _currentState = State.GestureComplete;
                        }
                    }
                    else
                    {
                        _currentState = State.Ended;
                    }
                    break;
                case State.GestureComplete :
                    OnGesture?.Invoke();
                    _currentState = State.WaitingForReset;
                    break;
                case State.WaitingForReset:
                    if (!_hand.GetIndexFingerIsPinching())
                    {
                        _currentState = State.Ended;
                    }
                    break;
                case State.Ended :
                    _currentGestureTime = 0;
                    _currentState = State.Idle;
                    break;
            }
        }
    }
}