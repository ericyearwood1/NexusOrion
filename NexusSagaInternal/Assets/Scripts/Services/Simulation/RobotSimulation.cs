using UnityEngine;

namespace SiroComms.Runtime.Services
{
    public class RobotSimulation
    {
        private const float robotSpeed = 1.5f;
        private SimulationState _state;
        private float _startTime;
        private Vector2 _targetPosition;
        private float _currentYaw;
        private float _robotStartYaw;
        private Vector2 _currentPosition;
        private float _totalTime;
        private float _currentTime;
        private Vector2 _robotStartPosition;
        private float _targetYaw;
        private readonly Vector2 _homePosition;

        public RobotSimulation(Vector3 homePosition)
        {
            _homePosition = new Vector2(homePosition.x, homePosition.z);
            _currentPosition = homePosition;
        }

        public SimulationState State => _state;
        public float CurrentYaw => _currentYaw;
        public Vector3 CurrentPosition => _currentPosition;
        public Quaternion CurrentRotation => Quaternion.identity;

        public void NavigateToPosition(Vector3 position, float targetYaw = 0)
        {
            _state = SimulationState.None;
            _targetYaw = targetYaw;
            _targetPosition = new Vector2(position.x, position.z);
            StartMoving();
        }

        public void ReturnToHome()
        {
            _targetYaw = 0;
            _targetPosition = _homePosition;
            StartMoving();
        }

        private void StartMoving()
        {
            _currentTime = 0;
            _robotStartPosition = _currentPosition;
            _robotStartYaw = _currentYaw;
            var distance = Vector2.Distance(_targetPosition, _robotStartPosition);
            _totalTime = distance / robotSpeed;
            _state = SimulationState.Starting;
        }

        public void Tick()
        {
            switch (_state)
            {
                case SimulationState.None:
                    break;
                case SimulationState.Starting:
                    _state = SimulationState.InProgress;
                    _currentTime = 0;
                    _startTime = Time.realtimeSinceStartup;
                    break;
                case SimulationState.InProgress:
                    _currentTime += Time.deltaTime;
                    var progress = _currentTime / _totalTime;
                    _currentPosition = Vector2.Lerp(_robotStartPosition, _targetPosition, _currentTime / _totalTime);
                    _currentYaw = Mathf.LerpAngle(_robotStartYaw, _targetYaw, progress);
                    if (progress >= 1)
                    {
                        _state = SimulationState.Success;
                    }
                    break;
            }
        }

        public void ResetState()
        {
            _state = SimulationState.None;
        }
    }
}