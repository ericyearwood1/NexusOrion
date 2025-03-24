using UnityEngine;

namespace SiroComms.Runtime.Services
{
    public class RobotArmSimulation
    {
        private const float robotSpeed = 1.5f;
        private SimulationState _state;
        private float _startTime;
        private Vector3 _targetPosition;
        private float _currentYaw;
        private float _robotStartYaw;
        private Vector3 _currentPosition;
        private float _totalTime;
        private float _currentTime;
        private Vector3 _robotStartPosition;
        private float _targetYaw;
        private readonly Vector3 _homePosition;
        private const float _wobbleDegrees = 15; 
        private const float _wobbleFrequency = 0.5f;

        public RobotArmSimulation(Vector3 homePosition)
        {
            _homePosition = homePosition;
            _currentPosition = homePosition;
        }

        public SimulationState State => _state;
        public float CurrentYaw => _currentYaw;
        public Vector3 CurrentPosition => _currentPosition;
        public Quaternion CurrentRotation => WobbleRotation();


        private Quaternion WobbleRotation()
        {
            var wobbleP = Mathf.Sin(Time.time * _wobbleFrequency) * _wobbleDegrees;
            var wobbleR = Mathf.Cos(Time.time * _wobbleFrequency) * _wobbleDegrees;
            var wobbleY = Mathf.Sin(Time.time * _wobbleFrequency) * _wobbleDegrees;
            
            return Quaternion.Euler(wobbleP, wobbleR, wobbleY);
            
        }
        public void NavigateToPosition(Vector3 position, float targetYaw = 0)
        {
            _state = SimulationState.None;
            _targetYaw = targetYaw;
            _targetPosition = position;
            StartMoving();
        }

        public void ReturnToHome()
        {
            _targetYaw = 0;
            _targetPosition = _homePosition;
            _targetPosition.y = _homePosition.y;
            StartMoving();
        }

        private void StartMoving()
        {
            _currentTime = 0;
            _robotStartPosition = _currentPosition;
            _robotStartYaw = _currentYaw;
            var distance = Vector3.Distance(_targetPosition, _robotStartPosition);
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
                    _currentPosition = Vector3.Lerp(_robotStartPosition, _targetPosition, _currentTime / _totalTime);
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