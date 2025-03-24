using Robot.Runtime.Data;
using Robot.Runtime.Data.Robot;
using Robot.Runtime.View;
using UnityEngine;

namespace Robot.Runtime.Systems
{
    public class TargetObjectHighlightSystem : HighlightSystem<TargetObjectHighlightView, TargetObjectHighlight>
    {
        private RobotData _robotData;
        private float _fadeOutDelay=2.0f;

        public void Initialise(Transform cameraTransform, Transform highlightContainer, GameObject highlightPrefab,
            RobotData robotData, Color color)
        {
            base.Initialise(cameraTransform, highlightContainer, highlightPrefab, color,1);
            _robotData = robotData;
        }
        
        protected override void InitialiseView(TargetObjectHighlight data)
        {
            _currentView.ShowDroppedState();
            _currentView.Show(_cameraTransform, data.Label);
        }
        
        public override void Tick()
        {
            base.Tick();
            if (_currentView == null) return;
            if (_robotData.GripperState != GripperState.Holding) return;
            _currentView.transform.position = _robotData.Display.EEGripPosition.position;
        }

        public void HideMarkerWithDelay(float delay)
        {
            if (_currentView == null) return;
            _currentView.ShowDroppedState();
            _currentView.Hide(HighlightView.DefaultFadeOutTime, delay);
        }

        public void ProcessGripperHolding()
        {
            if (_currentView == null) return;
            _currentView.ShowCarryState(_fadeOutDelay);
        }
    }
}