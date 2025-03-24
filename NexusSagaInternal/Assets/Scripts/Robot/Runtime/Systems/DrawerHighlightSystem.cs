using Robot.Runtime.Data;
using Robot.Runtime.Data.Robot;
using Robot.Runtime.View;
using UnityEngine;

namespace Robot.Runtime.Systems
{
    public class DrawerHighlightSystem : HighlightSystem<DrawerMarker, HighlightData>
    {
        private RobotData _robotData;

        public void Initialise(Transform cameraTransform, Transform highlightContainer, GameObject highlightPrefab,
            RobotData robotData, Color color)
        {
            base.Initialise(cameraTransform, highlightContainer, highlightPrefab, color,20);
            _robotData = robotData;
        }

        public override void Tick()
        {
            base.Tick();
            if (_currentView == null) return;
            if (_robotData.GripperState != GripperState.Holding) return;
            _currentView.TrackingHighlight.position = _robotData.Display.EEGripPosition.position;
        }

        protected override void InitialiseView(HighlightData data)
        {
            _currentView.Show(_cameraTransform);
        }
    }
}