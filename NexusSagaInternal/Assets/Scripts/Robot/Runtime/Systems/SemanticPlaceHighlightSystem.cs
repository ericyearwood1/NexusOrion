using Robot.Runtime.Data;
using Robot.Runtime.Data.Robot;
using Robot.Runtime.View;
using UnityEngine;

namespace Robot.Runtime.Systems
{
    public class SemanticPlaceHighlightSystem : HighlightSystem<SemanticPlaceMarker, SemanticPlaceLocationData>
    {
        private RobotData _robotData;
        private Quaternion _upOrientation;
        private Vector3 _targetEuler;

        public void Initialise(Transform cameraTransform, RobotData robotData)
        {
            _cameraTransform = cameraTransform;
            _robotData = robotData;
            _upOrientation = Quaternion.identity;
        }
        
        public override void Tick()
        {
            base.Tick();
            if (_currentView== null) return;
            try
            {
                _currentView.SetIndicatorPosition(_robotData.Display.EEGripPosition.position);
                _currentView.SetIndicatorOrientation(_robotData.Display.EEGripPosition.rotation);
            }
            catch
            {
                Debug.LogError("Error setting indicator position and orientation");
            }
            
            if (_robotData.GripperState != GripperState.Holding)
            {
                HideAllMarkers();
            }
        }
        
        protected override void InitialiseView(SemanticPlaceLocationData data)
        {
            _currentView.Show(_cameraTransform);
            _currentView.transform.SetParent(_robotData.Display.transform, false);
            _upOrientation = Quaternion.Euler(data.TargetEuler); 
            _currentView.SetUpIndicatorOrientation(_upOrientation);
        }
    }
}