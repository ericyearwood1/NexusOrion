using Robot.Runtime.Data.Robot;

namespace Robot.Runtime.Systems
{
    public class DetectingObjectsHighlightSystem : HighlightSystem<DetectingObjectHighlightView, TargetObjectHighlight>
    {
        protected override void InitialiseView(TargetObjectHighlight data)
        {
            _currentView.Show(_cameraTransform, data.OrderIndex);
        }
    }
}