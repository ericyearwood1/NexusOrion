using Robot.Runtime.Data.Robot;
using Robot.Runtime.View;

namespace Robot.Runtime.Systems
{
    public class NavigationHighlightSystem : HighlightSystem<NavigationHighlightView, LabelHighlightData>
    {
        public override void OnHighlightReceived(LabelHighlightData highlightDataData)
        {
            if (_currentView != null)
            {
                _currentView.OnActionComplete();
                _currentView = null;
            }

            base.OnHighlightReceived(highlightDataData);
        }
        
        protected override void InitialiseView(LabelHighlightData data)
        {
            _currentView.Show(_cameraTransform, data.Label);
        }
    }
}