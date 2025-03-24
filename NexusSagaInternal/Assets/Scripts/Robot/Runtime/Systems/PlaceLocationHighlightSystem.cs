using Robot.Runtime.Data.Robot;
using Robot.Runtime.View;

namespace Robot.Runtime.Systems
{
    public class PlaceLocationHighlightSystem : HighlightSystem<PlaceLocationMarker, PlaceLocationData>
    {
        protected override void InitialiseView(PlaceLocationData data)
        {
            _currentView.Show(_cameraTransform);
        }
    }
}