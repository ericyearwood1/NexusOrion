using Multiplayer.Runtime.Services;
using Robot.Runtime.ServiceHandlers;

namespace Systems
{
    public class ActiveHARWithWorldVisualisationSystem : IMultiplayerFeatureService
    {
        private HumanActivityMessageHandler _handler;

        public ActiveHARWithWorldVisualisationSystem(HumanActivityMessageHandler handler)
        {
            _handler = handler;
        }

        public void Activate()
        {
            _handler.Enable();
        }

        public void Deactivate()
        {
            _handler.Disable();
        }
    }
}