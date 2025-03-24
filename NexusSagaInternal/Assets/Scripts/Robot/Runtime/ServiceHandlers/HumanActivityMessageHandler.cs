using System;
using Newtonsoft.Json;
using Robot.Runtime.Messages.Server.HumanActivity;
using SiroComms.Runtime.Messages.Server;
using SiroComms.Runtime.Services;

namespace Robot.Runtime.ServiceHandlers
{
    public class HumanActivityMessageHandler : IMessageHandler
    {
        public const string TYPE = "human_activity";
        public const string HUMAN_ACTIVITY = "human_activity";
        
        public Action<HumanActivityMessage> OnHumanActivity;
        private bool _isEnabled;

        public void HandleMessage(string messageType, string message)
        {
            switch (messageType)
            {
                case HUMAN_ACTIVITY:
                    HandleHumanActivity(message);
                    break;
            }            
        }

        private void HandleHumanActivity(string message)
        {
            if (!_isEnabled) return;
            var humanActivityMessage = ServerMessage<HumanActivityMessage>.Deserialize(message).Data;
            OnHumanActivity?.Invoke(humanActivityMessage);
        }

        public void Enable()
        {
            _isEnabled = true;
        }
        
        public void Disable()
        {
            _isEnabled = false;
        }

        public void SetEnabled(bool isEnabled)
        {
            if (isEnabled)
            {
                Enable();
            }
            else
            {
                Disable();
            }
        }
    }
}