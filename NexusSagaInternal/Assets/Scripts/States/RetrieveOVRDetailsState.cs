using Oculus.Platform;
using UnityEngine;
using User = Oculus.Platform.Models.User;

namespace States
{
    public class RetrieveOVRDetailsState : BaseInitialiseUserState
    {
        public override void begin()
        {
            base.begin();
            Core.Initialize();
            Users.GetLoggedInUser().OnComplete(ProcessUserData);
        }

        private void ProcessUserData(Message<User> message)
        {
            var isLoggedInUserMessage = message.Type == Message.MessageType.User_GetLoggedInUser;
            if (!isLoggedInUserMessage)
            {
                return;
            }

            if (message.IsError)
            {
                _context.FatalError = $"Error getting user details. Please restart app.\n{message.GetError()}";
                _machine.changeState<FatalErrorState>();
                return;
            }

            var ovrUser = message.GetUser();
            Debug.Log($"InitialiseOVRUserState::OVR USER {ovrUser.ID} | {ovrUser.DisplayName} | {ovrUser.OculusID}");
            var user = _context.MultiplayerData.ThisUser;
            user.DisplayName = string.IsNullOrWhiteSpace(ovrUser.DisplayName) ? user.DisplayName : ovrUser.DisplayName;
            user.Id = ovrUser.ID;
            _context.UserIdString = user.Id.ToString();
            /*if (user.Id == 0)
            {
                _context.FatalError = "No log in details found. Please log in and restart the application.";
                _machine.changeState<FatalErrorState>();
                return;
            }*/

            ProcessPlatformDetails(user.Id);
        }
    }
}