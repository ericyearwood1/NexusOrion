using Data;
using Multiplayer.Runtime.Data;
using Prime31.StateKit;
using UnityEngine;

namespace States
{
    public class BaseInitialiseUserState : SKState<AppData>
    {
        public override void begin()
        {
            base.begin();
            _context.FullFocusCanvasUI.ShowRetrievingPlatformDetails();
        }
        
        protected void ProcessPlatformDetails(ulong userId)
        {
            var userData = _context.MultiplayerData.ThisUser;
            _context.IsUserSaveAvailable = PlayerPrefs.GetInt(Consts.PlayerPref_HasSaveDetails) == 1;
            ProcessUserId(userData, userId);
            ProcessUserType(userData);
            ProcessUsername(userData);
            ProcessUserColor(userData);
            
            if (_context.IsUserSaveAvailable)
            {
                _machine.changeState<ConfirmUserDetailsState>();
            }
            else
            {
                _machine.changeState<EditUserDetailsState>();
            }
        }

        private void ProcessUserId(User userData, ulong userId)
        {
            var saveData = PlayerPrefs.GetString(Consts.PlayerPref_UserId);
            userData.Id = userId;
            _context.UserIdString = userId.ToString();
            if (string.IsNullOrWhiteSpace(saveData))
            {
                _context.IsUserSaveAvailable = false;
                return;
            }
            var parsedId = ulong.Parse(saveData);
            if (parsedId != userId)
            {
                _context.IsUserSaveAvailable = false;
            }
        }

        private void ProcessUserType(User userData)
        {
            var saveData = PlayerPrefs.GetInt(Consts.PlayerPref_UserType);
            if (saveData is <= 0 or > 3)
            {
                _context.IsUserSaveAvailable = false;
                return;
            }
            var userType = (UserType)saveData;
            userData.UserType = userType;
        }
        
        private void ProcessUsername(User userData)
        {
            var saveData = PlayerPrefs.GetString(Consts.PlayerPref_Username);
            if (string.IsNullOrWhiteSpace(saveData))
            {
                _context.IsUserSaveAvailable = false;
                return;
            }
            userData.DisplayName = saveData;
        }
        
        private void ProcessUserColor(User userData)
        {
            var saveData = PlayerPrefs.GetInt(Consts.PlayerPref_Color);
            if (saveData < 0 || saveData > _context.ColorConfig.Options.Length - 1)
            {
                _context.IsUserSaveAvailable = false;
                return;
            }
            userData.Color = saveData;
        }


        public override void update(float deltaTime)
        {
        }
    }
}