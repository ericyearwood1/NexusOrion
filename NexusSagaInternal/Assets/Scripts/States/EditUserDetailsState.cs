using System.Collections.Generic;
using Data;
using Multiplayer.Runtime.Data;
using Prime31.StateKit;
using TMPro;
using UnityEngine;
using View;

namespace States
{
    public class EditUserDetailsState : SKState<AppData>
    {
        private ConfigureUserDetailsView _view;
        private List<TMP_Dropdown.OptionData> _roomList;

        public override void begin()
        {
            base.begin();
            var roomList = _context.MultiplayerData.RoomList;
            var roomListOptions = new List<TMP_Dropdown.OptionData>(roomList.Length);
            foreach (var roomId in roomList)
            {
                roomListOptions.Add(new TMP_Dropdown.OptionData(roomId));
            }
            _view = _context.FullFocusCanvasUI.ShowUserDetailsView();
            _view.Initialise(_context.MultiplayerData.ThisUser, _context.ColorConfig, roomListOptions);
            _view.OnUserDetailsConfirmed += GoToNextState;
        }

        public override void update(float deltaTime)
        {
        }

        private void GoToNextState()
        {
            PlayerPrefs.SetString(Consts.PlayerPref_UserId, $"{_context.MultiplayerData.ThisUser.Id}");
            PlayerPrefs.SetInt(Consts.PlayerPref_HasSaveDetails, 1);
            _view.OnUserDetailsConfirmed -= GoToNextState;
            _context.EnableSagaPlugin();
            _machine.changeState<JoinRoomState>();
        }
        
    }
}