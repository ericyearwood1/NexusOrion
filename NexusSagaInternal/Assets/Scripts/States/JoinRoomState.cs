using System.Collections.Generic;
using Data;
using Multiplayer.Runtime.Data;
using Multiplayer.Runtime.Messages.Client;
using Prime31.StateKit;
using UnityEngine;

namespace States
{
    public class JoinRoomState : SKState<AppData>
    {
        private MultiplayerData _data;

        public override void begin()
        {
            Debug.Log("Begin State:JoinRoomState");
            base.begin();
            _data = _context.MultiplayerData;
            _context.FullFocusCanvasUI.ShowJoiningRoomView();
            _context.MultiplayerHandler.OnRoomStateReceived += OnRoomStateReceived;
            _context.SiroService.SendMessage(JoinRoomMessage.Type,
                new JoinRoomMessage(_data.ThisUser.Room, _data.ThisUser));
        }

        public override void update(float deltaTime)
        {
            
        }

        private void OnRoomStateReceived()
        {
            _context.FullFocusCanvasUI.HideAll();
            Debug.Log("JoinRoomState:OnRoomStateReceived");
            _context.MultiplayerData.Room.WorldGraphAnchors ??= new List<SpatialAnchorData>();
            _context.MultiplayerHandler.OnRoomStateReceived -= OnRoomStateReceived;
            _machine.changeState<InitialisingRoomState>();
        }
    }
}