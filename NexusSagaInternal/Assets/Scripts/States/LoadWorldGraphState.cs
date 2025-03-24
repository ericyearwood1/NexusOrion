using Data;
using Prime31.StateKit;
using Robot.Runtime.Data;
using UnityEngine;
using View;

namespace States
{
    public class LoadWorldGraphState : SKState<AppData>
    {
        private ConfigureWorldGraphPanelsView _view;

        public override void begin()
        {
            Debug.Log("ChangeState::LoadWorldGraphState");
            _view = _context.FullFocusCanvasUI.ShowSetUpWorldGraphPanelsView();
            if (_context.WorldGraphVisualsSystem.IsWorldGraphInitialised)
            {
                OnWorldGraphProcessed();
                return;
            }
            
            ShowMessage("Loading World Graph");
            _context.WorldGraphVisualsSystem.OnWorldGraphProcessed += OnWorldGraphProcessed;
            _context.SiroService.SendRequest(ServiceEvents.Get_World_Graph);
        }
        
        private void OnWorldGraphProcessed()
        {
            _context.WorldGraphVisualsSystem.OnWorldGraphProcessed -= OnWorldGraphProcessed;

            var roomList = _context.WorldGraphVisualsSystem.RoomList;
            if (roomList.Count == 0)
            {
                _context.FatalError = 
                    "World graph room list is empty. Has the robot been walked around the demo area? Please retry once the world graph has been created.";
                _machine.changeState<FatalErrorState>();
                return;
            }
            _view.Hide();
            if (_context.MultiplayerData.IsThisUserAHost())
            {
                _machine.changeState<ConfigureWorldGraphPanelsState>();
            }
            else
            {
                _machine.changeState<WaitingForWorldGraphPanelsState>();
            }
        }

        private void ShowMessage(string message)
        {
            _view.ShowGeneralMessage(message);
        }

        public override void update(float deltaTime)
        {
        }
    }
}