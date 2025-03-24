using Data;
using Multiplayer.Runtime.Data;
using Prime31.StateKit;
using UnityEngine;

namespace States
{
    public class CreateRobotHomeAnchorState : SKState<AppData>
    {
        private MultiplayerData _data;

        public override void begin()
        {
            base.begin();
            _data = _context.MultiplayerData;
            _context.FullFocusCanvasUI.HideAll();
            Debug.Log($"CreateRobotHomeAnchorState {_data.RobotHomeAnchor}");
            if (_data.RobotHomeAnchor == null)
            {
                _machine.changeState<ThreeDotRobotSyncState>();
            }
            else
            {
                _context.FullFocusCanvasUI.ShowCreatingRobotHomeAnchor();
                CheckRobotHomeAnchorReady();
            }
        }
        
        public override void update(float deltaTime)
        {
            if (_data.RobotHomeAnchor == null) return;
            CheckRobotHomeAnchorReady();
        }

        private void CheckRobotHomeAnchorReady()
        {
            var robotHomeAnchor = _data.RobotHomeAnchor;
            var robotHomeData = robotHomeAnchor.Data;
            if (robotHomeData.State == SpatialAnchorState.Error)
            {
                // @TODO not fatal, but keeping for dev purposes for now
                Debug.LogError("Error loading robot home anchor");
                _context.FatalError = "Error loading robot home anchor";
                _machine.changeState<FatalErrorState>();
                return;
            }
            
            if (robotHomeData.State != SpatialAnchorState.Ready) return;

            _context.FullFocusCanvasUI.HideAll();
            _machine.changeState<RobotCommandState>();
        }
    }
}