using Data;
using Multiplayer.Runtime.Data;
using Multiplayer.Runtime.View;
using Prime31.StateKit;
using UnityEngine;

namespace States
{
    public class WaitingForAnchorsState : SKState<AppData>
    {
        private MultiplayerData _data;

        public override void begin()
        {
            Debug.Log("ChangeState::WaitingForAnchorsState");
            base.begin();
            _data = _context.MultiplayerData;
            if (_data.RobotHomeAnchor == null)
            {
                _context.FullFocusCanvasUI.ShowWaitingForRobotHomeView();
            }
            else
            {
                CheckAnchorsLoaded();
            }
        }
        
        public override void update(float deltaTime)
        {
            if (_data.RobotHomeAnchor == null) return;
            CheckAnchorsLoaded();
        }

        private void CheckAnchorsLoaded()
        {
            var anchorsLoaded = CheckAnchorLoaded(_data.RobotHomeAnchor);
            if (anchorsLoaded)
            {
                foreach (var anchorEntry in _data.WorldGraphAnchors)
                {
                    var isAnchorLoaded = CheckAnchorLoaded(anchorEntry.Value);
                    if (!isAnchorLoaded) anchorsLoaded = false;
                }
            }
            if (!anchorsLoaded) return;
            InitialiseWorldGraphPanels();
            _machine.changeState<RobotCommandState>();
        }

        private void InitialiseWorldGraphPanels()
        {
            foreach (var anchorEntry in _data.WorldGraphAnchors)
            {
                var anchorData = anchorEntry.Value;
                _context.WorldGraphVisualsSystem.AnchorPanelToView(anchorData.transform, anchorData.Data.Name);
            }
        }

        private bool CheckAnchorLoaded(SpatialAnchorView anchor)
        {
            if (anchor == null) return false;
            var anchorData = anchor.Data;
            // if (anchorData.State == SpatialAnchorState.FatalError)
            // {
            //     // @TODO not fatal, but keeping for dev purposes for now
            //     Debug.LogError("Error loading anchor"); 
            //     _context.FatalError = "Error loading anchor";
            //     _machine.changeState<FatalErrorState>();
            //     return false;
            // }
            
            
            return anchorData.State == SpatialAnchorState.Ready;
        }
    }
}