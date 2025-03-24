using Data;
using Multiplayer.Runtime.Data;
using Prime31.StateKit;
using UnityEngine;

namespace States
{
    public class WaitingForWorldGraphPanelsState : SKState<AppData>
    {
        private MultiplayerData _data;

        public override void begin()
        {
            _data = _context.MultiplayerData;
            Debug.Log($"WaitingForWorldGraphPanelsState::begin {_data.WorldGraphAnchors.Count}");
            if (_data.WorldGraphAnchors.Count == 0 ||
                _data.WorldGraphAnchors.Count != _data.Room.WorldGraphAnchors.Count)
            {
                _context.FullFocusCanvasUI.ShowWaitingForWorldGraphAnchorsView();
            }
        }

        public override void update(float deltaTime)
        {
            CheckForComplete();
        }

        private void CheckForComplete()
        {
            if (_data.WorldGraphAnchors.Count == 0) return;
            if (_data.WorldGraphAnchors.Count != _data.Room.WorldGraphAnchors.Count) return;
            foreach (var anchor in _data.WorldGraphAnchors)
            {
                var anchorView = anchor.Value;
                _context.WorldGraphVisualsSystem.AnchorPanelToView(anchorView.transform, anchorView.Data.Name);
            }

            _machine.changeState<RobotCommandState>();
        }
    }
}