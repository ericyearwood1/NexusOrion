using Data;
using Multiplayer.Runtime.Data;
using Prime31.StateKit;
using UnityEngine;
using View;

namespace States
{
    public class ThreeDotRobotSyncState : SKState<AppData>
    {
        private ThreeDotRobotSyncView _view;

        public override void begin()
        {
            base.begin();
            Debug.Log($"ThreeDotRobotSyncState");
            _view = _context.FullFocusCanvasUI.ShowSyncWithRobotView();
            _view.OnComplete += OnRobotSyncComplete;
        }

        public override void end()
        {
            base.end();
            if (_view == null) return;
            _view.OnComplete -= OnRobotSyncComplete;
            _view.Reset();
            _view.gameObject.SetActive(false);
        }

        private void OnRobotSyncComplete(Vector3 position, Quaternion rotation)
        {
            _view.OnComplete -= OnRobotSyncComplete;
            Debug.Log($"OnRobotSyncComplete:: {position} || {rotation}");
            _context.SpatialAnchorService.CreateAnchor("Robot Home Anchor", SpatialAnchorType.RobotHome, position,
                rotation);
            _machine.changeState<CreateRobotHomeAnchorState>();
        }

        public override void update(float deltaTime)
        {
        }
    }
}