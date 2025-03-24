using System;
using System.Collections.Generic;
using Data;
using Multiplayer.Runtime.Data;
using Prime31.StateKit;
using UnityEngine;
using View;

namespace States
{
    public class InitialisingRoomState : SKState<AppData>
    {
        private MultiplayerData _data;
        private CoLocationAnchorSetupView _colocationView;

        public override void begin()
        {
            Debug.Log("ChangeState::InitialisingRoomState");
            base.begin();
            _data = _context.MultiplayerData;
            ProcessActiveFeatures();
            _colocationView = _context.FullFocusCanvasUI.ShowCoLocationView();
            if (!_context.IsUseSpatialAnchors && _data.IsThisUserAHost())
            {
                _context.SpatialAnchorService.CreateColocationAnchor();
                _colocationView.ShowCreatingAnchorView();
            }
            else
            {
                ProcessRoomAnchors();
            }
        }

        public override void end()
        {
            base.end();
            if (_data == null || _context.SpatialAnchorService == null) return;
            
        }

        

        private void ProcessActiveFeatures()
        {
            _context.FeatureActivationHandler.HandleActiveFeatureList(_data.Room.ActiveFeatures);
            var isCGActive = _context.FeatureActivationHandler.IsFeatureActive(AppFeatures.WORLD_VISUALISATION);
            var isManualSTTActive = _context.FeatureActivationHandler.IsFeatureActive(AppFeatures.MANUAL_STT_INSTRUCTION_PANEL);
            _context.LeftHandUI.SetFeatureButtons(isCGActive, isManualSTTActive);
        }

        public override void update(float deltaTime)
        {
            if (_data.Room == null) return;
            if (_data.CoLocationAnchor == null) return;

            if (_data.CoLocationAnchor.Data.State == SpatialAnchorState.Error)
            {
                return;
            }

            if (_data.CoLocationAnchor.Data.State != SpatialAnchorState.Ready) return;
            Debug.Log($"_data.Room.RobotHomeAnchor {_data.Room.RobotHomeAnchor} | {_data.RobotHomeAnchor == null}");
            var serverRobotData = _data.Room.RobotHomeAnchor;
            if (serverRobotData != null)
            {
                var spatialAnchor = _data.RobotHomeAnchor;
                if (spatialAnchor == null) return;
                if (spatialAnchor.Data.State != SpatialAnchorState.Ready) return;
            }

            Debug.Log(
                $"_data.Room.WorldGraphAnchors {_data.WorldGraphAnchors.Count} | {_data.Room.WorldGraphAnchors.Count}");
            if (_data.WorldGraphAnchors.Count != _data.Room.WorldGraphAnchors.Count) return;
            _context.FullFocusCanvasUI.HideAll();
            _machine.changeState<LoadWorldGraphState>();
        }

        #region Anchors

        private void ProcessRoomAnchors()
        {
            var roomData = _data.Room;
            _context.SpatialAnchorService.RemoveDeadAnchorsFromSave(roomData);
            
            // see which anchors are in the room. remove any anchors from the save data that aren't in the room
            
            if (_data.IsThisUserAHost())
            {
                var colocationAnchor = roomData.ColocationAnchor;
                if (colocationAnchor == null)
                {
                    Debug.Log("InitialisingRoomState::No anchors in room. Creating co-location anchor");
                    _context.SpatialAnchorService.CreateColocationAnchor();
                    _colocationView.ShowCreatingAnchorView();
                }
                else
                {
                    if (colocationAnchor.CreatedBy == _data.ThisUser.Id)
                    {
                        _colocationView.OnOverrideAnchors += OnOverrideAnchors;
                        _colocationView.OnUseExistingAnchors += OnUseExistingAnchors;
                        _colocationView.ShowOverrideView();
                    }
                    else
                    {
                        
                        var anchorOwner = colocationAnchor.CreatedBy;
                        var isAnchorOwnerOnline = _data.OtherUsers.ContainsKey(anchorOwner);
                        
                        if (isAnchorOwnerOnline)
                        {
                            var owner = _data.OtherUsers[anchorOwner];
                            if (owner.IsHost())
                            {
                                // @TODO running out of time to deal with this nicely. If time in future, 
                                // take user back to User details panel
                                _context.FatalError = "There is another Host currently online. Please restart as guest/observer.";
                                _machine.changeState<FatalErrorState>();
                            }
                            else
                            {
                                // force override if another host has created anchors
                                OnOverrideAnchors();
                            }
                            
                        }
                        else
                        {
                            Debug.Log("InitialisingRoomState::Existing anchor, but host offline. Attempting override");
                            OnOverrideAnchors();
                        }
                    }
                }
            }
            else
            {
                Debug.Log("InitialisingRoomState::");
                _colocationView.ShowLoadingAnchorsView();
                LoadRoomAnchors();
            }
        }

        private void LoadRoomAnchors()
        {
            Debug.Log("LoadRoomAnchors");
            _context.SpatialAnchorService.LoadAnchors(new List<SpatialAnchorData>(_data.GetAvailableAnchors()));
        }

        private void OnOverrideAnchors()
        {
            Debug.Log("OnOverrideAnchors");
            _colocationView.OnOverrideAnchors -= OnOverrideAnchors;
            _colocationView.OnUseExistingAnchors -= OnUseExistingAnchors;
            _data.Room.ColocationAnchor = null;
            _data.Room.RobotHomeAnchor = null;
            _data.Room.WorldGraphAnchors.Clear();
            _colocationView.ShowCreatingAnchorView();
            _context.SpatialAnchorService.CreateColocationAnchor();
        }

        private void OnUseExistingAnchors()
        {
            Debug.Log("OnUseExistingAnchors");
            _colocationView.OnOverrideAnchors -= OnOverrideAnchors;
            _colocationView.OnUseExistingAnchors -= OnUseExistingAnchors;
            _colocationView.ShowLoadingAnchorsView();
            LoadRoomAnchors();
        }
        

        #endregion
    }
}