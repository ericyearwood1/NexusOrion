using System.Collections.Generic;
using System.Linq;
using Data;
using Meta.XR.MRUtilityKit;
using Multiplayer.Runtime.Data;
using Multiplayer.Runtime.View;
using Prime31.StateKit;
using UIComponents.Runtime;
using UnityEngine;
using View;

namespace States
{
    public class ConfigureWorldGraphPanelsState : SKState<AppData>
    {
        private LabelFilter _raycastFilter;
        private int _currentDebugPoseIndex;
        private List<DebugPose> _debugPoses = new();
        private List<string> _worldGraphRoomList;
        private ConfigureWorldGraphPanelsView _view;
        private int _currentPanelEditIndex;
        private RoomEditData _activeData;
        private List<RoomEditData> _processingSaves = new();
        private PinchRecogniser _leftHandPinch = new PinchRecogniser();
        private PinchRecogniser _rightHandPinch = new PinchRecogniser();

        private Dictionary<string, RoomEditData> _editData = new();

        private struct DebugPose
        {
            public Vector3 Position;
            public Vector3 Normal;
        }

        private class RoomEditData
        {
            public SpatialAnchorView Anchor;
            public WorldGraphPanel Panel;
            public GameObject PositionIndicator;
            public bool IsNewAnchorRequested;
            public bool IsInitialised;
            public string Room;

            public void Hide()
            {
                Panel.Hide();
                PositionIndicator.SetActive(false);
            }

            public void Show()
            {
                Panel.Show();
                PositionIndicator.SetActive(true);
                Debug.Log($"ConfigureWorldGraphPanelsState::Show panel: {Room}");
            }

            public void SetNewEditPosition()
            {
                var transform = Panel.transform;
                transform.parent = null;
                transform.position = PositionIndicator.transform.position;
                transform.forward = PositionIndicator.transform.forward;
                IsNewAnchorRequested = true;
                IsInitialised = true;
            }
        }

        private enum State
        {
            None,
            LoadExistingAnchors,
            EditingRoom,
            Saving,
            NextState,
            Complete,
            Error
        }

        private State _currentState;

        public override void begin()
        {
            Debug.Log("ConfigureWorldGraphPanelsState.begin");
            _leftHandPinch.Initialise(_context.LeftHand);
            _rightHandPinch.Initialise(_context.RightHand);
            _raycastFilter = LabelFilter.FromEnum(MRUKAnchor.SceneLabels.WALL_FACE);
            InitialiseDebugPositions();
            InitialiseView();
            _currentState = State.LoadExistingAnchors;
            _view.ShowInitialisingDataView();
        }

        private void InitialiseDebugPositions()
        {
            if (Application.isEditor)
            {
                _debugPoses = new List<DebugPose>
                {
                    new() { Position = new Vector3(-2, 1, 0), Normal = new Vector3(1, 0, 0) },
                    new() { Position = new Vector3(2, 1, 0), Normal = new Vector3(-1, 0, 0) },
                    new() { Position = new Vector3(0, 1, 2), Normal = new Vector3(0, 0, -1) },
                    new() { Position = new Vector3(0, 1, -2), Normal = new Vector3(0, 0, 1) }
                };
            }
        }

        private void InitialiseView()
        {
            _view = _context.FullFocusCanvasUI.ShowSetUpWorldGraphPanelsView();
            _view.OnOverrideAnchorsRequested += OnOverrideAnchorsRequested;
            _view.OnReuseAnchorsRequested += OnReuseAnchorsRequested;
            _view.OnNextPanelRequest += OnNextPanelRequested;
            _view.OnSaveRequest += OnSaveRequested;
            _view.OnPreviousPanelRequest += OnPreviousPanelRequested;
        }

        private void OnSaveRequested()
        {
            _view.ShowSaveView();
            foreach (var roomEditData in _editData)
            {
                var data = roomEditData.Value;
                if (!data.IsNewAnchorRequested) continue;
                var panel = data.Panel.transform;
                data.Anchor = _context.SpatialAnchorService.CreateAnchor(roomEditData.Key,
                    SpatialAnchorType.WorldGraph,
                    panel.position, panel.rotation, data.Room);
                _context.MultiplayerData.WorldGraphAnchors[roomEditData.Key] = data.Anchor;
                _processingSaves.Add(data);
            }

            _currentState = State.Saving;
        }

        private void OnReuseAnchorsRequested()
        {
            _currentState = State.NextState;
        }

        private bool CheckAnchorsReady()
        {
            var isComplete = true;
            foreach (var room in _worldGraphRoomList)
            {
                if (!_context.MultiplayerData.WorldGraphAnchors.TryGetValue(room, out var anchor)) continue;
                if (anchor.Data.State != SpatialAnchorState.Ready)
                {
                    return false;
                }
            }
            return true;
        }

        private void GoToNextState()
        {
            foreach (var activePanel in _editData)
            {
                activePanel.Value.Panel.Hide();
            }
            _machine.changeState<CreateRobotHomeAnchorState>();
        }

        private void OnNextPanelRequested()
        {
            ShowEditPanelForRoomIndex(_currentPanelEditIndex + 1);
        }

        private void OnPreviousPanelRequested()
        {
            ShowEditPanelForRoomIndex(_currentPanelEditIndex - 1);
        }

        private void OnOverrideAnchorsRequested()
        {
            foreach (var activePanel in _editData)
            {
                activePanel.Value.Panel.Hide();
            }
            InitialiseEditState();
        }

        private void InitialiseEditData()
        {
            _currentState = State.EditingRoom;
            var roomList = _context.WorldGraphVisualsSystem.RoomList;
            if (roomList.Count == 0)
            {
                ShowError(
                    "World graph room list is empty. Has the robot been walked around the demo area? Please retry once the world graph has been created.");
                return;
            }

            _worldGraphRoomList = roomList.ToList();
            var numberRoomsInitialised = 0;
            foreach (var room in _worldGraphRoomList)
            {
                Debug.Log($"New room : {room}");
                var positionIndicator = new GameObject("Pointer");
                GameObject.Instantiate(_context.PinchMarker, positionIndicator.transform, true);
                positionIndicator.SetActive(false);
                var editData = new RoomEditData
                {
                    Room = room,
                    Anchor = _context.MultiplayerData.GetWorldGraphAnchorForRoom(room),
                    PositionIndicator = positionIndicator,
                };
                var parent = editData.Anchor == null ? editData.PositionIndicator.transform : editData.Anchor.transform;
                editData.Panel = _context.WorldGraphVisualsSystem.AnchorPanelToView(parent, room);
                editData.IsInitialised = editData.Anchor != null;
                if (editData.IsInitialised) numberRoomsInitialised++;
                _editData.Add(room, editData);
            }

            if (numberRoomsInitialised == _worldGraphRoomList.Count)
            {
                foreach (var activePanel in _editData)
                {
                    activePanel.Value.Panel.Show();
                }

                _view.ShowOverrideView();
            }
            else
            {
                InitialiseEditState();
            }
        }


        private bool IsSaveAllowed()
        {
            foreach (var roomEditData in _editData)
            {
                var editData = roomEditData.Value;
                if (!editData.IsInitialised && !editData.IsNewAnchorRequested) return false;
            }

            return true;
        }

        private void InitialiseEditState()
        {
            if (Application.isEditor)
            {
                ShowEditPanelForRoomIndex(0);
                return;
            }

            if (_context.MRUK == null)
            {
                ShowError("MRUK Instance is null");
                return;
            }
            _view.ShowGeneralMessage("Initialising panel editing: loading OVR rooms.");
            _context.MRUK.gameObject.SetActive(true);
            _context.MRUK.RegisterSceneLoadedCallback(OnSceneLoaded);
        }

        private void OnSceneLoaded()
        {
            Debug.Log($"ConfigureWorldGraphPanelsState::OnSceneLoaded");
            var rooms = _context.MRUK.Rooms;
            if (rooms.Count == 0)
            {
                ShowError(
                    "Room list is empty. Please exit the app and go to Settings > Physical Space > Space Setup and set up a space for each room used by the robot.");
                return;
            }

            ShowEditPanelForRoomIndex(0);
        }

        private void ShowEditPanelForRoomIndex(int index)
        {
            Debug.Log($"WorldGraphPanelFix::ShowEditPanelForRoomIndex1 {index}");
            _currentState = State.EditingRoom;
            _activeData?.Hide();
            _currentPanelEditIndex = index;
            var currentRoomName = _worldGraphRoomList[_currentPanelEditIndex];
            Debug.Log($"WorldGraphPanelFix::ShowEditPanelForRoomIndex2 {currentRoomName}");
            _activeData = _editData[currentRoomName];
            _activeData.Show();

            _view.ShowEditRoomPanel(_currentPanelEditIndex, _worldGraphRoomList.Count, currentRoomName,
                _activeData.IsInitialised);
            
            if (IsSaveAllowed())
            {
                _view.EnableSaveButton();
            }
        }
        
        public override void update(float deltaTime)
        {
            _leftHandPinch.Tick();
            _rightHandPinch.Tick();
            if (_view != null)
            {
                _view.Tick();
            }

            switch (_currentState)
            {
                case State.LoadExistingAnchors :
                    CheckExistingAnchorsLoaded();
                    break;
                case State.Saving:
                    CheckForSaveComplete();
                    break;
                case State.EditingRoom:
                    CheckForNewEdit();
                    break;
                case State.NextState when !CheckAnchorsReady():
                    return;
                case State.NextState:
                    _currentState = State.Complete;
                    GoToNextState();
                    break;
            }
        }

        private void CheckExistingAnchorsLoaded()
        {
            var multiplayerData = _context.MultiplayerData;
            var existingAnchorData = multiplayerData.Room.WorldGraphAnchors;
            var loadedAnchors = multiplayerData.WorldGraphAnchors;
            if(existingAnchorData.Count != loadedAnchors.Count) return;
            foreach (var loadedAnchor in loadedAnchors)
            {
                var anchorData = loadedAnchor.Value.Data;
                if (loadedAnchor.Value.Data.State != SpatialAnchorState.Ready) return;
            }

            if (multiplayerData.ThisUser.IsHost())
            {
                InitialiseEditData();
            }
            else
            {
                GoToNextState();
            }
        }

        private void CheckForNewEdit()
        {
            if (Application.isEditor)
            {
                if (Input.GetKeyDown(KeyCode.Space))
                {
                    var pose = _debugPoses[_currentDebugPoseIndex];
                    _activeData.PositionIndicator.transform.position = pose.Position;
                    _activeData.PositionIndicator.transform.forward = pose.Normal;
                    _activeData.SetNewEditPosition();
                    _currentDebugPoseIndex++;
                    _currentDebugPoseIndex %= _debugPoses.Count;
                    if(_currentPanelEditIndex < _worldGraphRoomList.Count - 1) OnNextPanelRequested();
                }
            }
            else
            {
                if (_context.MRUK == null) return;
                var ray = GetControllerRay();
                var currentRoom = _context.MRUK.GetCurrentRoom();
                if(currentRoom == null) return;
                currentRoom.Raycast(ray, Mathf.Infinity, _raycastFilter, out var hit);
                ShowHitNormal(hit);
                if (IsPlacePanel()) 
                {
                    _activeData.SetNewEditPosition();
                    _view.EnableNavigation();
                }
            }

            if (IsSaveAllowed())
            {
                _view.EnableSaveButton();
            }
        }

        private bool IsPlacePanel()
        {
            return _leftHandPinch.IsGesture;
        }

        private void CheckForSaveComplete()
        {
            var isComplete = true;
            var isAllSaved = true;
            var isErrors = false;
            foreach (var editData in _processingSaves)
            {
                if (editData.Anchor.Data.State != SpatialAnchorState.Error &&
                    editData.Anchor.Data.State != SpatialAnchorState.Ready)
                {
                    isComplete = false;
                }

                if (editData.Anchor.Data.State == SpatialAnchorState.Error)
                {
                    isErrors = true;
                    editData.Anchor = null;
                    editData.IsInitialised = false;
                    editData.Panel.transform.parent = editData.PositionIndicator.transform.parent;
                }
                else if (editData.Anchor.Data.State != SpatialAnchorState.Ready)
                {
                    isAllSaved = false;
                }
            }

            if (isComplete)
            {
                if (isAllSaved)
                {
                    _currentState = State.NextState;
                }
                else if (isErrors)
                {
                    ShowEditPanelForRoomIndex(0);
                }
            }
        }

        private void ShowError(string errorMessage)
        {
            _currentState = State.Error;
            _view.ShowErrorState(errorMessage);
        }

        private void ShowHitNormal(RaycastHit hit)
        {
            if (_activeData == null) return;
            if (hit.point != Vector3.zero && hit.distance != 0)
            {
                _activeData.PositionIndicator.SetActive(true);
                _activeData.PositionIndicator.transform.position = hit.point;
                _activeData.PositionIndicator.transform.forward = hit.normal;
            }
            else
            {
                _activeData.PositionIndicator.SetActive(false);
            }
        }
        
        private Ray GetControllerRay()
        {
            var Origin = _context.RightHandPointerPose.position;
            var Rotation = _context.RightHandPointerPose.rotation;
            var Forward = Rotation * Vector3.forward;
            return new Ray(Origin, Forward);
        }

        public override void end()
        {
            base.end();
            if (_editData != null)
            {
                foreach (var roomEditData in _editData)
                {
                    var data = roomEditData.Value;
                    if (data.PositionIndicator != null) GameObject.Destroy(data.PositionIndicator);
                }

                _editData.Clear();
            }

            if (_view != null)
            {
                _view.OnOverrideAnchorsRequested -= OnOverrideAnchorsRequested;
                _view.OnReuseAnchorsRequested -= OnReuseAnchorsRequested;
                _view.OnNextPanelRequest -= OnNextPanelRequested;
                _view.OnSaveRequest -= OnSaveRequested;
                _view.OnPreviousPanelRequest -= OnPreviousPanelRequested;
            }

            _editData = null;
            _activeData = null;
        }
    }
}