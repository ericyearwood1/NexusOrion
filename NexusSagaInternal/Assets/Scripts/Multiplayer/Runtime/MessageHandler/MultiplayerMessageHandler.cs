using System;
using System.Collections.Generic;
using Multiplayer.Runtime.Data;
using Multiplayer.Runtime.Messages.Client;
using Multiplayer.Runtime.Messages.RelayMessage;
using Multiplayer.Runtime.Messages.Server;
using Multiplayer.Runtime.Pools;
using Multiplayer.Runtime.View;
using Newtonsoft.Json;
using Notifications.Runtime.Data;
using Notifications.Runtime.Systems;
using Robot.Runtime.Data;
using Robot.Runtime.Messages.Client;
using SiroComms.Runtime.Messages.Server;
using SiroComms.Runtime.Services;
using UnityEngine;

namespace Multiplayer.Runtime.Services
{
    public class MultiplayerMessageHandler : BaseMessageHandler
    {
        public const string Type = "multiplayer";

        private readonly MultiplayerData _data;
        private readonly MultiplayerPools _pools;

        private const string USER_JOIN = "user_join";
        private const string USER_LEFT = "user_left";
        private const string USER_DETAILS_UPDATE = "user_details_update";

        public Action<string> OnDeactivateFeature;
        public Action<string> OnActivateFeature;
        public Action<SpatialAnchorData> OnSpatialAnchorUpdate;
        public Action<InitialStateMessage> OnInitialStateReceived;
        public Action<UserDetailsMessage> OnUserUpdateReceived;
       
        public Action OnRoomStateReceived;
        public Action<User> OnUserJoined;
        public Action<ulong> OnUserLeft;
        private readonly SpatialAnchorService _spatialAnchorService;
        private readonly NotificationSystem _notificationSystem;
        private List<UserNameDisplay> _animatingOut = new();
        private readonly Camera _mainCamera;

        public MultiplayerMessageHandler(MultiplayerData data, MultiplayerPools pools,
            SpatialAnchorService spatialAnchorService, NotificationSystem notificationSystem, Camera mainCamera)
        {
            _data = data;
            _pools = pools;
            _spatialAnchorService = spatialAnchorService;
            _notificationSystem = notificationSystem;
            _mainCamera = mainCamera;
        }

        public override void HandleMessage(string messageType, string message)
        {
            switch (messageType)
            {
                case USER_JOIN:
                    ProcessUserJoinedMessage(message);
                    break;
                case USER_LEFT:
                    ProcessUserLeftMessage(message);
                    break;
                case USER_DETAILS_UPDATE:
                    ProcessUserDetailsChangedMessage(message);
                    break;
                case InitialStateMessage.Type:
                    ProcessMessage(message, OnInitialStateReceived);
                    break;
                case RoomStateMessage.Type:
                    ProcessRoomState(message);
                    break;
                case SyncedEntityMessage.Type:
                    // ProcessSyncedEntityMessage(message);
                    Debug.Log("SyncedEntity Skipped");
                    break;
                case SpatialAnchorMessage.Type:
                    ProcessSpatialAnchorMessage(message);
                    break;
                case ActivateFeatureMessage.Type:
                    HandleActivateFeatureMessage(message);
                    break;
                case DeactivateFeatureMessage.Type:
                    HandleDeactivateFeatureMessage(message);
                    break;
            }
        }

        private void HandleActivateFeatureMessage(string message)
        {
            // var data = JsonConvert.DeserializeObject<ActivateFeatureMessage>(message);
            var featureDTO = ServerMessage<FeatureUpdateDTO>.Deserialize(message).Data;
            OnActivateFeature?.Invoke(featureDTO.Feature);
        }
        
        private void HandleDeactivateFeatureMessage(string message)
        {
            // var data = JsonConvert.DeserializeObject<DeactivateFeatureMessage>(message);
            var featureDTO = ServerMessage<FeatureUpdateDTO>.Deserialize(message).Data;
            OnDeactivateFeature?.Invoke(featureDTO.Feature);
        }

        // Method to Hide username Names
        private void HideUserNames()
        {
            // Iterate over all the username values
            foreach (var userNameDisplay in _data.UserNameDisplays.Values)
            {
                // Hide or deactivate the username display
                userNameDisplay.Hide(); 
            }
        }

        // Method to Show username Names
        private void DisplayUserNames()
        {
             // Iterate over all the username values
            foreach (var userNameDisplay in _data.UserNameDisplays.Values)
            {
                // Show or activate the user name display
                userNameDisplay.Show();
            }

        }

        private void ProcessSpatialAnchorMessage(string message)
        {
            var spatialAnchorDTO = ServerMessage<SpatialAnchorUpdateDTO>.Deserialize(message).Data;
            // var spatialAnchorMessage = JsonConvert.DeserializeObject<SpatialAnchorMessage>(message);
            if (_data.Room == null) return;
            var anchorData = spatialAnchorDTO.Data;
            
            if (anchorData.CreatedBy == _data.ThisUser.Id) return;
            if (anchorData.AnchorType == SpatialAnchorType.CoLocation)
            {
                if (_data.Room.ColocationAnchor == null)
                {
                    _data.Room.ColocationAnchor = anchorData;
                    _data.Room.ColocationAnchor.State = SpatialAnchorState.NoStatus;
                }
                else
                {
                    CopyAnchorData(_data.Room.ColocationAnchor, anchorData);
                }
            }
            else if (anchorData.AnchorType == SpatialAnchorType.RobotHome)
            {
                if (_data.Room.RobotHomeAnchor == null)
                {
                    _data.Room.RobotHomeAnchor = anchorData;
                    _data.Room.RobotHomeAnchor.State = SpatialAnchorState.NoStatus;
                }
                else
                {
                    CopyAnchorData(_data.Room.RobotHomeAnchor, anchorData);
                }
                
            }
            else if (anchorData.AnchorType == SpatialAnchorType.WorldGraph)
            {
                var isEdit = false;
                for(var i = 0; i < _data.Room.WorldGraphAnchors.Count; ++i)
                {
                    var data = _data.Room.WorldGraphAnchors[i];
                    
                    if (data.Name == anchorData.Name)
                    {
                        isEdit = true;
                        CopyAnchorData(_data.Room.WorldGraphAnchors[i], anchorData);
                        break;
                    }
                }

                if (!isEdit)
                {
                    anchorData.State = SpatialAnchorState.NoStatus;
                    _data.Room.WorldGraphAnchors.Add(anchorData);
                }
            }

            ProcessAnchors(_data.Room, _data.ThisUser);
            OnSpatialAnchorUpdate?.Invoke(anchorData);
        }

        private void CopyAnchorData(SpatialAnchorData to, SpatialAnchorData from)
        {
            to.SharedWith = from.SharedWith;
        }

        private void ProcessRoomState(string message)
        {
            // var roomStateData = ServerMessage<RoomDTO>.Deserialize(message).Data;
            var roomStateData = JsonConvert.DeserializeObject<RoomStateMessage>(message);
            var thisUser = _data.ThisUser;
            var room = roomStateData.Room;
            _data.Room = room;
            thisUser.Room = _data.Room.Id;
            if (!_data.IsUseSpatialAnchors)
            {
                room.ColocationAnchor = null;
                room.RobotHomeAnchor = null;
            }
            foreach (var user in _data.Room.Users)
            {
                AddUser(user, false);
            }
            OnRoomStateReceived?.Invoke();
        }

        private void ProcessUserLeftMessage(string message)
        {
            var userLeftMessage = ServerMessage<UserLeftDTO>.Deserialize(message).Data;
            var userId = userLeftMessage.Id;
            if (_data.IsThisUser(userId)) return;
            if (_data.OtherUsers.TryGetValue(userId, out var user))
            {
                _data.RemoveUser(userId);
                OnUserLeft?.Invoke(userId);
                var userHeadEntityId = user.HeadEntityId;
                if (_data.SyncedEntities.TryGetValue(userHeadEntityId, out var head))
                {
                    _data.SyncedEntities.Remove(userHeadEntityId);
                    _pools.HeadPool.Return(head);
                }
                else
                {
                    Debug.LogError($"Attempting to add synced entity that already exists : {userHeadEntityId}");
                }

                if (_data.UserNameDisplays.TryGetValue(userHeadEntityId, out var display))
                {
                    display.Hide();
                    _animatingOut.Add(display);
                    _data.UserNameDisplays.Remove(userHeadEntityId);
                }
                else
                {
                    Debug.LogError($"Attempting to add synced entity that already exists : {userHeadEntityId}");
                }
                _notificationSystem.ShowNotification($"User left {user.DisplayName}", NotificationType.Info);
            }
            else
            {
                Debug.LogError($"Attempting to remove user that doesn't exist : {userId}");
            }
        }

        /**
         * Process another user joining the session
         */
        private void ProcessUserJoinedMessage(string message)
        {
            var userJoinedMessage = ServerMessage<UserMessage>.Deserialize(message).Data;
            // var userJoinedMessage = JsonConvert.DeserializeObject<UserMessage>(message);
            var user = userJoinedMessage.User;
            if (_data.IsThisUser(user)) return;
            AddUser(user, true);
            var thisUser = _data.ThisUser;
            _notificationSystem.ShowNotification($"User joined {user.DisplayName} | {_data.ReadyAnchors.Count}", NotificationType.Info);
            foreach (var anchorPair in _data.ReadyAnchors)
            {
                var anchor = anchorPair.Value;
                if (anchor.Data.CreatedBy != thisUser.Id) continue;
                if (anchor.IsSharedWithUser(user.Id)) continue;
                _spatialAnchorService.ShareAnchor(anchor);
            }
            
        }

        private void AddUser(User user, bool shouldTriggerEvent)
        {
            if (_data.IsThisUser(user)) return;
            if (_data.DoesUserExist(user.Id)) return;
            if (!_data.OtherUsers.ContainsKey(user.Id))
            {
                _data.AddUser(user);
                if (shouldTriggerEvent)
                {
                    OnUserJoined?.Invoke(user);
                }
            }
            else
            {
                Debug.LogError($"Attempting to add user that already exists : {user.Id}");
            }

            AddSyncedUserEntity(user);
        }

        private void AddSyncedUserEntity(User user)
        {
            var userHeadEntityId = user.HeadEntityId;
            if (!_data.SyncedEntities.ContainsKey(userHeadEntityId))
            {
                var head = _pools.HeadPool.Get();
                head.Initialise(userHeadEntityId);
                var userNameDisplay = _pools.UserDisplayPool.Get();
                userNameDisplay.UpdateDisplay(user.DisplayName, _data.ColourOptions[user.Color].Colour, _mainCamera.transform);
                userNameDisplay.TrackEntity(head);
                userNameDisplay.Hide();
                _data.UserNameDisplays.Add(userHeadEntityId, userNameDisplay);
                _data.SyncedEntities.Add(userHeadEntityId, head);
            }
            else
            {
                Debug.LogError($"Attempting to add synced entity that already exists : {userHeadEntityId}");
            }
        }

        private void ProcessUserDetailsChangedMessage(string message)
        {
            var userDetailsMessage = JsonConvert.DeserializeObject<UserMessage>(message);
            if (_data.IsThisUser(userDetailsMessage.User)) return;
            if (_data.OtherUsers.TryGetValue(userDetailsMessage.User.Id, out var user))
            {
                user.DisplayName = userDetailsMessage.User.DisplayName;
                user.Color = userDetailsMessage.User.Color;
                // @TODO get user display and change colour/ name
                // lerp to colour
                // fade the name out and in
                // user.UpdateDisplay();
            }
        }

        private void ProcessSyncedEntityMessage(string message)
        {
            if (_data.CoLocationAnchor == null) return;
            if (_data.CoLocationAnchor.Data.State != SpatialAnchorState.Ready) return; 
            // var serverData = JsonConvert.DeserializeObject<SyncedEntityMessage>(message);
            var syncedEntity = ServerMessage<SyncedEntityDTO>.Deserialize(message).Data;
            if (_data.ThisUser.IsUserEntity(syncedEntity.EntityId)) return;
            if (!_data.SyncedEntities.TryGetValue(syncedEntity.EntityId, out var entity)) return;
            entity.SetTargets(syncedEntity.Position, syncedEntity.Rotation, syncedEntity.Scale);
            if (!_data.UserNameDisplays.TryGetValue(syncedEntity.EntityId, out var display)) return;
            if (display.State is HighlightState.AnimatingIn or HighlightState.OnDisplay) return;
            var displayPosition = display.transform;
            displayPosition.localPosition = syncedEntity.Position;
            displayPosition.localRotation = syncedEntity.Rotation;
            display.Show(); //Comment to not show username when Sync Entity is called
        }

        private void ProcessAnchors(Room room, User thisUser)
        {
            var coLocationAnchor = room.ColocationAnchor;
            var robotHomeAnchor = room.RobotHomeAnchor;
            var anchorsToLoad = new List<SpatialAnchorData>();
            Debug.Log($"\nProcessAnchors {thisUser.IsHost()} | {room.WorldGraphAnchors.Count}");
            if (thisUser.IsHost())
            {
                ProcessHostAnchors(thisUser, coLocationAnchor, robotHomeAnchor, room.WorldGraphAnchors, anchorsToLoad);

            }
            else
            {
                CheckAnchorForGuest(thisUser, coLocationAnchor, anchorsToLoad);
                CheckAnchorForGuest(thisUser, robotHomeAnchor, anchorsToLoad);
                foreach (var anchor in room.WorldGraphAnchors)
                {
                    CheckAnchorForGuest(thisUser, anchor, anchorsToLoad);
                }
            }

            _spatialAnchorService.LoadAnchors(anchorsToLoad);
        }

        private void ProcessHostAnchors(User thisUser, SpatialAnchorData coLocationAnchor, SpatialAnchorData robotHomeAnchor, List<SpatialAnchorData> worldGraphAnchors,
            List<SpatialAnchorData> anchorsToLoad)
        {
            Debug.Log($"\nProcessHostAnchors {worldGraphAnchors.Count}");
            ProcessHostColocationAnchor(thisUser, coLocationAnchor, anchorsToLoad);
            ProcessHostAnchor(thisUser, robotHomeAnchor, anchorsToLoad);
            foreach (var anchor in worldGraphAnchors)
            {  
                ProcessHostAnchor(thisUser, anchor, anchorsToLoad);
            }
        }

        private void ProcessHostColocationAnchor(User user, SpatialAnchorData coLocationAnchor,
            List<SpatialAnchorData> anchorsToLoad)
        {
            // if host, try and create the anchors if they are missing
            if (coLocationAnchor == null)
            {
                Debug.Log("\nCreating new co-location anchor");
                _spatialAnchorService.CreateAnchor("Co-Location Anchor", SpatialAnchorType.CoLocation, Vector3.zero,
                    Quaternion.identity);
            }
            else if (coLocationAnchor.CreatedBy != user.Id)
            {
                Debug.Log("\nA co-location anchor already exists. Should we override it?");
                _spatialAnchorService.CreateAnchor("Co-Location Anchor", SpatialAnchorType.CoLocation, Vector3.zero,
                    Quaternion.identity);
            }
            else if (_data.ReadyAnchors.TryGetValue(coLocationAnchor.UUID, out var anchor))
            {
                Debug.Log($"\nFound matching anchor data. Processing. {anchor.Data.State}");
                // check anchor state and trigger save/share as needed
                if (anchor.Data.State == SpatialAnchorState.Ready)
                {
                    _spatialAnchorService.ShareAnchor(anchor);
                }
            }
            else
            {
                Debug.Log("\nHost loading existing anchor");
                anchorsToLoad.Add(coLocationAnchor);
            }
        }
        
        private void ProcessHostAnchor(User user, SpatialAnchorData anchorData,
            List<SpatialAnchorData> anchorsToLoad)
        {
            if (anchorData == null)
            {
                Debug.Log("\nHost needs to create robot home anchor at right time");
            }
            else if (anchorData.CreatedBy != user.Id)
            {
                Debug.Log(
                    $"\n<color=red>{anchorData.AnchorType} created by another host user. Should we override? Doing nothing for now</color>");
            }
            else if (_data.ReadyAnchors.TryGetValue(anchorData.UUID, out var anchor))
            {
                Debug.Log($"\nFound matching anchor data. Processing. {anchor.Data.State}");
                // check anchor state and trigger save/share as needed
                if (anchor.Data.State == SpatialAnchorState.Ready)
                {
                    _spatialAnchorService.ShareAnchor(anchor);
                }
            }
            else
            {
                
                anchorsToLoad.Add(anchorData);
                Debug.Log($"\nHost loading existing anchor {anchorData.AnchorType}");
            }
        }

        private void CheckAnchorForGuest(User user, SpatialAnchorData anchor, List<SpatialAnchorData> anchorsToLoad)
        {
            if (anchor == null)
            {
                Debug.Log($"\nWAITING FOR HOST to send anchor data");
            }
            else
            {
                Debug.Log($"CheckAnchorForGuest {anchor.Name} | {anchor.AnchorType} | {anchor.State} | {_data.ReadyAnchors.ContainsKey(anchor.UUID)} | {_data.PendingAnchors.Contains(anchor.UUID)} | {anchor.SharedWith.Contains(user.Id)} | {user.Id}");
                if (!_data.ReadyAnchors.ContainsKey(anchor.UUID))
                {
                    if (anchor.SharedWith.Contains(user.Id))
                    {
                        anchorsToLoad.Add(anchor);
                    }
                    else
                    {
                        Debug.Log($"\nWAITING FOR HOST to send share anchor {anchor.AnchorType}");
                    }
                }
                else
                {
                    // already loaded
                }
            }
        }

        public void Tick()
        {
            for (var i = _animatingOut.Count - 1; i >= 0; --i)
            {
                var display = _animatingOut[i];
                if (display.State is not (HighlightState.Hidden or HighlightState.None)) continue;
                _animatingOut.RemoveAt(i);
                _pools.UserDisplayPool.Return(display);
            }
        }
    }
}