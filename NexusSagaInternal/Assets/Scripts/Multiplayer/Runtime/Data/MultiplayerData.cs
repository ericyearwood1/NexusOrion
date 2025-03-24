using System;
using System.Collections.Generic;
using Multiplayer.Runtime.View;
using UnityEngine;

namespace Multiplayer.Runtime.Data
{
    public class MultiplayerData
    {
        public readonly Dictionary<ulong, User> OtherUsers = new();
        public readonly List<ulong> OtherUserIds = new();
        public readonly Dictionary<string, UserNameDisplay> UserNameDisplays = new();
        public readonly Dictionary<string, UserEntityView> SyncedEntities = new();
        public readonly Dictionary<Guid, SpatialAnchorView> ReadyAnchors = new();
        public readonly Dictionary<string, SpatialAnchorView> WorldGraphAnchors = new();
        public readonly HashSet<Guid> PendingAnchors = new();
        public readonly List<SpatialAnchorView> LoadedAnchorsStillProcessing = new();
        public ColourOption[] ColourOptions;
        public SyncedEntityView HeadEntityView;
        public SpatialAnchorView RobotHomeAnchor;
        public SpatialAnchorView CoLocationAnchor;
        public User ThisUser;
        public Room Room;
        public string[] RoomList;
        public bool IsConnected;
        public bool IsUseSpatialAnchors;
        public SagaPluginConfiguration SagaPluginConfiguration;

        public bool IsThisUser(User user)
        {
            if (user == null) return false;
            return ThisUser?.Id == user.Id;
        }
        
        public bool IsThisUser(ulong userId)
        {
            if (userId == 0) return false;
            return ThisUser?.Id == userId;
        }

        public bool DoesUserExist(ulong userId)
        {
            return OtherUsers.ContainsKey(userId);
        }

        public bool IsRoomListValid()
        {
            return RoomList != null && RoomList.Length > 0;
        }

        public bool IsSendSyncedUpdates()
        {
            return IsConnected && Room != null && HeadEntityView != null && CoLocationAnchor != null &&
                   CoLocationAnchor.Data?.State == SpatialAnchorState.Ready;
        }

        public void Reset()
        {
            RoomList = Array.Empty<string>();
            Room = null;
            SyncedEntities.Clear();
            UserNameDisplays.Clear();
            OtherUsers.Clear();
            OtherUserIds.Clear();
        }

        public void RemoveUser(ulong userId)
        {
            OtherUsers.Remove(userId);
            OtherUserIds.Remove(userId);
        }

        public void AddUser(User user)
        {
            OtherUsers.Add(user.Id, user);
            OtherUserIds.Add(user.Id);
        }

        public List<SpatialAnchorData> GetAvailableAnchors()
        {
            var anchors = new List<SpatialAnchorData>();
            if (Room.ColocationAnchor != null) anchors.Add(Room.ColocationAnchor);
            if (Room.RobotHomeAnchor != null) anchors.Add(Room.RobotHomeAnchor);
            foreach (var worldGraphAnchor in Room.WorldGraphAnchors)
            {
                Debug.Log($"GetAvailableAnchors:: {worldGraphAnchor.Name}");
                anchors.Add(worldGraphAnchor);
            }
            Debug.Log($"GetAvailableAnchors:: {anchors.Count}");
            return anchors;
        }

        public bool IsThisUserAHost()
        {
            return ThisUser.UserType == UserType.Host;
        }

        public SpatialAnchorView GetWorldGraphAnchorForRoom(string room)
        {
            return WorldGraphAnchors.TryGetValue(room, out var anchorView) ? anchorView : null;
        }
    }
}