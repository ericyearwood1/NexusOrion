using System.Collections.Generic;
using System.Threading.Tasks;
using Logger.Runtime;
using Multiplayer.Runtime.Data;
using UnityEngine;

namespace Multiplayer.Runtime.View
{
    public class OVRSpatialAnchorView : SpatialAnchorView
    {
        [SerializeField] private OVRSpatialAnchor _anchor;
        private int _extraTicks = 0;

        //@TODO get rid of this
        public override OVRSpatialAnchor GetOVRAnchor()
        {
            return _anchor;
        }
        
        public override bool IsLocalized()
        {
            return _anchor.Localized;
        }

        public override void Save()
        {
            SaveAnchor();
        }

        private async Task SaveAnchor()
        {
            var task = await _anchor.SaveAnchorAsync();
            if (task.Success)
            {
                _data.UUID = _anchor.Uuid;
                DevLog.Log($"Successfully saved anchor(s) {_anchor.Uuid}");
                OnSaveSuccess?.Invoke(this);
            }
            else
            {
                DevLog.LogError($"Error saving anchor(s) {_anchor.Uuid} | {task.Status}");
                _data.SaveError = task.Status == OVRAnchor.SaveResult.FailureInsufficientView
                    ? "OVRSpatialAnchor.OperationResult.Failure_SpaceMappingInsufficient"
                    : "OVRSpatialAnchor.OperationResult.Failure";
                OnSaveError?.Invoke(this);
            }
        }

        public override void Share(List<ulong> users)
        {
            DevLog.Log($"Share anchor {Data.AnchorType} | {users.Count}");
            if (users.Count == 0) return;
            var spaceUserList = new List<OVRSpaceUser>();
            foreach (var user in users)
            {
                if(_data.SharedWith.Contains(user)) continue;
                spaceUserList.Add(new OVRSpaceUser(user));
            }
            if (spaceUserList.Count == 0) return;
            ShareAnchor(spaceUserList);
        }

        public override bool IsLoaded()
        {
            if (!_anchor.Localized || _anchor.PendingCreation)
                return false;
            _extraTicks++;
            return _extraTicks >= 2; // seems to take one extra update
        }

        private async Task ShareAnchor(List<OVRSpaceUser> spaceUserList)
        {
            var anchors = new List<OVRSpatialAnchor> { _anchor };
            OVRSpatialAnchor.ShareAsync(anchors, spaceUserList).ContinueWith((result) =>
            {
                if (result.IsError())
                {
                    switch (result)
                    {
                        case OVRSpatialAnchor.OperationResult.Failure_SpaceNetworkRequestFailed:
                        {
                            // Unable to reach Meta servers.
                            // Instruct user to check internet connection
                            _data.ShareError = $"Failed to share anchors {result}. Check internet connection";
                            break;
                        }
                        case OVRSpatialAnchor.OperationResult.Failure_SpaceCloudStorageDisabled:
                        {
                            _data.ShareError = $"Failed to share anchors {result}. Turn on Share Point Cloud Data";
                            // inform user to turn on Share Point Cloud Data
                            // Settings > Privacy and Safety > Device Permissions > Turn on "Share point cloud data"
                            break;
                        }
                        default:
                            _data.ShareError = $"Failed to share anchors {result}";
                            break;
                    }
                    Debug.LogError($"Error sharing anchor {_data.ShareError}");
                    OnShareError?.Invoke(this);
                }
                else
                {
                    foreach (var user in spaceUserList)
                    {
                        DevLog.Log($"Adding to shared anchor list {user.Id}");
                        _data.SharedWith.Add(user.Id);
                    }

                    OnShareSuccess?.Invoke(this);
                }
            });
        }
    }
}