using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Multiplayer.Runtime.Config;
using Multiplayer.Runtime.Data;
using Multiplayer.Runtime.View;
using Notifications.Runtime.Data;
using Notifications.Runtime.Systems;
using UnityEngine;
using UnityEngine.Assertions;

namespace Multiplayer.Runtime.Services
{
    public class SpatialAnchorService
    {
        private const int MaxLoadAttempts = 100;
        private const int MaxShareAttempts = 5;
        public Action<string> OnFatalAnchorLoadError;
        public Action<SpatialAnchorView> OnAnchorShareErrorEvent;
        public Action<SpatialAnchorView> OnAnchorSavedEvent;
        public Action<SpatialAnchorView> OnAnchorSaveErrorEvent;
        public Action<SpatialAnchorView> OnAnchorSharedEvent;
        public Action<SpatialAnchorView> OnAnchorLoadedEvent;
        public Action<List<Guid>> OnLoadAnchorErrorEvent;
        private NotificationSystem _notificationSystem;
        private SpatialAnchorConfig _config;
        private MultiplayerData _data;
        private Transform _playerTransform;
        private Transform _coLocationContainer;
        private Transform _robotHome;
        private Camera _mainCamera;
        private bool _isDisposed;
        private bool _isCoLocated;
        private bool _isShowNotifications = false;
        private int _loadRetries;
        private bool _isShowDebugView;
        private HashSet<Guid> _localAnchors;

        public void Initialise(SpatialAnchorConfig config, MultiplayerData data, Transform playerTransform,
            Transform robotHomeTransform, Transform coLocationContainer, NotificationSystem notificationSystem, Camera mainCamera, bool isShowDebugView)
        {
            _isShowDebugView = isShowDebugView;
            _config = config;
            _data = data;
            _playerTransform = playerTransform;
            _robotHome = robotHomeTransform;
            _coLocationContainer = coLocationContainer;
            _notificationSystem = notificationSystem;
            _mainCamera = mainCamera;
            LoadReadyAnchorIds();
        }

        private void LoadReadyAnchorIds()
        {
            _localAnchors = new HashSet<Guid>();
            var savedExistingAnchors = PlayerPrefs.GetString(Consts.PlayerPref_ReadyAnchors, "");
            var existingAnchors = savedExistingAnchors.Split(",");
            foreach (var anchorId in existingAnchors)
            {
                if (string.IsNullOrWhiteSpace(anchorId)) continue;
                _localAnchors.Add(Guid.Parse(anchorId));
            }
        }

        public SpatialAnchorView CreateAnchor(string anchorName, SpatialAnchorType anchorType, Vector3 position, Quaternion rotation, string passthroughData = null)
        {
            var userData = _data.ThisUser;
            if (userData.IsHost())
            {
                Debug.Log($"SpatialAnchorService::Creating anchor: {anchorName} | {anchorType} | {_data.Room.Id}");
                var data = new SpatialAnchorData
                {
                    Name = anchorName,
                    AnchorType = anchorType,
                    Room = _data.Room.Id,
                    CreatedBy = _data.ThisUser.Id,
                    Details = passthroughData,
                    State = SpatialAnchorState.Initialising
                };

                var view = !_config.IsOVRTesting
                    ? InstantiateAnchorAtPosition<StubSpatialAnchorView>(_config.AnchorPrefab, position, rotation)
                    : InstantiateAnchorAtPosition<OVRSpatialAnchorView>(_config.AnchorPrefab, position, rotation);
                view.Initialise(data, _isShowDebugView);
                //@TODO make this less rigid
                Debug.Log($"SpatialAnchorService::Anchor type : {anchorType}");
                if (data.AnchorType == SpatialAnchorType.CoLocation)
                {
                    _data.CoLocationAnchor = view;
                }
                else if (data.AnchorType == SpatialAnchorType.RobotHome)
                {
                    _data.RobotHomeAnchor = view;
                }
                else if (data.AnchorType == SpatialAnchorType.WorldGraph)
                {
                    _data.WorldGraphAnchors[passthroughData] = view;
                }
                InitialiseAnchor(view);
                return view;
            }
            Debug.LogError($"SpatialAnchorService::Only hosts can create spatial anchors | {SpatialAnchorType.CoLocation}");
            ShowNotification("Only hosts can create spatial anchors", NotificationType.Error);
            return null;
        }

        private async Task InitialiseAnchor(SpatialAnchorView anchor, Transform player = null)
        {
            await WaitingForAnchorLocalization(anchor);
            if (anchor.Data.AnchorType == SpatialAnchorType.CoLocation && player != null)
            {
                await AlignToAnchor(anchor, player);
            }

            SaveAnchor(anchor);
        }

        private async Task WaitingForAnchorLocalization(SpatialAnchorView anchor)
        {
            while (!_isDisposed && !anchor.IsLocalized())
            {
                await Task.Yield();
            }

            Debug.Log($"SpatialAnchorService::WaitingForAnchorLocalization: Anchor Localized");
        }

        private async Task AlignToAnchor(SpatialAnchorView anchor, Transform player)
        {
            player.position = Vector3.zero;
            player.eulerAngles = Vector3.zero;

            await Task.Yield();

            var anchorTransform = anchor.transform;
            var positionOffset = anchorTransform.InverseTransformPoint(Vector3.zero);
            var rotationOffset = new Vector3(0, -anchorTransform.eulerAngles.y, 0);

            if (player)
            {
                player.position = positionOffset;
                player.eulerAngles = rotationOffset;
            }
        }

        private async void SaveAnchor(SpatialAnchorView anchor)
        {
            Debug.Log($"SpatialAnchorService:SaveAnchor");
            anchor.OnSaveSuccess = null;
            anchor.OnSaveError = null;
            anchor.OnSaveError += OnAnchorSaveError;
            anchor.OnSaveSuccess += OnAnchorSaved;
            anchor.Save();
        }

        private void OnAnchorSaveError(SpatialAnchorView anchor)
        {
            anchor.OnSaveSuccess -= OnAnchorSaved;
            anchor.OnSaveError -= OnAnchorSaveError;
            ProcessAnchorError(anchor.Data);
            ShowNotification($"Error saving anchor {anchor.Data.AnchorType} | {anchor.Data.SaveError}", NotificationType.Error);
            OnAnchorSaveErrorEvent?.Invoke(anchor);
        }

        private void OnAnchorSaved(SpatialAnchorView anchor)
        {
            Debug.Log($"SpatialAnchorService:OnAnchorSaved {anchor.Data.Name}");
            anchor.OnSaveSuccess -= OnAnchorSaved;
            anchor.OnSaveError -= OnAnchorSaveError;
            anchor.Data.State = SpatialAnchorState.Ready;
            UpdateLocalAnchors(anchor.Data.UUID);
            if (anchor.Data.AnchorType == SpatialAnchorType.RobotHome)
            {
                AlignTransformToAnchor(_robotHome, anchor);
            }
            else if (anchor.Data.AnchorType == SpatialAnchorType.CoLocation)
            {
                AlignTransformToAnchor(_coLocationContainer, anchor);
            }

            _data.ReadyAnchors.Add(anchor.Data.UUID, anchor);
            OnAnchorSavedEvent?.Invoke(anchor);
            Share(anchor);
        }

        private void UpdateLocalAnchors(Guid uuid)
        {
            _localAnchors.Add(uuid);
            PlayerPrefs.SetString(Consts.PlayerPref_ReadyAnchors, string.Join(",", _localAnchors));
        }

        public void ShareAnchor(SpatialAnchorView anchor)
        {
            Debug.Log($"SpatialAnchorService::ShareAnchor {anchor.Data.AnchorType}");
            Share(anchor);
        }

        private void OnShareError(SpatialAnchorView anchor)
        {
            anchor.OnShareError -= OnShareError;
            anchor.OnShareSuccess -= OnAnchorShared;
            anchor.ShareRetries += 1;
            Debug.LogError($"SpatialAnchorService::Failed to share anchor {anchor.Data.Name} | {anchor.Data.UUID}");
            ProcessAnchorError(anchor.Data);
            ShowNotification(anchor.Data.ShareError, NotificationType.Error);
            if (anchor.ShareRetries < MaxShareAttempts)
            {
                Share(anchor);
            }
            else
            {
                _notificationSystem.ShowNotification($"Could not share anchor {anchor.Data.Name}.", NotificationType.Error);
                OnAnchorShareErrorEvent?.Invoke(anchor);
            }
        }

        private void ProcessAnchorError(SpatialAnchorData data)
        {
            data.State = SpatialAnchorState.Error;
            _data.PendingAnchors.Remove(data.UUID);
        }

        private void OnAnchorShared(SpatialAnchorView anchor)
        {
            Debug.Log($"SpatialAnchorService::OnAnchorShared {anchor.Data.AnchorType} | {anchor.Data.Name}");
            anchor.OnShareError -= OnShareError;
            anchor.OnShareSuccess -= OnAnchorShared;
            ShowNotification($"Anchor shared {anchor.Data.AnchorType} | {anchor.Data.SharedWith.Count}");
            OnAnchorSharedEvent?.Invoke(anchor);
        }

        public void LoadAnchors(List<SpatialAnchorData> anchors)
        {
            ShowNotification($"LoadAnchors {anchors.Count}");
            Debug.Log($"SpatialAnchorService::LoadAnchors {anchors.Count}");
            for (var i = anchors.Count - 1; i >= 0; --i)
            {
                var anchor = anchors[i];
                Debug.Log($"SpatialAnchorService::LoadAnchors {anchor.Name} | {anchor.AnchorType} | {anchor.Details} | {anchor.State}");
                
                if (_data.PendingAnchors.Contains(anchor.UUID) || _data.ReadyAnchors.ContainsKey(anchor.UUID) || anchor.State == SpatialAnchorState.Ready || anchor.State == SpatialAnchorState.Binding || anchor.State == SpatialAnchorState.Loading)
                {
                    Debug.Log($"SpatialAnchorService::Anchor load already in progress {anchor.UUID}");
                    anchors.RemoveAt(i);
                }
            }

            if (anchors.Count == 0)
            {
                Debug.Log("SpatialAnchorService::No anchors to load");
                return;
            }
           
            if (!_config.IsOVRTesting)
            {
                Debug.Log($"SpatialAnchorService::LoadingAnchors:!_config.IsOVRTesting: {anchors.Count}");
                OnEditorUnboundAnchorsLoaded(anchors);
            }

            else
            {
                Debug.Log($"SpatialAnchorService::LoadingAnchors {anchors.Count}");
                RetrieveLocalAnchors(anchors);
                RetrieveSharedAnchors(anchors);
            }
        }
        
        private void ShowNotification(string message, NotificationType notificationType = NotificationType.Info)
        {
            if (!_isShowNotifications || _notificationSystem == null) return; 
            _notificationSystem.ShowNotification(message, notificationType);
        }

        private async Task RetrieveLocalAnchors(List<SpatialAnchorData> anchorData)
        {
            var anchors = anchorData.Where(a => (a.CreatedBy == _data.ThisUser.Id || _localAnchors.Contains(a.UUID))).ToList();
            var anchorIds = anchors.Select(a => a.UUID).ToList();
            Debug.Log($"SpatialAnchorService::RetrieveLocalAnchors {anchorIds.Count}");
            if (anchorIds.Count == 0) return;
            Assert.IsTrue(anchorIds.Count <= OVRPlugin.SpaceFilterInfoIdsMaxSize,
                "SpaceFilterInfoIdsMaxSize exceeded.");
            foreach (var id in anchorIds)
            {
                foreach (var data in anchorData)
                {
                    if (data.UUID == id)
                    {
                        data.State = SpatialAnchorState.Loading;
                    }
                }
                _data.PendingAnchors.Add(id);
            }

            var unboundAnchors = new List<OVRSpatialAnchor.UnboundAnchor>();
            var result = await OVRSpatialAnchor.LoadUnboundAnchorsAsync(anchorIds, unboundAnchors);
            Debug.Log($"SpatialAnchorService::RetrieveLocalAnchors:LoadComplete : {result} | {unboundAnchors.Count}");
            if (result.Success)
            {
                if (unboundAnchors.Count == 0)
                {
                    Debug.LogError("SpatialAnchorService::RetrieveLocalAnchors::Received 0 unbound anchors on load success. Should not be possible..are the anchors from the current space? Trying to load from cloud.");
                    _localAnchors.Clear();
                    foreach (var anchor in anchorData)
                    {
                        ProcessAnchorError(anchor);
                    }
                    RetrieveSharedAnchors(anchors);
                    return;
                }
                var loadedAnchors = result.Value;
                if (!_config.IsOVRTesting)
                {
                    OnEditorUnboundAnchorsLoaded(anchorData);
                }
                else
                {
                    OnOVRUnboundAnchorsLoaded(anchorData, loadedAnchors);
                }
            }
            else
            {
                foreach (var anchor in anchorData)
                {
                    ProcessAnchorError(anchor);
                }

                ShowNotification("Failed to load anchors", NotificationType.Error);
                Debug.LogError(
                    $"SpatialAnchorService::Failed to load anchors {string.Join(", ", anchorIds)} | {result} | {result.Status}");
                OnLoadAnchorErrorEvent?.Invoke(anchorIds);

                if (_data.IsThisUserAHost())
                {
                    OnFatalAnchorLoadError?.Invoke("Could not find local anchors for Host. Please restart and select to override anchors");
                }
                else
                {
                    _localAnchors.Clear();
                    RetrieveSharedAnchors(anchors);
                }
            }
        }

        private async Task RetrieveSharedAnchors(List<SpatialAnchorData> anchorData)
        {
            var userId = _data.ThisUser.Id;
            var anchorIds = anchorData.Where(a => (a.CreatedBy != userId && !_localAnchors.Contains(a.UUID) && a.SharedWith.Contains(userId))).Select(a => a.UUID).ToList();
            if (anchorIds.Count == 0) return;
            foreach (var id in anchorIds)
            {
                foreach (var data in anchorData)
                {
                    if (data.UUID == id)
                    {
                        data.State = SpatialAnchorState.Loading;
                    }
                }
                _data.PendingAnchors.Add(id);
            }

            var unboundAnchors = new List<OVRSpatialAnchor.UnboundAnchor>();
            ShowNotification("Loading spatial anchors");
            //var result = await OVRSpatialAnchor.LoadUnboundSharedAnchorsAsync(anchorIds, unboundAnchors);
            //Debug.Log($"SpatialAnchorService::RetrieveSharedAnchors:LoadComplete : {result} | {unboundAnchors.Count}");
            /*if (result.Success)
            {
                if (unboundAnchors.Count == 0)
                {
                    Debug.LogError("SpatialAnchorService::RetrieveSharedAnchors::Received 0 unbound anchors on load success. Should not be possible.");
                    foreach (var anchor in anchorData)
                    {
                        ProcessAnchorError(anchor);
                    }
                    RetrieveSharedAnchors(anchorData);
                    return;
                }
                var anchors = result.Value;
                if (!_config.IsOVRTesting)
                {
                    OnEditorUnboundAnchorsLoaded(anchorData);
                }
                else
                {
                    OnOVRUnboundAnchorsLoaded(anchorData, anchors);
                }
            }
            else
            {
                Debug.LogError(
                    $"SpatialAnchorService::Failed to load anchors {string.Join(", ", anchorIds)} | {result} | {result.Status}");
                var isFatalError = false;
                foreach (var anchor in anchorData)
                {
                    anchor.LoadAttempts += 1;
                    ProcessAnchorError(anchor);
                    if (anchor.LoadAttempts > MaxLoadAttempts)
                    {
                        isFatalError = true;
                    }
                }

                if (isFatalError)
                {
                    OnFatalAnchorLoadError?.Invoke("Error loading anchors. Maxed out load attempts.");
                }
                else
                {
                    await Task.Delay(1000);
                    ShowNotification("Failed to load anchors. Retrying", NotificationType.Error);
                    RetrieveSharedAnchors(anchorData);
                }
            }*/
        }

        private void OnEditorUnboundAnchorsLoaded(List<SpatialAnchorData> anchorData)
        {
            foreach (var data in anchorData)
            {
                var view = CreateLoadedAnchorView(data);
                switch (data.AnchorType)
                {
                    case SpatialAnchorType.CoLocation:
                        _data.CoLocationAnchor = view;
                        break;
                    case SpatialAnchorType.RobotHome:
                        _data.RobotHomeAnchor = view;
                        break;
                    case SpatialAnchorType.WorldGraph:
                        _data.WorldGraphAnchors[data.Name] = view;
                        break;
                }

                _data.LoadedAnchorsStillProcessing.Add(view);
            }
        }

        private void OnOVRUnboundAnchorsLoaded(List<SpatialAnchorData> anchorData,
            List<OVRSpatialAnchor.UnboundAnchor> unboundAnchors)
        {
            foreach (var data in anchorData)
            {
                foreach (var unboundAnchor in unboundAnchors)
                {
                    if (unboundAnchor.Uuid != data.UUID) continue;
                    
                    Debug.Log($"Loaded anchor {data.AnchorType} | {data.Name}");
                    ShowNotification($"Loaded anchor {data.AnchorType} | {data.Name}");
                    var view = CreateLoadedAnchorView(data);
                    switch (data.AnchorType)
                    {
                        case SpatialAnchorType.CoLocation:
                            _data.CoLocationAnchor = view;
                            break;
                        case SpatialAnchorType.RobotHome:
                            _data.RobotHomeAnchor = view;
                            break;
                        case SpatialAnchorType.WorldGraph:
                            _data.WorldGraphAnchors[data.Name] = view;
                            break;
                    }
                    data.State = SpatialAnchorState.Binding;
                    BindToOVRAnchor(unboundAnchor, view);
                }
            }
        }

        private void BindToOVRAnchor(OVRSpatialAnchor.UnboundAnchor unboundAnchor, SpatialAnchorView anchor)
        {
            try
            {
                Debug.Log($"SpatialAnchorService::Binding Anchor... {anchor.Data.Name}");
                anchor.UnboundAnchor = unboundAnchor;
                unboundAnchor.BindTo(anchor.GetOVRAnchor());
                _data.LoadedAnchorsStillProcessing.Add(anchor);
            }
            catch
            {
                Debug.LogError($"SpatialAnchorService::UnboundAnchor.BindTo has failed");
                ProcessAnchorError(anchor.Data);
                ShowNotification("Failed to bind anchors", NotificationType.Error);
                GameObject.Destroy(anchor.gameObject);
                throw;
            }
        }

        private SpatialAnchorView CreateLoadedAnchorView(SpatialAnchorData data)
        {
            Debug.Log($"SpatialAnchorService::CreateLoadedAnchorView {data.AnchorType} | {data.Name}");
            if (!_config.IsOVRTesting)
            {
                var view = InstantiateAnchor<StubSpatialAnchorView>(_config.AnchorPrefab);
                view.Initialise(data, _isShowDebugView);
                return view;
            }
            else
            {
                var view = InstantiateAnchor<OVRSpatialAnchorView>(_config.AnchorPrefab);
                view.Initialise(data, _isShowDebugView);
                return view;
            }
        }

        private SpatialAnchorView InstantiateAnchorAtPosition<T>(GameObject prefab, Vector3 position,
            Quaternion rotation) where T : SpatialAnchorView
        {
            Debug.Log("SpatialAnchorService::InstantiateAnchorAtPosition");
            var anchorGO = GameObject.Instantiate(prefab, position, rotation);
            return anchorGO.GetComponent<T>();
        }

        private SpatialAnchorView InstantiateAnchor<T>(GameObject prefab) where T : SpatialAnchorView
        {
            Debug.Log("SpatialAnchorService::InstantiateAnchor");
            var anchorGO = GameObject.Instantiate(prefab);
            return anchorGO.GetComponent<T>();
        }

        public void Update()
        {
            ProcessLoadingAnchors();
        }

        private void Share(SpatialAnchorView anchor)
        {
            Debug.Log($"SpatialAnchorService::Share: {anchor.Data.Name}");
            var users = _data.OtherUserIds;
            anchor.CleanListeners();
            anchor.OnShareSuccess += OnAnchorShared;
            anchor.OnShareError += OnShareError;
            ShowNotification($"Sharing anchor {anchor.Data.AnchorType} with {users.Count} users");
            anchor.Share(users);
        }

        private void ProcessLoadingAnchors()
        {
            if (_data == null) return;
            var anchors = _data.LoadedAnchorsStillProcessing;

            for (var i = anchors.Count - 1; i >= 0; i--)
            {
                var anchor = anchors[i];
                if (!anchor.IsLoaded()) continue;
                if (!_isCoLocated && anchor.Data.AnchorType != SpatialAnchorType.CoLocation) continue;
                Debug.Log($"SpatialAnchorService::AnchorLoaded: {anchor.Data.AnchorType} | {anchor.Data}");
                _data.LoadedAnchorsStillProcessing.RemoveAt(i);
                _data.PendingAnchors.Remove(anchor.Data.UUID);
                _data.ReadyAnchors.Add(anchor.Data.UUID, anchor);
                switch (anchor.Data.AnchorType)
                {
                    case SpatialAnchorType.CoLocation:
                        AlignToCoLocationAnchor(anchor, _playerTransform);
                        break;
                    case SpatialAnchorType.RobotHome:
                        AlignRobotHomeToAnchor(anchor, _robotHome);
                        break;
                    case SpatialAnchorType.WorldGraph:
                        _data.WorldGraphAnchors[anchor.Data.Name] = anchor;
                        break;
                }

                anchor.Data.State = SpatialAnchorState.Ready;
                OnAnchorLoadedEvent?.Invoke(anchor);
                if (anchor.Data.CreatedBy == _data.ThisUser.Id)
                {
                    Share(anchor);
                }
                UpdateLocalAnchors(anchor.Data.UUID);
            }
        }

        private async Task AlignToCoLocationAnchor(SpatialAnchorView anchor, Transform player)
        {
            player.position = Vector3.zero;
            player.eulerAngles = Vector3.zero;

            await Task.Yield();

            var anchorTransform = anchor.transform;
            var trackingSpacePose = anchorTransform.ToTrackingSpacePose(_mainCamera);
            anchorTransform.SetPositionAndRotation(trackingSpacePose.position, trackingSpacePose.orientation);

            // Transform the camera to the inverse of the anchor pose to align the scene
            player.position = anchorTransform.InverseTransformPoint(Vector3.zero);
            player.eulerAngles = new Vector3(0, -anchorTransform.eulerAngles.y, 0);

            // Update the world space position of the anchor so it renders in a consistent world-locked position.
            var worldSpacePose = trackingSpacePose.ToWorldSpacePose(_mainCamera);
            anchorTransform.SetPositionAndRotation(worldSpacePose.position, worldSpacePose.orientation);
            _isCoLocated = true;
            
            _coLocationContainer.position = trackingSpacePose.position;
            _coLocationContainer.rotation = trackingSpacePose.orientation;
        }

        private async Task AlignRobotHomeToAnchor(SpatialAnchorView anchor, Transform robotHome)
        {
            if (!_config.IsOVRTesting) return;
            await Task.Yield();
            Debug.Log($"SpatialAnchorService::AlignRobotHomeToAnchor");
            AlignTransformToAnchor(robotHome, anchor);
        }

        private void AlignTransformToAnchor(Transform alignTransform, SpatialAnchorView anchor)
        {
            var transform = anchor.transform;
            alignTransform.position = transform.position;
            alignTransform.rotation = transform.rotation;
        }

        public void Dispose()
        {
            _isDisposed = true;
            OnAnchorSavedEvent = null;
            OnAnchorLoadedEvent = null;
            OnAnchorSharedEvent = null;
        }

        public void CreateColocationAnchor()
        {
            CreateAnchor("Co-Location Anchor", SpatialAnchorType.CoLocation,
                Vector3.zero,
                Quaternion.identity);
        }

        public void RemoveDeadAnchorsFromSave(Room roomData)
        {
            var activeAnchors = new List<Guid>();
            if(roomData.ColocationAnchor != null) activeAnchors.Add(roomData.ColocationAnchor.UUID);
            if(roomData.RobotHomeAnchor != null) activeAnchors.Add(roomData.RobotHomeAnchor.UUID);
            foreach (var worldGraphAnchor in roomData.WorldGraphAnchors)
            {
                activeAnchors.Add(worldGraphAnchor.UUID);
            }

            var anchorsToDelete = new List<Guid>();
            foreach (var localAnchor in _localAnchors)
            {
                if(!activeAnchors.Contains(localAnchor)) anchorsToDelete.Add(localAnchor);
            }

            foreach (var guid in anchorsToDelete)
            {
                activeAnchors.Remove(guid);
                // todo delete anchor from the cloud too
            }
        }
    }
}