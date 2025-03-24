using System.Collections.Generic;
using Multiplayer.Runtime.Services;
using Notifications.Runtime.Data;
using Notifications.Runtime.Systems;
using UnityEngine;

namespace Multiplayer.Runtime.Systems
{
    public class AppFeatureHandler
    {
        private readonly MultiplayerMessageHandler _messageHandler;
        private readonly NotificationSystem _notificationSystem;
        private readonly Dictionary<string, List<IMultiplayerFeatureService>> _featureActivationTracker = new();
        private readonly HashSet<string> _activeList = new();
        private bool _isEnabled;

        public AppFeatureHandler(NotificationSystem notificationSystem, MultiplayerMessageHandler messageHandler)
        {
            _notificationSystem = notificationSystem;
            _messageHandler = messageHandler;
            AddListeners();
        }

        public void AddFeature(string featureId, IMultiplayerFeatureService service)
        {
            Debug.Log($"AppFeatureHandler::AddFeature {featureId} ");
            if (_featureActivationTracker.TryGetValue(featureId, out var value))
            {
                if (value.Contains(service)) return;
                value.Add(service);
            }
            else
            {
                _featureActivationTracker.Add(featureId, new List<IMultiplayerFeatureService>{service});
            }
        }
        
        public void RemoveFeature(string featureId)
        {
            if (!_featureActivationTracker.ContainsKey(featureId)) return;
            _featureActivationTracker.Remove(featureId);
        }

        private void AddListeners()
        {
            _messageHandler.OnActivateFeature += OnActivateFeature;
            _messageHandler.OnDeactivateFeature += OnDeactivateFeature;
        }

        private void OnActivateFeature(string feature)
        {
            _activeList.Add(feature);
            Debug.Log($"AppFeatureHandler::OnActivateFeature {feature} | {_featureActivationTracker.ContainsKey(feature)}");
            if (!_featureActivationTracker.TryGetValue(feature, out var featureServices)) return;
            if (_isEnabled)
            {
                foreach (var service in featureServices)
                {
                    service.Activate();
                }
                _notificationSystem.ShowNotification($"{feature} : ON", NotificationType.Info);    
            }
        }
        
        private void OnDeactivateFeature(string feature)
        {
            _activeList.Remove(feature);
            if (!_featureActivationTracker.TryGetValue(feature, out var featureServices)) return;
            foreach (var service in featureServices)
            {
                service.Deactivate();
            }
            _notificationSystem.ShowNotification($"{feature} : OFF", NotificationType.Info);
        }

        public void HandleActiveFeatureList(List<string> activeFeatures)
        {
            foreach (var featureService in _featureActivationTracker)
            {
                var isFound = false;
                foreach (var feature in activeFeatures)
                {
                    if(feature != featureService.Key) continue;
                    if (_isEnabled)
                    {
                        foreach (var service in featureService.Value)
                        {
                            service.Activate();
                        }
                    }

                    isFound = true;
                    break;
                }

                if (isFound)
                {
                    _activeList.Add(featureService.Key);
                }
                else 
                {
                    _activeList.Remove(featureService.Key);
                    foreach (var service in featureService.Value)
                    {
                        service.Deactivate();
                    }
                }
            }
        }

        public void Enable()
        {
            _isEnabled = true;
            foreach (var feature in _activeList)
            {
                if (!_featureActivationTracker.TryGetValue(feature, out var featureServices)) continue;
                foreach (var service in featureServices)
                {
                    service.Activate();
                }
            }
        }

        public void Disable()
        {
            _isEnabled = false;
        }

        public bool IsFeatureActive(string featureId)
        {
            return _activeList.Contains(featureId);
        }
    }
}