using System;
using System.Collections.Generic;
using Multiplayer.Runtime.Services;
using Robot.Runtime.Data;
using Robot.Runtime.Data.WorldGraph;
using Robot.Runtime.ServiceHandlers;
using Robot.Runtime.Utils;
using Robot.Runtime.View;
using UnityEngine;

namespace Robot.Runtime.Systems
{
    public class WorldGraphVisualsSystem : IMultiplayerFeatureService
    {
        public Action OnWorldGraphProcessed;
        private State _currentState;
        private WorldGraphMessageHandler _handler;

        private List<FurnitureHighlight> _activeFurniture = new();
        private List<WorldGraphObjectHighlight> _activeObjects = new();
        private Dictionary<string, FurnitureHighlight> _furnitureHighlightLookup = new();
        private Dictionary<string, WorldGraphObjectHighlight> _objectHighlightLookup = new();
        private Dictionary<string, int> _furnitureTypeTracker = new();
        private Dictionary<string, int> _objectTypeTracker = new();
        private HashSet<string> _roomList = new();

        private Dictionary<string, WorldGraphPanel> _panelLookup = new();

        private HighlightPool<FurnitureHighlight> _furnitureHighlightPool;
        private HighlightPool<WorldGraphObjectHighlight> _objectHighlightPool;
        private HighlightPool<WorldGraphIcon> _iconPool;
        private HighlightPool<WorldGraphPanel> _roomPanelPool;
        private WorldGraphData _data;
        private Transform _cameraTransform;
        private bool _isInitialised;
        private bool _isActive;
        private int _numberPanels;
        private bool _isWorldGraphInitMessageReceived;
        private bool _isWorldGraphInitialised;
        private Color _defaultColor;
        private bool _isObjectHighlightsEnabled;
        private bool _isWorldGraph2DSurfacesEnabled;

        public bool IsWorldGraphInitialised => _isWorldGraphInitialised;

        private enum State
        {
            None,
            Showing,
            OnDisplay,
            Hiding
        }

        public HashSet<string> RoomList => _roomList;

        public void Initialise(Transform cameraTransform, WorldGraphMessageHandler handler, WorldGraphData data,
            Color worldGraphColor)
        {
            _defaultColor = worldGraphColor;
            _cameraTransform = cameraTransform;
            _handler = handler;
            _data = data;

            AddListeners();
            InitialisePools();
            _isInitialised = true;
        }

        public void Activate()
        {
            _currentState = State.Showing;
            _isActive = true;
            if (!_isInitialised) return;
            DisplayAllItems();
        }

        public void Deactivate()
        {
            _isActive = false;
            if (_currentState != State.Showing && _currentState != State.OnDisplay) return;
            _currentState = State.Hiding;
            HideAllItems();
        }

        public bool IsActive()
        {
            return _isActive;
        }

        private void AddListeners()
        {
            _handler.OnWorldGraph += OnInitialWorldGraphMessage;
            _handler.OnNodeAdded += OnNodeAdded;
            _handler.OnNodeRemoved += OnNodeRemoved;
            _handler.OnReceptacleItemsAdded += OnReceptacleItemsAdded;
            _handler.OnReceptacleItemsRemoved += OnReceptacleItemsRemoved;
            _handler.OnNodeRoomChangedRemoved += OnNodeRoomChanged;
        }

        private void OnNodeRoomChanged(string lastRoom, WorldGraphNodeData node)
        {
            var lastRoomPanel = GetPaneForRoom(lastRoom);
            if (node.Category == WorldGraphCategory.Furniture)
            {
                var iconView = lastRoomPanel.RemoveFurniture(node.Id);
                RepoolIcon(iconView);
                AddIconToRoomPanel(node, false);
            }
            else
            {
                var iconView = lastRoomPanel.RemoveObject(node.Id);
                RepoolIcon(iconView);
                AddIconToRoomPanel(node, true);
            }
        }

        private void OnReceptacleItemsRemoved(WorldGraphNodeData receptacle, WorldGraphNodeData node)
        {
            if (_furnitureHighlightLookup.TryGetValue(receptacle.Id, out var receptacleView))
            {
                receptacleView.RemoveObject(node);
            }
        }

        private void OnReceptacleItemsAdded(WorldGraphNodeData receptacle, WorldGraphNodeData node)
        {
            if (_furnitureHighlightLookup.TryGetValue(receptacle.Id, out var receptacleView))
            {
                if (!GetSprite(node, out var icon))
                {
                    Debug.LogError($"No icon for World graph object {node.Name}");
                }

                receptacleView.AddObject(node, icon);
            }

            if (_objectHighlightLookup.TryGetValue(node.Id, out var highlight))
            {
                _objectHighlightLookup.Remove(node.Id);
                highlight.IsDisposed = true;
                highlight.Hide();
            }

            //@TODO some tracking highlight here once established that EE pose tracks ok   
            if (!node.IsHeldByRobot() && !node.IsHeldByPerson())
            {
                AddObjectHighlight(node);
            }
        }

        private void OnNodeRemoved(WorldGraphNodeData node)
        {
            Debug.Log($"WorldGraph::WorldGraphVisuals::OnNodeRemoved {node.Category}");
            if (node.Category == WorldGraphCategory.Furniture)
            {
                RemoveFurniture(node);
            }
            else
            {
                RemoveObject(node);
            }
        }

        private void OnNodeAdded(WorldGraphNodeData node)
        {
            if (node.Category == WorldGraphCategory.Furniture)
            {
                AddFurniture(node);
            }
            else
            {
                AddObject(node);
            }
        }

        private void InitialisePools()
        {
            _iconPool = new HighlightPool<WorldGraphIcon>(_data.ViewContainer, 200, _data.IconPrefab);
            _furnitureHighlightPool =
                new HighlightPool<FurnitureHighlight>(_data.ViewContainer, 100, _data.FurnitureHighlightPrefab);
            _objectHighlightPool =
                new HighlightPool<WorldGraphObjectHighlight>(_data.ViewContainer, 100, _data.ObjectHighlightPrefab);
            _roomPanelPool = new HighlightPool<WorldGraphPanel>(_data.ViewContainer, 10, _data.PanelPrefab);
        }

        public WorldGraphPanel AnchorPanelToView(Transform view, string roomName)
        {
            Debug.Log($"WorldGraph::AnchorPanelToView: {roomName}");
            var panel = GetPaneForRoom(roomName);
            var transform = panel.transform;
            transform.SetParent(view, false);
            transform.localPosition = Vector3.zero;
            transform.localRotation = Quaternion.identity;
            panel.Initialise(roomName, _iconPool, _defaultColor);
            if (_isActive)
            {
                panel.Show();
            }
            return panel;
        }

        public void Dispose()
        {
            if (_handler == null) return;
            _handler.OnWorldGraph -= OnInitialWorldGraphMessage;
            _handler.OnNodeAdded -= OnNodeAdded;
            _handler.OnNodeRemoved -= OnNodeRemoved;
            _handler.OnNodeRoomChangedRemoved -= OnNodeRoomChanged;
            _handler.OnReceptacleItemsAdded -= OnReceptacleItemsAdded;
            _handler.OnReceptacleItemsRemoved -= OnReceptacleItemsRemoved;
        }

        private void OnInitialWorldGraphMessage()
        {
            _handler.OnWorldGraph -= OnInitialWorldGraphMessage;
            _isWorldGraphInitialised = true;
            Reset();
            var worldGraphNodes = _data.Nodes;
            Debug.Log($"WorldGraph::OnInitialWorldGraphMessage:: {_data.Nodes.Count}");
            foreach (var node in worldGraphNodes)
            {
                var item = node.Value;
                if (item.Category == WorldGraphCategory.Furniture)
                {
                    AddFurniture(item);
                }
                else
                {
                    AddObject(item);
                }
            }

            if (_isInitialised && _isActive)
            {
                DisplayAllItems();
            }

            OnWorldGraphProcessed?.Invoke();
        }

        private void AddObject(WorldGraphNodeData item)
        {
            if (_objectHighlightLookup.ContainsKey(item.Id)) return;
            _roomList.Add(item.Room);
            AddIconToRoomPanel(item, true);
            AddObjectHighlight(item);
            IncrementTracker(_objectTypeTracker, item.Name);
        }

        private void AddObjectHighlight(WorldGraphNodeData item)
        {
            if(!_isObjectHighlightsEnabled) return;
            var highlight = _objectHighlightPool.Get();
            highlight.Initialise(item, _cameraTransform, _defaultColor);
            _objectHighlightLookup.Add(item.Id, highlight);
            _activeObjects.Add(highlight);
            if (!IsAnimateInVisual())
            {
                highlight.Popup();
            }

            if (IsAnimateInVisual())
            {
                highlight.Show();
            }
        }

        private bool IsAnimateInVisual()
        {
            return _currentState == State.OnDisplay || _currentState == State.Showing;
        }

        private bool GetSprite(WorldGraphNodeData item, out Sprite sprite)
        {
            foreach (var mapping in _data.IconConfig.Mappings)
            {
                if (item.Name.Contains(mapping.Id))
                {
                    sprite = mapping.Icon;
                    return true;
                }                
            }

            sprite = item.Category == WorldGraphCategory.Furniture
                ? _data.IconConfig.UnknownFurniture
                : _data.IconConfig.UnknownObject;
            Debug.LogError($"Using unknown icon for item : {item.Id} | {item.Name}");
            return false;
        }

        private int IncrementTracker(Dictionary<string, int> tracker, string objectType)
        {
            if (tracker.TryGetValue(objectType, out var amount))
            {
                tracker[objectType] = amount + 1;
            }
            else
            {
                tracker.Add(objectType, 1);
            }

            return amount;
        }

        private WorldGraphIcon GetIconView(WorldGraphNodeData item, Sprite icon, bool isShowLabel)
        {
            var view = _iconPool.Get();
            var isDisplayLabel = isShowLabel && !string.IsNullOrWhiteSpace(item.Name);
            var label = isDisplayLabel ? item.Name.Replace("_", " ") : null;
            view.Initialise(item, icon, _defaultColor, label);
            Debug.Log($"WorldGraph::GetIconView: {view.Node}");
            return view;
        }

        private void RemoveFurniture(WorldGraphNodeData item)
        {
            if (_furnitureHighlightLookup.TryGetValue(item.Id, out var view))
            {
                _furnitureHighlightLookup.Remove(item.Id);
                view.Hide();
            }

            var panel = GetPaneForRoom(item.Room);
            if (panel == null)
            {
                Debug.LogError($"RemoveFurniture::Object is not assigned to a room. Exiting {item}");
                return;
            }

            var iconView = panel.RemoveFurniture(item.Id);
            RepoolIcon(iconView);
        }

        private void RemoveObject(WorldGraphNodeData item)
        {
            Debug.Log($"WorldGraph::WorldGraphVisuals::RemoveObject {item.Id}");
            if (_objectHighlightLookup.TryGetValue(item.Id, out var view))
            {
                _objectHighlightLookup.Remove(item.Id);
                view.Hide();
            }

            RemoveIconFromPanel(item);
        }

        private void RemoveIconFromPanel(WorldGraphNodeData item)
        {
            var panel = GetPaneForRoom(item.Room);
            Debug.Log($"WorldGraph::WorldGraphVisuals::RemoveIconFromPanel {item.Id} | {panel} | {item.Room}");
            if (panel == null)
            {
                Debug.LogError($"RemoveObject::Object is not assigned to a room. Exiting {item}");
                return;
            }

            var iconView = panel.RemoveObject(item.Id);
            Debug.Log($"WorldGraph::WorldGraphVisuals::RemoveIconFromPanel:iconView {item.Id} | {iconView}");
            // RepoolIcon(iconView);
        }

        private void RepoolIcon(WorldGraphIcon iconView)
        {
            if (iconView == null)
            {
                Debug.LogError($"Attempting to remove an icon from the panel that doesn't exist");
                return;
            }

            _iconPool.Return(iconView);
        }

        private void AddFurniture(WorldGraphNodeData item)
        {
            if (_furnitureHighlightLookup.ContainsKey(item.Id)) return;
            _roomList.Add(item.Room);
            AddIconToRoomPanel(item, false);
            AddFurnitureHighlight(item);
            IncrementTracker(_furnitureTypeTracker, item.Name);
        }

        private void AddIconToRoomPanel(WorldGraphNodeData item, bool isShowLabel)
        {
            if (!GetSprite(item, out var icon))
            {
                Debug.Log($"World graph item not supported {item.Name}");
            }

            var panel = GetPaneForRoom(item.Room);
            if (panel == null)
            {
                Debug.LogError($"AddIconToRoomPanel::Object is not assigned to a room. Exiting {item}");
                return;
            }

            var iconView = GetIconView(item, icon, isShowLabel);
            if (item.Category == WorldGraphCategory.Furniture)
            {
                panel.AddFurniture(iconView);
            }
            else
            {
                panel.AddObject(iconView);
            }
        }

        private int GetNumberItemsInGraph(WorldGraphNodeData node)
        {
            var tracker = node.Category == WorldGraphCategory.Furniture ? _furnitureTypeTracker : _objectTypeTracker;
            var key = node.Name;
            if (!tracker.ContainsKey(key))
            {
                tracker.Add(key, 0);
            }

            return tracker[key];
        }

        private void AddFurnitureHighlight(WorldGraphNodeData item)
        {
            var highlight = _furnitureHighlightPool.Get();
            highlight.name = item.Id;
            var objectName = item.Name.ToLower();
            var isShowFlatExtends = false;
            if(_isWorldGraph2DSurfacesEnabled)
            {
                foreach(var flatItem in _data.IconConfig.FlatDisplayList)
                {
                    if(!objectName.Contains(flatItem)) continue;
                    isShowFlatExtends = true;
                    break;
                }
            }
            highlight.Initialise(_cameraTransform, item, _iconPool, isShowFlatExtends,
                _data.IconConfig.IsUseObjectIdForFurnitureLabel, _defaultColor);
            foreach (var containedItem in item.Items)
            {
                if (!GetSprite(containedItem, out var icon))
                {
                    Debug.LogError($"World graph object not supported {containedItem.Name}");
                }

                var iconView = highlight.AddObject(containedItem, icon);
                iconView.ShowImmediate();
            }

            _furnitureHighlightLookup.Add(item.Id, highlight);
            _activeFurniture.Add(highlight);

            if (IsAnimateInVisual())
            {
                highlight.Show();
            }
        }

        private WorldGraphPanel GetPaneForRoom(string room)
        {
            if (string.IsNullOrWhiteSpace(room)) return null;
            if (!_panelLookup.TryGetValue(room, out var panel))
            {
                Debug.Log($"WorldGraph::GetPaneForRoom: {room}:: could not find panel - creating");
                panel = _roomPanelPool.Get();
                panel.Initialise(room, _iconPool, _defaultColor);
                var cameraPosition = _cameraTransform.position;
                var panelPosition = cameraPosition + new Vector3(2 + _numberPanels, 1, 0f);
                panel.transform.position = panelPosition;
                var lookRotation = Quaternion.LookRotation(panelPosition - cameraPosition);
                lookRotation = Quaternion.Euler(new Vector3(0f, lookRotation.eulerAngles.y, 0f));
                panel.transform.rotation = lookRotation;
                _panelLookup.Add(room, panel);
                _numberPanels++;
            }

            return panel;
        }

        private void Reset()
        {
            Debug.Log($"WorldGraph::WorldGraphVisualsSystem:reset");
            foreach (var worldGraphPanel in _panelLookup)
            {
                var panel = worldGraphPanel.Value;
                var objects = panel.ActiveObjects;
                var furniture = panel.ActiveFurniture;
                foreach (var worldGraphIcon in objects)
                {
                    _iconPool.Return(worldGraphIcon);
                }

                foreach (var worldGraphIcon in furniture)
                {
                    _iconPool.Return(worldGraphIcon);
                }

                panel.Reset();
            }
        }

        private void DisplayAllItems()
        {
            foreach (var panel in _panelLookup)
            {
                if (panel.Value.IsDisposed) continue;
                panel.Value.Show();
            }

            foreach (var highlight in _activeObjects)
            {
                if (highlight.IsDisposed) continue;
                highlight.Show();
            }

            foreach (var highlight in _activeFurniture)
            {
                if (highlight.IsDisposed) continue;
                highlight.Show();
            }
        }

        private void HideAllItems()
        {
            foreach (var panel in _panelLookup)
            {
                panel.Value.Hide();
            }

            foreach (var highlight in _activeObjects)
            {
                highlight.Hide();
            }

            foreach (var highlight in _activeFurniture)
            {
                highlight.Hide();
            }
        }

        public void Tick()
        {
            CheckForObjectRemoval();
            CheckForFurnitureRemoval();
            foreach (var panel in _panelLookup)
            {
                panel.Value.Tick();
            }
        }

        private void CheckForFurnitureRemoval()
        {
            for (var i = _activeFurniture.Count - 1; i >= 0; --i)
            {
                var view = _activeFurniture[i];
                if (view.Data.IsRemoved && view.State == HighlightState.Hidden)
                {
                    _activeFurniture.RemoveAt(i);
                    _furnitureHighlightPool.Return(view);
                }
            }
        }

        private void CheckForObjectRemoval()
        {
            for (var i = _activeObjects.Count - 1; i >= 0; --i)
            {
                var view = _activeObjects[i];
                if ((view.IsDisposed || view.Data.IsRemoved) && view.State == HighlightState.Hidden)
                {
                    _activeObjects.RemoveAt(i);
                    _objectHighlightPool.Return(view);
                }
            }
        }

        public void SetWorldGraphObjectsEnabled(bool isEnabled)
        {
            _isObjectHighlightsEnabled = isEnabled;
        }

        public void SetWorldGraph2DSurfacesEnabled(bool isEnabled)
        {
            _isWorldGraph2DSurfacesEnabled = isEnabled;
        }
    }
}