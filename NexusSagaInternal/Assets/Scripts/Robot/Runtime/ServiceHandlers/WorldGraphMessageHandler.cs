using System;
using System.Collections.Generic;
using Logger.Runtime;
using Newtonsoft.Json;
using Notifications.Runtime.Data;
using Notifications.Runtime.Systems;
using Robot.Runtime.Data;
using Robot.Runtime.Data.WorldGraph;
using Robot.Runtime.Messages.Server;
using Robot.Runtime.Utils;
using SiroComms.Runtime.Messages.Server;
using SiroComms.Runtime.Services;
using UnityEngine;

namespace Robot.Runtime.ServiceHandlers
{
    public class WorldGraphMessageHandler : IMessageHandler
    {
        public const string TYPE = "world_graph";
        public const string WORLD_GRAPH = "world_graph";
        public const string EDIT = "edit";

        public Action OnWorldGraph;
        public Action<WorldGraphNodeData> OnNodeRemoved;
        public Action<WorldGraphNodeData> OnNodeAdded;
        public Action<string, WorldGraphNodeData> OnNodeRoomChangedRemoved;
        public Action<WorldGraphNodeData, WorldGraphNodeData> OnReceptacleItemsRemoved;
        public Action<WorldGraphNodeData, WorldGraphNodeData> OnReceptacleItemsAdded;

        private WorldGraphData _data;
        private bool _isInitialGraphProcessed;

        private HashSet<string> _filteredRelationships = new HashSet<string>() { "under", "next to" };

        public WorldGraphMessageHandler(WorldGraphData data)
        {
            _data = data;
        }
        
        public void HandleMessage(string messageType, string message)
        {
            switch (messageType)
            {
                case WORLD_GRAPH:
                    HandleWorldGraph(message);
                    break;
                case EDIT:
                    HandleWorldGraphEditsMessage(message);
                    break;
                
            }
        }

        private void HandleWorldGraphEditsMessage(string message)
        {
            if (!_isInitialGraphProcessed) return;
            var editData = ServerMessage<WorldGraphNodeEditDTO>.Deserialize(message).Data;
            var node = editData.Node;
            switch (node.Operation)
            {
                case WorldGraphOperation.NODE_ADDED:
                    ProcessItemAddedToWorldGraph(node);
                    break;
                case WorldGraphOperation.NODE_REMOVED: 
                    ProcessItemRemovedFromWorldGraph(node);
                    break;
                case WorldGraphOperation.ITEM_ADDED_TO_RECEPTACLE:
                    ProcessItemAddedToReceptacle(node);
                    break;
                case WorldGraphOperation.ITEM_REMOVED_FROM_RECEPTACLE:
                    // we only receive an add, on the assumption that item will switch from one receptacle to another,
                    // never be de-parented
                    // ProcessItemRemovedFromReceptacle(node); 
                    break;
            }
        }

        private void ProcessItemRemovedFromWorldGraph(WorldGraphNodeEdit node)
        {
            Debug.Log($"WorldGraphTest::ProcessItemRemovedFromWorldGraph: {node.Id} | {_data.Nodes.ContainsKey(node.Id)} | {node.Room}");
            if (!IsRoomAllowed(node.Room))
            {
                Debug.LogError($"Received bad room data on node. ignoring item {node.Id} | {node.Room}");
                return;
            }
            if (!_data.Nodes.TryGetValue(node.Id, out var dataNode)) return;
            dataNode.IsRemoved = true;
            RemoveItemFromReceptacle(dataNode);
            if (_data.Nodes.ContainsKey(node.Id))
            {
                _data.Nodes.Remove(node.Id);
            }
            OnNodeRemoved?.Invoke(dataNode);
        }

        private void RemoveItemFromReceptacle(WorldGraphNodeData node)
        {
            if (!IsRoomAllowed(node.Room))
            {
                Debug.LogError($"Received bad room data on node. ignoring item {node.Id} | {node.Room}");
                return;
            }
            var currentReceptacleId = node.Receptacle;
            if (string.IsNullOrWhiteSpace(node.Receptacle)) return;
            if (!_data.Nodes.TryGetValue(currentReceptacleId, out var currentReceptacle)) return;
            currentReceptacle.Items.Remove(node);
            OnReceptacleItemsRemoved?.Invoke(currentReceptacle, node);
        }

        private void AddItemToReceptacle(WorldGraphNodeData data, WorldGraphNodeEdit node)
        {
            if (!IsRoomAllowed(node.Room))
            {
                Debug.LogError($"Received bad room data on node. ignoring item {node.Id} | {node.Room}");
                return;
            }
            var newReceptacleId = node.Receptacle;
            if (string.IsNullOrWhiteSpace(newReceptacleId)) return;
            if (!_data.Nodes.TryGetValue(newReceptacleId, out var receptacle)) return;
            receptacle.Items.Add(data);
            data.Receptacle = newReceptacleId;
            if (data.Room != receptacle.Room)
            {
                OnNodeRoomChangedRemoved?.Invoke(data.Room, data);
                data.Room = receptacle.Room;
            }
            OnReceptacleItemsAdded?.Invoke(receptacle, data);
        }

        private void ProcessItemAddedToWorldGraph(WorldGraphNodeEdit node)
        {
            if (_data.Nodes.ContainsKey(node.Id))
            {
                Debug.LogError($"Item with id {node.Id} is already in the graph. ignoring | {node.Room}");
                return;
            }
            if (!IsRoomAllowed(node.Room))
            {
                Debug.LogError($"Received bad room data on node. ignoring item {node.Id} | {node.Room}");
                return;
            }
            var data = new WorldGraphNodeData
            {
                Id = node.Id,
                Name = node.ObjectName.Replace("_", " "),
                Position = RobotConversion.RightHandToLeftHand(node.Position),
                Yaw = node.Yaw * -1,
                Extents = RobotConversion.RightHandToLeftHand(node.Extents),
                Room = node.Room,
                Category = node.Category
            };
            _data.Nodes.Add(data.Id, data);

            AddItemToReceptacle(data, node);
            OnNodeAdded?.Invoke(data);
        }

        private bool IsRoomAllowed(string roomName)
        {
            return !string.IsNullOrWhiteSpace(roomName) && roomName.ToLower() != "unknown";
        }

        private void ProcessItemAddedToReceptacle(WorldGraphNodeEdit node)
        {
            Debug.Log($"WorldGraphTest::ProcessItemAddedToReceptacle: {node.Id}");
            if (_data.Nodes.TryGetValue(node.Id, out var data))
            {
                data.Position = RobotConversion.RightHandToLeftHand(node.Position);
                data.Extents = RobotConversion.RightHandToLeftHand(node.Extents);
                RemoveItemFromReceptacle(data);
                AddItemToReceptacle(data, node);
            }
            else
            {
                DevLog.LogError($"Received update for node that isn't yet stored {node.Id}");
            }
        }

        private void HandleWorldGraph(string message)
        {
            var worldGraph = ServerMessage<WorldGraphDTO>.Deserialize(message).Data;
            _data.Nodes.Clear();
            foreach (var worldGraphNode in worldGraph.Graph.Nodes)
            {
                if (!IsRoomAllowed(worldGraphNode.Room))
                {
                    Debug.LogError($"Received bad room data on node. ignoring item {worldGraphNode.Id} | {worldGraphNode.Room}");
                    continue;
                }
                var node = new WorldGraphNodeData
                {
                    Id = worldGraphNode.Id,
                    Name = worldGraphNode.ObjectName.Replace(" ", "_"),
                    Position = RobotConversion.RightHandToLeftHand(worldGraphNode.Position),
                    Extents = RobotConversion.RightHandToLeftHand(worldGraphNode.Extents),
                    Room = worldGraphNode.Room,
                    Category = worldGraphNode.Category,
                    Yaw = worldGraphNode.Yaw * -1
                };
                _data.Nodes.Add(node.Id, node);
            }

            foreach (var edge in worldGraph.Graph.Edges)
            {
                if(_filteredRelationships.Contains(edge.Relationship)) continue;
                if (_data.Nodes.TryGetValue(edge.SourceId, out var source))
                {
                    source.Receptacle = edge.TargetId;
                }
                if (_data.Nodes.TryGetValue(edge.TargetId, out var target))
                {
                    target.Items.Add(source);
                }
            }

            _isInitialGraphProcessed = true;
            OnWorldGraph?.Invoke();
        }
    }
}