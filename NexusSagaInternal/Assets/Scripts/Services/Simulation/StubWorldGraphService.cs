using System.Collections.Generic;
using System.Linq;
using Robot.Runtime.Data.WorldGraph;
using Robot.Runtime.Messages.Server;
using Robot.Runtime.ServiceHandlers;
using SiroComms.Runtime.Services.Utils;
using UnityEngine;

namespace SiroComms.Runtime.Services
{
    public class StubWorldGraphService : StubService
    {
        private readonly WorldGraph _worldGraph;
        private readonly WorldGraph _robotGraph;

        public StubWorldGraphService(Queue<string> messageQueue, WorldGraph worldGraph) : base(
            messageQueue)
        {
            _worldGraph = worldGraph;
            _robotGraph = new WorldGraph();
            var nodes = new List<WorldGraphNode>();
            foreach (var node in worldGraph.Nodes)
            {
                var rosNode = new WorldGraphNode
                {
                    Category = node.Category,
                    ObjectName = node.ObjectName,
                    Room = node.Room,
                    Extents = RobotConversion.UnityToRobot(node.Extents),
                    Position = RobotConversion.UnityToRobot(node.Position),
                    Id = node.Id
                };
                nodes.Add(rosNode);
            }

            _robotGraph.Nodes = nodes;
            var edges = new List<WorldGraphEdge>();
            foreach (var edge in _worldGraph.Edges)
            {
                var copy = new WorldGraphEdge
                    { Relationship = edge.Relationship, SourceId = edge.SourceId, TargetId = edge.TargetId };
                edges.Add(copy);
            }

            _robotGraph.Edges = edges;
        }

        public void ProcessItemPickedUp(string itemId)
        {
            var node = _worldGraph.Nodes.FirstOrDefault(i => i.Id == itemId);
            if (node == null) return;
            for (var i = _worldGraph.Edges.Count - 1; i >= 0; --i)
            {
                var edge = _worldGraph.Edges[i];
                if (edge.SourceId != itemId) continue;
                if (edge.Relationship != NodeRelationship.ON) continue;
                edge.TargetId = "robot";
                var data = GetBasicEditData(node, WorldGraphOperation.ITEM_ADDED_TO_RECEPTACLE);
                data.Node.Receptacle = "robot[1]";
                SendMessage(WorldGraphMessageHandler.TYPE, WorldGraphMessageHandler.EDIT, data);
            }
        }

        public void ProcessItemPlaced(string itemId, string receptacleId, Vector3 position)
        {
            var node = _worldGraph.Nodes.FirstOrDefault(i => i.Id == itemId);
            if (node == null)
            {
                Debug.LogError($"ProcessItemPlaced:Could not find node {itemId} | {receptacleId}");
            }

            var existingOnEdge = _worldGraph.Edges.FirstOrDefault(i => i.SourceId == itemId);
            if (existingOnEdge == null)
            {
                var edgeData = new WorldGraphEdge
                    { SourceId = itemId, TargetId = receptacleId, Relationship = NodeRelationship.ON };
                _worldGraph.Edges.Add(edgeData);
                var data = GetBasicEditData(node, WorldGraphOperation.ITEM_ADDED_TO_RECEPTACLE);
                data.Node.Position = RobotConversion.UnityToRobot(position);
                data.Node.Receptacle = receptacleId;
                SendMessage(WorldGraphMessageHandler.TYPE, WorldGraphMessageHandler.EDIT, data);
            }
            else
            {
                existingOnEdge.TargetId = receptacleId;
                var data = GetBasicEditData(node, WorldGraphOperation.ITEM_ADDED_TO_RECEPTACLE);
                data.Node.Receptacle = receptacleId;
                data.Node.Position = RobotConversion.UnityToRobot(position);
                SendMessage(WorldGraphMessageHandler.TYPE, WorldGraphMessageHandler.EDIT, data);
            }
        }

        private WorldGraphNodeEditDTO GetBasicEditData(WorldGraphNode node, string operation, string receptacle = null)
        {
            return new WorldGraphNodeEditDTO
            {
                Node = new WorldGraphNodeEdit
                {
                    Id = node.Id,
                    ObjectName = node.ObjectName,
                    Position = RobotConversion.UnityToRobot(node.Position),
                    Extents = RobotConversion.UnityToRobot(node.Extents),
                    Category = node.Category,
                    Room = node.Room,
                    Operation = operation,
                    Receptacle = receptacle
                }
            };
        }

        public void GetCurrentWorldGraph()
        {
            SendMessage(WorldGraphMessageHandler.TYPE, WorldGraphMessageHandler.WORLD_GRAPH,
                new WorldGraphDTO { Graph = _robotGraph });
        }

        public void ProcessItemRemoved(string id)
        {
            var node = _worldGraph.Nodes.FirstOrDefault(i => i.Id == id);
            var data = GetBasicEditData(node, WorldGraphOperation.NODE_REMOVED);
            SendMessage(WorldGraphMessageHandler.TYPE, WorldGraphMessageHandler.EDIT, data);
        }

        public void ProcessItemAdded(string id, string receptacleId, string ObjectName, Vector3 position,
            Vector3 extents, string category, string room)
        {
            var data = new WorldGraphNodeEditDTO
            {
                Node = new WorldGraphNodeEdit
                {
                    Id = id,
                    ObjectName = ObjectName,
                    Position = RobotConversion.UnityToRobot(position),
                    Extents = RobotConversion.UnityToRobot(extents),
                    Category = category,
                    Room = room,
                    Operation = WorldGraphOperation.NODE_ADDED,
                    Receptacle = receptacleId
                }
            };
            SendMessage(WorldGraphMessageHandler.TYPE, WorldGraphMessageHandler.EDIT, data);
        }
    }
}