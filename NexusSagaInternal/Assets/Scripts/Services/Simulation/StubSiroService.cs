using System;
using System.Collections.Generic;
using Data;
using Multiplayer.Runtime.Data;
using Multiplayer.Runtime.Messages.Client;
using Multiplayer.Runtime.Messages.RelayMessage;
using Multiplayer.Runtime.Messages.Server;
using Multiplayer.Runtime.Services;
using Newtonsoft.Json;
using Robot.Runtime.Config;
using Robot.Runtime.Data;
using Robot.Runtime.Data.WorldGraph;
using Robot.Runtime.Messages.Client;
using Robot.Runtime.Messages.Server;
using SiroComms.Runtime.Data;
using SiroComms.Runtime.Messages.Client;
using SiroComms.Runtime.Messages.Server;
using Speech.Messages.Client;
using Speech.ServiceHandlers;
using UnityEngine;

namespace SiroComms.Runtime.Services
{
    public class StubSiroService : BaseSiroService
    {
        private readonly AppData _appData;
        private StubPlannerService _plannerService;
        private StubSkillsService _skillsService;
        private StubHumanActivityService _humanActivityService;
        private StubWorldGraphService _worldGraphService;
        private Dictionary<string, Waypoint> _waypoints;
        private Dictionary<string, WorldGraphNode> _nodes;
        private WorldGraph _worldGraph;
        private bool _isUseColocationAnchorStub = false;
        private bool _isUseRobotAnchorStub = false;

        private SpatialAnchorData _colocationAnchorStub = new SpatialAnchorData
        {
            AnchorType = SpatialAnchorType.CoLocation,
            Name = "Co-location",
            State = SpatialAnchorState.Ready,
            Room = "siro",
            SharedWith = new List<ulong>(),
            UUID = Guid.NewGuid()
        };

        private SpatialAnchorData _robotAnchorStub = new SpatialAnchorData
        {
            AnchorType = SpatialAnchorType.RobotHome,
            Name = "robot home",
            State = SpatialAnchorState.Ready,
            Room = "siro",
            SharedWith = new List<ulong>(),
            UUID = Guid.NewGuid()
        };

        public StubSiroService(AppData appData)
        {
            _appData = appData;
        }

        public override void Start(SiroServiceData data)
        {
            ProcessWaypointConfig();
            ProcessWorldGraphConfig();
            var robotData = _appData.RobotData;
            _skillsService = new StubSkillsService(_messages, robotData.Display.transform, robotData.HomeContainer);
            _worldGraphService = new StubWorldGraphService(_messages, _worldGraph);
            _humanActivityService = new StubHumanActivityService(_messages);
            _plannerService = new StubPlannerService(_messages, _skillsService, _worldGraphService, _waypoints, _nodes);
            _data = data;
            _data.ConnectionError = null;
            _data.ServerConnectionState = ConnectionStatus.Connecting;
            TriggerServiceConnectedEvent();
            FakeReceiveMessage(new InitialStateMessage
            {
                EventName = MultiplayerMessageHandler.Type, EventType = InitialStateMessage.Type,
                SID = Guid.NewGuid().ToString(), list = new[] { "siro" },
                Config = new FeatureConfiguration
                {
                    IsNotificationsActive = true, 
                    IsShowSemanticPlace = true, 
                    IsShowWorldObjectHighlights = true, 
                    IsShowPlaceObjectHighlight = true,
                    IsShowTargetObjectHighlight = true,
                    IsWorldGraph2DSurfacesEnabled = true,
                    HARConfig = 1
                }
            });
        }

        private void ProcessWorldGraphConfig()
        {
            var config = Resources.Load<WorldGraphConfig>("Stub World Graph Config");
            if (config.Graph != null)
            {
                var graph = new WorldGraph();
                graph.Nodes = new List<WorldGraphNode>(config.Graph.Nodes.Count);
                _nodes = new Dictionary<string, WorldGraphNode>(config.Graph.Nodes.Count);
                graph.Edges = new List<WorldGraphEdge>(config.Graph.Edges.Count);
                foreach (var node in config.Graph.Nodes)
                {
                    var copy = new WorldGraphNode
                    {
                        Id = node.Id,
                        ObjectName = node.ObjectName,
                        Category = node.Category,
                        Extents = node.Extents,
                        Position = node.Position,
                        Room = node.Room
                    };
                    _nodes.Add(node.Id, copy);
                    graph.Nodes.Add(copy);
                }

                foreach (var edge in config.Graph.Edges)
                {
                    var copy = new WorldGraphEdge
                    {
                        SourceId = edge.SourceId,
                        TargetId = edge.TargetId,
                        Relationship = edge.Relationship
                    };
                    graph.Edges.Add(copy);
                }

                _worldGraph = graph;
            }
        }

        private void ProcessWaypointConfig()
        {
            var config = Resources.Load<WaypointConfig>("StubWaypointConfig");
            if (config.Waypoints != null)
            {
                _waypoints = new Dictionary<string, Waypoint>(config.Waypoints.Count);
                foreach (var waypoint in config.Waypoints)
                {
                    _waypoints.Add(waypoint.Name, waypoint);
                }
            }
        }

        private void FakeReceiveMessage(ServerMessage message)
        {
            _messages.Enqueue(JsonConvert.SerializeObject(message));
        }

        public override void SendMessage(string messageType, IClientMessage message)
        {
            if (messageType != SyncedEntityMessage.Type) Debug.Log($"SendMessage {messageType}");
            switch (messageType)
            {
                case ServiceEvents.SpeechToText:
                    SendSTTConfirmationMessage(message);
                    break;
                case SyncedEntityMessage.Type:
                    RelaySyncedEntityMessage(message as SyncedEntityMessage);
                    break;
                case ActivateFeatureMessage.Type:
                    RelayActivateFeatureMessage(message as ActivateFeatureMessage);
                    break;
                case DeactivateFeatureMessage.Type:
                    RelayDeactivateFeatureMessage(message as DeactivateFeatureMessage);
                    break;
                case JoinRoomMessage.Type:
                    SendJoinRoomResponse(message);
                    break;
                case ServiceEvents.Instruction:
                    _plannerService.PlaybackGoldenPath();
                    break;
                case ServiceEvents.Get_World_Graph:
                    _worldGraphService.GetCurrentWorldGraph();
                    break;
            }
        }

        private void SendSTTConfirmationMessage(IClientMessage clientMessage)
        {
            var sttMessage = clientMessage as SttRequestMessage;
            if (sttMessage == null) return;
            var data = sttMessage.data;
            var serverMessage = new ServerMessage<SttResultMessage>
            {
                EventName = SpeechMessageHandler.TYPE,
                EventType = SpeechMessageHandler.SPEECH_TO_TEXT_RESULT,
                Data = new SttResultMessage
                    { Id = data.id, Text = "This is some stub text", UserId = _appData.UserIdString }
            };

            _messages.Enqueue(JsonConvert.SerializeObject(serverMessage));
        }

        private void RelaySyncedEntityMessage(SyncedEntityMessage message)
        {
            var data = new SyncedEntityDTO();
            data.Position = message.Position;
            data.Scale = message.Scale;
            data.Room = message.Room;
            data.Rotation = message.Rotation;
            data.EntityId = message.EntityId;
            var serverMessage = new ServerMessage<SyncedEntityDTO>();
            serverMessage.Data = data;
            serverMessage.EventName = MultiplayerMessageHandler.Type;
            serverMessage.EventType = SyncedEntityMessage.Type;

            _messages.Enqueue(JsonConvert.SerializeObject(serverMessage));
        }

        private void RelayActivateFeatureMessage(ActivateFeatureMessage message)
        {
            RelayFeatureMessage(message.Feature, ActivateFeatureMessage.Type);
        }

        private void RelayDeactivateFeatureMessage(DeactivateFeatureMessage message)
        {
            RelayFeatureMessage(message.Feature, DeactivateFeatureMessage.Type);
        }

        private void RelayFeatureMessage(string feature, string messageType)
        {
            var serverMessage = new ServerMessage<FeatureUpdateDTO>
            {
                Data = new FeatureUpdateDTO
                {
                    Feature = feature
                },
                EventName = MultiplayerMessageHandler.Type,
                EventType = messageType
            };

            _messages.Enqueue(JsonConvert.SerializeObject(serverMessage));
        }

        private void SendJoinRoomResponse(IClientMessage message)
        {
            if (message is not JoinRoomMessage clientMessage) return;
            if (clientMessage.user.IsHost())
            {
                _colocationAnchorStub.CreatedBy = clientMessage.user.Id;
                _robotAnchorStub.CreatedBy = clientMessage.user.Id;
            }
            else
            {
                _colocationAnchorStub.SharedWith.Add(clientMessage.user.Id);
                _robotAnchorStub.SharedWith.Add(clientMessage.user.Id);
            }

            var roomStateMessage = new RoomStateMessage
            {
                Room = new Room
                {
                    Id = clientMessage.room, Users = new List<User>(), IsAugmentFeatureAvailable = true,
                    IsRobotFeatureAvailable = true,
                    ActiveFeatures = new List<string>()
                    {
                    },
                    ColocationAnchor = _isUseColocationAnchorStub ? _colocationAnchorStub : null,
                    RobotHomeAnchor = _isUseColocationAnchorStub && _isUseRobotAnchorStub ? _robotAnchorStub : null,
                },
                EventName = MultiplayerMessageHandler.Type,
                EventType = RoomStateMessage.Type
            };
            _messages.Enqueue(JsonConvert.SerializeObject(roomStateMessage));
        }

        public override void SendRequest(string eventName)
        {
            switch (eventName)
            {
                case ServiceEvents.Get_Next_Action:
                    _plannerService.GetCurrentAction();
                    break;
                case ServiceEvents.Get_Skill_Feedback: break;
                case ServiceEvents.Get_World_Graph:
                    _worldGraphService.GetCurrentWorldGraph();
                    break;
                case ServiceEvents.Get_Human_Activity: break;
                case ServiceEvents.Get_Default_Instructions:
                    GetDefaultInstructions();
                    break;
                case ServiceEvents.Cancel:
                    _plannerService.Cancel();
                    break;
            }
        }

        public void GetDefaultInstructions()
        {
            var instructions = new List<string>
            {
                "Pick up the object",
                "Place the object in the receptacle",
                "Close the door"
            };
            var message = new ServerMessage<DefaultInstructionsMessage>
            {
                EventName = "siro",
                EventType = DefaultInstructionsMessage.TYPE,
                Data = new DefaultInstructionsMessage { Instructions = instructions }
            };
            _messages.Enqueue(JsonConvert.SerializeObject(message));
        }

        public override void Update()
        {
            base.Update();

            _skillsService?.Tick();
            _plannerService?.Tick();
            _humanActivityService?.Tick();
        }
    }
}