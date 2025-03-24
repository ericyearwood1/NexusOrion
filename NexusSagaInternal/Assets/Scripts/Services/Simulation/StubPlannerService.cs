using System.Collections.Generic;
using System.Threading.Tasks;
using Robot.Runtime.Data;
using Robot.Runtime.Data.Planner;
using Robot.Runtime.Data.WorldGraph;
using Robot.Runtime.Messages.Server.Planner;
using Robot.Runtime.ServiceHandlers;
using SiroComms.Runtime.Messages.Server;
using UnityEngine;

namespace SiroComms.Runtime.Services
{
    public class StubPlannerService : StubService
    {
        private readonly StubSkillsService _skillsService;

        // items
        private const string CAN = "can[1]";
        private const string BIN = "trash_can[1]";

        // receptacles
        private const string TABLE_1 = "coffee_table[1]";
        private const string TABLE_2 = "coffee_table[2]";
        private const string DONUT = "donut[1]";
        private const string DRAWER = "drawer[1]";
        private const string DRAWER_HANDLE = "drawer_handle[1]";
        private const string NEAR_DRAWER = "near_drawer[1]";
        private const string FLOOR = "floor[1]";

        private string _currentAction = string.Empty;
        private Dictionary<string, Waypoint> _waypoints;
        private readonly Dictionary<string, WorldGraphNode> _nodes;
        private readonly StubWorldGraphService _worldGraphService;

        private bool _isCancelled;
        private bool _isTestFailState;

        public StubPlannerService(Queue<string> messageQueue, StubSkillsService skillsService, StubWorldGraphService
                worldGraphService,
            Dictionary<string, Waypoint> waypoints, Dictionary<string, WorldGraphNode> worldGraphNodes) : base(
            messageQueue)
        {
            _skillsService = skillsService;
            _worldGraphService = worldGraphService;
            _waypoints = waypoints;
            _nodes = worldGraphNodes;
        }

        public async void PlaybackGoldenPath()
        {
            if (_isTestFailState)
            {
                SendMessage(PlannerMessageHandler.Type, PlannerMessageHandler.INSTRUCTION_FAILED, new ServerMessage());
                return;
            }
            _isCancelled = false;
            if (_skillsService.IsRobotActive)
            {
                SendMessage(PlannerMessageHandler.Type, PlannerMessageHandler.PLANNER_BUSY, new ServerMessage());
            }
            await Task.Delay(2000);
            await CleanUpDonut();
            SendMessage(PlannerMessageHandler.Type, PlannerMessageHandler.REPLANNING, new ServerMessage());
            await Task.Delay(2000);
            await CleanUpCans();
            _worldGraphService.ProcessItemRemoved(DONUT);
            // await RearrangeStuff();
            SendMessage(PlannerMessageHandler.Type, PlannerMessageHandler.INSTRUCTION_COMPLETE, new ServerMessage());
            // SendMessage(PlannerMessageHandler.Type, PlannerMessageHandler.INSTRUCTION_FAILED, new ServerMessage());
            _currentAction = string.Empty;
        }

        private async Task CleanUpDonut()
        {
            await NavigateTo("Navigate[donut_on_table]", TABLE_1, _waypoints[TABLE_1]);
            await PickUpItem("Pick[donut_on_table]", DONUT, _nodes[DONUT].Position, 4, false, false,
                "Picking up donut from table.");
            await NavigateTo("Navigate[bin]", BIN, _waypoints[BIN]);
            await PlaceItem("Place[donut_in_bin]", DONUT, BIN, _nodes[BIN].Position,
                targetEuler: new Vector3(90, 0, 0));
        }

        private async Task CleanUpCans()
        {
            await NavigateTo("Navigate[cans_on_table]", TABLE_2, _waypoints[TABLE_2]);
            await PickUpItem("Pick[can]", CAN, _nodes[CAN].Position, 3);
            await NavigateTo("Navigate[drawer_open_position]", NEAR_DRAWER, _waypoints[DRAWER]);
            await PlaceItem("Place[can_on_floor]", CAN, FLOOR, _waypoints[NEAR_DRAWER].Get3DPosition(),
                targetEuler: new Vector3(0, 90, 0));
            await NavigateTo("Navigate[drawer]", DRAWER, _waypoints[DRAWER]);
            await OpenDrawer("Open[drawer]", _nodes[DRAWER_HANDLE].Position);
            await NavigateTo("Navigate[can_location]", NEAR_DRAWER, _waypoints[NEAR_DRAWER]);
            await PickUpItem("Pick[can_on_floor]", CAN, _waypoints[NEAR_DRAWER].Get3DPosition(), 1);
            await NavigateTo("Navigate[drawer]", DRAWER, _waypoints[DRAWER]);
            await PlaceItem("Place[can_in_drawer]", CAN, DRAWER, _nodes[DRAWER].Position,
                targetEuler: new Vector3(45, 45, 90));
            await CloseDrawer("Close[drawer]", _nodes[DRAWER_HANDLE].Position);
        }

        private async Task RearrangeStuff()
        {
            await RearrangeCompoundAction("Rearrange[Cup]");
        }

        private Vector3 WaypointTo3DPoint(Vector2 position)
        {
            return new Vector3(position.x, 0, position.y);
        }

        private async Task OpenDrawer(string actionId, Vector3 drawerPosition, bool isReplanning = false,
            bool isChildAction = false)
        {
            if (_isCancelled) return;
            Debug.Log(drawerPosition);
            SendNextActionMessage(actionId, isReplanning, isChildAction);
            await _skillsService.OpenDrawer(actionId, "drawer", drawerPosition);
            SendActionFeedbackMessage(actionId);
        }

        private async Task CloseDrawer(string actionId, Vector3 drawerPosition, bool isReplanning = false,
            bool isChildAction = false)
        {
            if (_isCancelled) return;
            SendNextActionMessage(actionId, isReplanning, isChildAction);
            await _skillsService.CloseDrawer(actionId, "drawer", drawerPosition);
            SendActionFeedbackMessage(actionId);
        }

        private async Task PickUpItem(string actionId, string itemId, Vector3 position, int numberOptions,
            bool isReplanning = false, bool isChildAction = false, string displayMessage = null)
        {
            if (_isCancelled) return;
            SendNextActionMessage(actionId, isReplanning, isChildAction, displayMessage);
            await _skillsService.PickUpItem(actionId, itemId, position, numberOptions);
            _worldGraphService.ProcessItemPickedUp(itemId);
            SendActionFeedbackMessage(actionId);
        }

        private async Task NavigateTo(string actionId, string label, Waypoint target, bool isReplanning = false,
            bool isChildAction = false)
        {
            if (_isCancelled) return;
            SendNextActionMessage(actionId, isReplanning, isChildAction);
            await _skillsService.NavigateTo(actionId, label, target);
            SendActionFeedbackMessage(actionId);
        }

        private async Task PlaceItem(string actionId, string itemId, string receptacleId, Vector3 position,
            bool isReplanning = false,
            bool isChildAction = false, Quaternion targetOrientation = default, Vector3 targetEuler = default)
        {
            if (_isCancelled) return;
            Debug.Log($"StubPlannerService::PlaceItem: {itemId} |  {position}");
            SendNextActionMessage(actionId, isReplanning, isChildAction);
            await _skillsService.PlaceItem(actionId, position, targetOrientation, targetEuler);
            _worldGraphService.ProcessItemPlaced(itemId, receptacleId, position);
            SendActionFeedbackMessage(actionId);
        }

        private async Task RearrangeCompoundAction(string actionId, bool isReplanning = false)
        {
            SendCompoundNextActionMessage(actionId,
                new[]
                {
                    "Navigate[cans_on_table]", "Pick[can]", "Navigate[drawer_open_position]", "Place[can_on_floor]"
                }, isReplanning);
            await NavigateTo("Navigate[cans_on_table]", TABLE_2, _waypoints[TABLE_2], false, true);
            await PickUpItem("Pick[can]", CAN, _nodes[CAN].Position, 3, false, true);
            await NavigateTo("Navigate[drawer_open_position]", NEAR_DRAWER, _waypoints[DRAWER], false, true);
            await PlaceItem("Place[can_on_floor]", CAN, FLOOR, _waypoints[NEAR_DRAWER].Get3DPosition(), false, true);
            SendActionFeedbackMessage(actionId);
        }

        private void SendCompoundNextActionMessage(string actionId, string[] childActionNames,
            bool isReplanning = false)
        {
            var childActions = new List<ServerMessage<NextActionDTO>>();
            foreach (var actionName in childActionNames)
            {
                childActions.Add(new ServerMessage<NextActionDTO>
                {
                    Data = new NextActionDTO
                    {
                        Action = actionName,
                        IsReplanned = isReplanning
                    }
                });
            }

            Debug.Log($"SendNextActionMessage {actionId}");
            var data = new NextActionDTO
            {
                Action = actionId,
                IsReplanned = isReplanning,
                ChildActions = childActions.ToArray()
            };
            _currentAction = actionId;
            SendMessage(PlannerMessageHandler.Type, PlannerMessageHandler.NEXT_ACTION, data);
        }

        private void SendNextActionMessage(string actionId, bool isReplanning = false, bool isChildAction = false,
            string displayMessage = null)
        {
            if (isChildAction) return; // only send feedback for child actions
            var data = new NextActionDTO
            {
                Action = actionId,
                IsReplanned = isReplanning,
                DisplayMessage = displayMessage
            };
            _currentAction = actionId;
            SendMessage(PlannerMessageHandler.Type, PlannerMessageHandler.NEXT_ACTION, data);
        }

        private void SendActionFeedbackMessage(string actionId)
        {
            var data = new ActionFeedbackMessage
            {
                Action = actionId,
                Feedback = "feedback",
                State = PlannerActionState.Complete
            };
            SendMessage(PlannerMessageHandler.Type, PlannerMessageHandler.ACTION_FEEDBACK, data);
        }

        public void GetCurrentAction()
        {
            SendNextActionMessage(_currentAction);
        }

        public void Cancel()
        {
            _isCancelled = true;
            SendMessage(PlannerMessageHandler.Type, PlannerMessageHandler.STATUS,
                new StatusMessage { Status = PlannerMessageHandler.STATUS_CANCELLED });
        }
    }
}