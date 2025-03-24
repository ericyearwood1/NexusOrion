using System;
using Robot.Runtime.Data;
using Robot.Runtime.Data.Planner;
using Robot.Runtime.Messages.Server.Planner;
using SiroComms.Runtime.Messages.Server;
using SiroComms.Runtime.Services;
using UnityEngine;

namespace Robot.Runtime.ServiceHandlers
{
    public class PlannerMessageHandler : IMessageHandler
    {
        public const string Type = "planner";
        public const string REPLANNING = "replanning";
        public const string NEXT_ACTION = "next_action";
        public const string ACTION_FEEDBACK = "action_feedback";
        public const string INSTRUCTION_COMPLETE = "instruction_complete";
        public const string SERVICE_UNAVAILABLE = "service_unavailable";
        public const string PLANNER_BUSY = "planner_is_busy";
        public const string INSTRUCTION_NOT_UNDERSTOOD = "instruction_not_understood";
        public const string INSTRUCTION_FAILED = "instruction_failed";
        public const string INSTRUCTION_RECEIVED = "instruction_received";
        public const string STATUS = "status";
        public const string HUMAN_ACTIVITY = "human_activity";
        
        public const string STATUS_CANCELLING = "is_cancelling";
        public const string STATUS_CANCELLED = "is_cancelled";

        public Action<PlannerState> OnStateChanged;
        public Action OnReplanning;
        public Action OnServiceUnavailable;
        public Action OnInstructionComplete;
        public Action OnCancelling;
        public Action OnCancelled;
        public Action<string> OnInstructionNotUnderstood;
        public Action<string> OnPlannerBusy;
        public Action OnInstructionFailed;
        public Action<Instruction> OnInstructionReceived;
        public Action<NextActionDTO> OnNextActionReceived;
        public Action<ActionFeedbackMessage> OnActionFeedbackUpdate;
        public Action<HumanActionData> OnHumanActivityPlanner;
        private readonly PlannerData _data;

        public PlannerMessageHandler(PlannerData data)
        {
            _data = data;
        }

        public void HandleMessage(string eventType, string message)
        {
            switch (eventType)
            {
                case NEXT_ACTION:
                    ProcessNextAction(message);
                    SendStateChanged();
                    break;
                case ACTION_FEEDBACK:
                    ProcessActionFeedback(message);
                    SendStateChanged();
                    break;
                case PLANNER_BUSY:
                    _data.State = PlannerState.PlannerBusy;
                    ProcessPlannerBusy(message);
                    SendStateChanged();
                    break;
                case INSTRUCTION_NOT_UNDERSTOOD:
                    _data.State = PlannerState.ReadyForInstruction;
                    ProcessInstructionNotUnderstood(message);
                    SendStateChanged();
                    break;
                case INSTRUCTION_RECEIVED:
                    ProcessInstructionReceivedMessage(message);
                    break;
                case INSTRUCTION_FAILED:
                    _data.State = PlannerState.ReadyForInstruction;
                    OnInstructionFailed?.Invoke();
                    SendStateChanged();
                    break;
                case INSTRUCTION_COMPLETE:
                    HandleInstructionComplete();
                    SendStateChanged();
                    break;
                case SERVICE_UNAVAILABLE:
                    _data.State = PlannerState.ServiceUnavailable;
                    OnServiceUnavailable?.Invoke();
                    SendStateChanged();
                    break;
                case REPLANNING:
                    if (_data.State == PlannerState.HumanActivity)
                    {
                        Debug.Log("Recieved replanning while displaying HAR. Ignoring replanning slate request");
                        break;
                    }
                    _data.State = PlannerState.Replanning;
                    OnReplanning?.Invoke();
                    SendStateChanged();
                    break;
                case STATUS:
                    ProcessStatus(message);
                    break;
                case HUMAN_ACTIVITY:
                    _data.State = PlannerState.HumanActivity;
                    ProcessHumanActivity(message);
                    SendStateChanged();
                    break;
                    
            }
            SendStateChanged();
        }

        private void ProcessInstructionReceivedMessage(string message)
        {
            Debug.Log($"ProcessInstructionReceivedMessage::{message}");
            var instruction = ServerMessage<Instruction>.Deserialize(message).Data;
            _data.State = PlannerState.ReceivedInstruction;
            OnInstructionReceived?.Invoke(instruction);
            SendStateChanged();
        }

        private void SendStateChanged()
        {
            OnStateChanged?.Invoke(_data.State);
        }

        private void ProcessInstructionNotUnderstood(string message)
        {
            var siroMessage = ServerMessage<InstructionNotUnderstoodMessage>.Deserialize(message).Data;
            if (siroMessage == null)
            {
                Debug.LogError($"Could not process planner busy message {message}");
                return;
            }

            OnInstructionNotUnderstood?.Invoke(siroMessage.Instruction);
        }

        private void ProcessPlannerBusy(string message)
        {
            var siroMessage = ServerMessage<PlannerBusyMessage>.Deserialize(message).Data;
            if (siroMessage == null)
            {
                Debug.LogError($"Could not process planner busy message {message}");
                return;
            }

            OnPlannerBusy?.Invoke(siroMessage.Instruction);
        }

        private void ProcessNextAction(string message)
        {
            var nextActionDTO = ServerMessage<NextActionDTO>.Deserialize(message).Data;
            if (nextActionDTO == null)
            {
                _data.CurrentAction = null;
                Debug.LogError($"Could not process next action message {message}");
                return;
            }
            else{
                Debug.Log("NextActionDTO is not null");
            }
            if (_data.CurrentAction !=null)
            {
                Debug.Log($"THE CURRENT ACTION {_data.CurrentAction.Action} ||| _____ |||");
            }
            _data.State = string.IsNullOrWhiteSpace(nextActionDTO.Action)
                ? PlannerState.ReadyForInstruction
                : PlannerState.PlannerBusy;
            var actionData = new NextActionData
            {
                Action = nextActionDTO.Action,
                IsReplanned = nextActionDTO.IsReplanned,
                ActionType = SetActionType(nextActionDTO),
                State = PlannerActionState.InProgress,
                DisplayMessage = nextActionDTO.DisplayMessage
            };
            Debug.Log($"THE NEXT ACTION {actionData.Action} |||_____ |||");

            if (actionData.ActionType == PlannerActionType.Compound)
            {
                Debug.Log("Current Action Type is Compound");
                actionData.ChildActionIndex = 0;
                foreach (var childAction in nextActionDTO.ChildActions)
                {
                    var data = childAction.Data;
                    var childData = new NextActionData
                    {
                        Action = data.Action,
                        IsReplanned = data.IsReplanned,
                        ActionType = SetActionType(data),
                        DisplayMessage = data.DisplayMessage
                    };
                    // do not support > 1 depth children
                    actionData.ChildActions.Add(childData);    
                }

                actionData.ChildActions[0].State = PlannerActionState.InProgress;
            }

            _data.CurrentAction = actionData;
            OnNextActionReceived?.Invoke(nextActionDTO);
        }

        private PlannerActionType SetActionType(NextActionDTO dto)
        {
            if (dto.ChildActions is { Length: > 0 })
            {
                return PlannerActionType.Compound;
            }

            var action = dto.Action;
            if (string.IsNullOrWhiteSpace(action)) return PlannerActionType.None;
            var actionType = PlannerActionType.None;
            action = action.ToLower();
            if (action.StartsWith("findagentactiontool"))
            {
                actionType = PlannerActionType.FindAgentActionTool;
            }
            else if (action.StartsWith("findobjecttool"))
            {
                actionType = PlannerActionType.FindObjectTool;
            }
            else if (action.StartsWith("nav"))
            {
                actionType = PlannerActionType.Navigate;
            }
            else if (action.StartsWith("pick"))
            {
                actionType = PlannerActionType.Pick;
            }
            else if (action.StartsWith("findreceptacletool"))
            {
                actionType = PlannerActionType.FindReceptacleTool;
            }
            else if (action.StartsWith("place"))
            {
                actionType = PlannerActionType.Place;
            }
            else if (action.StartsWith("open"))
            {
                actionType = PlannerActionType.Open;
            }
            else if (action.StartsWith("close"))
            {
                actionType = PlannerActionType.Close;
            }
            else
            {
                actionType = PlannerActionType.Unknown;  
            }
            

            return actionType;
        }

        private void ProcessActionFeedback(string message)
        {
            var progressMessage = ServerMessage<ActionFeedbackMessage>.Deserialize(message);
            if (progressMessage == null)
            {
                Debug.LogError($"Could not process action progress message {message}");
                return;
            }

            var actionId = progressMessage.Data.Action;
            var currentAction = _data.CurrentAction;
            if (currentAction == null)
            {
                Debug.LogError($"PlannerMessageHandler::Received feedback for an action that doesn't exist {message}");
                return;
            }
            if (actionId == currentAction.Action)
            {
                currentAction.State = progressMessage.Data.State;
                currentAction.Feedback = progressMessage.Data.Feedback;
            }
            else if (currentAction.ActionType == PlannerActionType.Compound)
            {
                foreach (var childAction in currentAction.ChildActions)
                {
                    if (actionId != childAction.Action) continue;
                    childAction.State = progressMessage.Data.State;
                    childAction.Feedback = progressMessage.Data.Feedback;
                    if (childAction.State == PlannerActionState.Complete)
                    {
                        currentAction.ChildActionIndex++;
                        if (currentAction.ChildActionIndex >= currentAction.ChildActions.Count)
                        {
                            currentAction.State = PlannerActionState.Complete;
                        }
                        else
                        {
                            currentAction.ChildActions[currentAction.ChildActionIndex].State =
                                PlannerActionState.InProgress;
                        }
                    }
                }
            }
            OnActionFeedbackUpdate?.Invoke(progressMessage.Data);
        }

        private void HandleInstructionComplete()
        {
            _data.State = PlannerState.InstructionComplete;
            OnInstructionComplete?.Invoke();
            _data.State = PlannerState.ReadyForInstruction;
        }
        
        private void ProcessStatus(string message)
        {
            var statusMessage = ServerMessage<StatusMessage>.Deserialize(message).Data;
            Debug.Log($"Received status message : {message}");
            if (statusMessage == null)
            {
                Debug.LogError($"Could not process planner status message {message}");
                return;
            }

            if (statusMessage.Status == STATUS_CANCELLING)
            {
                _data.State = PlannerState.PlannerBusy;
                OnCancelling?.Invoke();
            }
            else if (statusMessage.Status == STATUS_CANCELLED)
            {
                _data.State = PlannerState.ReadyForInstruction;
                OnCancelled?.Invoke();
            }
        }

        private void ProcessHumanActivity(string message)
        {
            Debug.Log("ProcessHumanActivity called");
            var statusMessage = ServerMessage<PlannerHARMessage>.Deserialize(message).Data;
            Debug.Log($"Deserialized statusMessage: {statusMessage}");
            var actionData = new HumanActionData
            {
                Action = statusMessage.Action,
                IsReplanned = statusMessage.IsReplanned,
                State = PlannerActionState.InProgress,
                DisplayMessage = statusMessage.DisplayMessage
            };
            Debug.Log($"Created actionData: {actionData}");
            _data.humanActionData=actionData;
            OnHumanActivityPlanner?.Invoke(actionData);
            Debug.Log($"OnHumanActivityPlanner invoked with statusMessage: {statusMessage}");
        }
    }
}