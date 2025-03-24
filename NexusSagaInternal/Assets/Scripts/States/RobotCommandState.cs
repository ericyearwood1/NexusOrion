using System;
using System.Collections.Generic;
using Data;
using JetBrains.Annotations;
using Multiplayer.Runtime.Messages.RelayMessage;
using Notifications.Runtime.Data;
using Prime31.StateKit;
using Robot.Runtime.Data;
using Robot.Runtime.Data.Planner;
using Robot.Runtime.Data.Robot;
using Robot.Runtime.Messages.Client;
using Robot.Runtime.Messages.Server.HumanActivity;
using Robot.Runtime.Messages.Server.Planner;
using Robot.Runtime.View;
using Speech.Data;
using Speech.Messages.Client;
using Speech.Runtime;
using UnityEngine;
using View;

namespace States
{
    public class RobotCommandState : SKState<AppData>
    {
        private const float MIN_RECORDING_TIME_SECONDS = 1f;
        private const float MAX_RECORDING_TIME_SECONDS = 15f;
        private const float SPEED = 45f;

        private Transform _robotTransform;
        private Transform _eePositionTransform;
        private Transform _eeOrientationTransform;
        private RobotOverlayView _robotUI;
        private RobotData _data;
        private RobotDisplay _display;
        private RecordingState _recordingState;
        private RecordInstructionPanel _recordInstructionPanel;
        private ManualControlsPanel _manualControlsPanel;
        private float _currentRecordingTime;
        private bool _isAudioInstructionsEnabled;
        private AudioClip _audioClip;
        private string DeviceName => Microphone.devices[0];

        [CanBeNull] private string _instruction = null;
        private string _asrResult;

        private Dictionary<string, double> _speechToTextSendTimes = new();
        private bool _recordingFlag;
        private double? _instructionSendTime;

        public Action<string, double> RoundTripTime;
        private GripperState _lastState;
        private GripperState _currentGripperState;
        private RecordingState _lastRecordingStateChange = RecordingState.Hack;

        public override void begin()
        {
            base.begin();
            _context.DebugUI.Show(_context.UseDebugUI);
            _isAudioInstructionsEnabled = _context.UseSpeechRecognition;
            _data = _context.RobotData;
            InitialiseRobotDisplay();
            InitialiseRobotMessageHandlers();
            InitializeSpeechMessageHandlers();
            RequestCurrentSystemState();
            _recordInstructionPanel = _context.LeftHandUI.ShowRecordInstructionPanel();
            _recordInstructionPanel.ShowTimings = _context.UseDebugUI;
            _manualControlsPanel = _context.LeftHandUI.ManualControlsPanel;
            PreInitializeMicrophone();
            _recordInstructionPanel.Show();
            _context.LeftHandUI.OnCGVisualisationChange += OnCGVisualisationChangeRequest;
            _context.LeftHandUI.OnManualSTTChange += OnManualSTTRequest;
            _context.LeftHandUI.Enable();
            _context.FullFocusCanvasUI.HideAll();
            InitializeRecordPanelListeners();
            _recordingState = RecordingState.Ready;
            _recordInstructionPanel.SetState(_recordingState);
            _context.FeatureActivationHandler.Enable();
        }

        private void PreInitializeMicrophone()
        {
            if (Microphone.devices.Length > 0)
            {
                Microphone.Start(DeviceName, false, 1, 48000);
                Microphone.End(DeviceName);
            }
        }

        private void OnCGVisualisationChangeRequest(bool isOn)
        {
            SetFeatureChangeRequest(AppFeatures.WORLD_VISUALISATION, isOn);
        }

        private void OnManualSTTRequest(bool isOn)
        {
            SetFeatureChangeRequest(AppFeatures.MANUAL_STT_INSTRUCTION_PANEL, isOn);
        }

        private void SetFeatureChangeRequest(string feature, bool isOn)
        {
            if (isOn)
            {
                _context.SiroService.SendMessage(ActivateFeatureMessage.Type,
                    new ActivateFeatureMessage { Feature = feature, Room = _context.MultiplayerData.Room.Id });
            }
            else
            {
                _context.SiroService.SendMessage(DeactivateFeatureMessage.Type,
                    new DeactivateFeatureMessage
                    {
                        Feature = feature, Room = _context.MultiplayerData.Room.Id
                    });
            }
        }

        public override void end()
        {
            RemoveListeners();
            RemoveMarkers();
            base.end();
        }

        private void RemoveMarkers()
        {
            _context.NavigationHighlightSystem.HideAllMarkers();
            _context.DetectingObjectsHighlightSystem.HideAllMarkers();
            _context.PlaceLocationHighlightSystem.HideAllMarkers();
            _context.TargetObjectHighlightSystem.HideAllMarkers();
        }

        private void InitializeRecordPanelListeners()
        {
            _recordInstructionPanel.OnRecordButtonClicked += SetRecordingFlag;
            _recordInstructionPanel.CancelButtonClicked += RejectTranscription;
            _manualControlsPanel.StopButtonClicked += CancelPlan;
            _recordInstructionPanel.ConfirmButtonClicked += AcceptTranscription;
            _recordInstructionPanel.TryAgainButtonClicked += ResetRecording;
        }

        private void RequestCurrentSystemState()
        {
            _context.SiroService.SendRequest(ServiceEvents.Get_Next_Action);
            _context.SiroService.SendRequest(ServiceEvents.Get_Skill_Feedback);
            _context.SiroService.SendRequest(ServiceEvents.Get_Human_Activity);
            _context.SiroService.SendRequest(ServiceEvents.Get_Default_Instructions);
        }

        private void InitialiseRobotDisplay()
        {
            _display = _data.Display;
            _display.Show();
            _robotTransform = _display.transform;
            _eePositionTransform = _display.EEPositionTransform;
            _eeOrientationTransform = _display.EEOrientationTransform;
            _robotUI = _display.RobotUI;
            _robotUI.SetNextActionInfo(_context.PlannerData);
        }

        private void OnSpeechToTextResult(SttResultMessage obj)
        {
            if (obj.UserId != _context.UserIdString) return;
            
            _asrResult = obj.Text;
            _recordingState = RecordingState.Transcribed;
            _recordInstructionPanel.SetState(_recordingState);
            _manualControlsPanel.SetRecordingState(_recordingState);
            _recordInstructionPanel.SetTranscription(_asrResult);

            if (obj.Id == null) return;
            var time = Time.time - _speechToTextSendTimes[obj.Id];
            _recordInstructionPanel.SetTimings($"STT: {time:0.00}s");
        }

        private void OnTextToSpeechResult(TtsResultMessage obj)
        {
            // Debug.Log($"TTS Result: {obj.Audio}");
            // Base64 decode audio
            var audio = Convert.FromBase64String(obj.Audio);
            var audioClip = WavUtility.ToAudioClip(audio, 0, "tts");
        }

        private void InitialiseRobotMessageHandlers()
        {
            _context.PlannerHandler.OnStateChanged += OnPlannerStateChanged;
            _context.PlannerHandler.OnNextActionReceived += OnNextActionReceived;
            _context.PlannerHandler.OnActionFeedbackUpdate += OnActionFeedbackUpdate;
            _context.PlannerHandler.OnInstructionComplete += OnInstructionComplete;
            _context.PlannerHandler.OnInstructionReceived += OnInstructionReceived;
            _context.PlannerHandler.OnServiceUnavailable += OnPlanningServiceUnavailable;
            _context.PlannerHandler.OnPlannerBusy += OnPlannerBusy;
            _context.PlannerHandler.OnInstructionNotUnderstood += OnInstructionNotUnderstood;
            _context.PlannerHandler.OnInstructionFailed += OnInstructionFailed;
            _context.PlannerHandler.OnReplanning += OnReplanning;
            _context.PlannerHandler.OnCancelling += OnCancelling;
            _context.PlannerHandler.OnCancelled += OnCancelled;

            _context.SkillsMessageHandler.OnNavigationHighlightReceived += OnNavigationHighlightReceived;
            _context.SkillsMessageHandler.OnSelectingTargetsReceived += OnSelectingTargetsReceived;
            _context.SkillsMessageHandler.OnTargetFound += OnTargetFound;
            _context.SkillsMessageHandler.OnGripperStateReceived += OnGripperStateReceived;
            _context.SkillsMessageHandler.OnPlaceLocationReceived += OnPlaceLocationReceived;
            //Initializing HAR Planner Listener
            _context.PlannerHandler.OnHumanActivityPlanner += OnHumanActivityPlanner;

            // _context.HumanActivityMessageHandler.OnHumanActivity += OnHumanActivity;
            _context.SiroMessageHandler.OnDefaultInstructionsReceived += OnDefaultInstructionsReceived;

            _context.RobotHomeUI.defaultInstructionsPanel.OnInstructionSent += OnManualInstructionSent;
            _context.LeftHandUI.RecordInstructionPanel.DefaultInstructionsPanel.OnInstructionSent += OnManualInstructionSent;

            _context.MultiplayerHandler.OnActivateFeature += OnActiveFeature;
            _context.MultiplayerHandler.OnDeactivateFeature += OnDeactivateFeature;
        }

        private void OnDeactivateFeature(string feature)
        {
            _context.LeftHandUI.UpdateManualUIPanel(feature, false);
        }

        private void OnActiveFeature(string feature)
        {
            _context.LeftHandUI.UpdateManualUIPanel(feature, true);
        }

        private void RemoveListeners()
        {
            _context.PlannerHandler.OnStateChanged -= OnPlannerStateChanged;
            _context.PlannerHandler.OnNextActionReceived -= OnNextActionReceived;
            _context.PlannerHandler.OnActionFeedbackUpdate -= OnActionFeedbackUpdate;
            _context.PlannerHandler.OnInstructionReceived -= OnInstructionReceived;
            _context.PlannerHandler.OnInstructionComplete -= OnInstructionComplete;
            _context.PlannerHandler.OnServiceUnavailable -= OnPlanningServiceUnavailable;
            _context.PlannerHandler.OnPlannerBusy -= OnPlannerBusy;
            _context.PlannerHandler.OnInstructionNotUnderstood -= OnInstructionNotUnderstood;
            _context.PlannerHandler.OnReplanning -= OnReplanning;
            _context.PlannerHandler.OnInstructionFailed -= OnInstructionFailed;
            _context.PlannerHandler.OnCancelling -= OnCancelling;
            _context.PlannerHandler.OnCancelled -= OnCancelled;

            _context.SkillsMessageHandler.OnNavigationHighlightReceived -= OnNavigationHighlightReceived;
            _context.SkillsMessageHandler.OnSelectingTargetsReceived -= OnSelectingTargetsReceived;
            _context.SkillsMessageHandler.OnTargetFound -= OnTargetFound;
            _context.SkillsMessageHandler.OnGripperStateReceived -= OnGripperStateReceived;
            _context.SkillsMessageHandler.OnPlaceLocationReceived -= OnPlaceLocationReceived;
            //Removing HAR Planner Listener
            _context.PlannerHandler.OnHumanActivityPlanner -= OnHumanActivityPlanner;


            // _context.HumanActivityMessageHandler.OnHumanActivity -= OnHumanActivity;

            _context.LeftHandUI.OnCGVisualisationChange -= OnCGVisualisationChangeRequest;
            _context.LeftHandUI.OnManualSTTChange -= OnManualSTTRequest;

            _context.RobotHomeUI.defaultInstructionsPanel.OnInstructionSent -= OnManualInstructionSent;

            _context.SiroMessageHandler.OnDefaultInstructionsReceived -= OnDefaultInstructionsReceived;
            _context.LeftHandUI.RecordInstructionPanel.DefaultInstructionsPanel.OnInstructionSent -= OnManualInstructionSent;
            
            _context.MultiplayerHandler.OnActivateFeature -= OnActiveFeature;

            _recordInstructionPanel.OnRecordButtonClicked -= SetRecordingFlag;
            _recordInstructionPanel.CancelButtonClicked -= RejectTranscription;
            _manualControlsPanel.StopButtonClicked -= CancelPlan;
            _recordInstructionPanel.ConfirmButtonClicked -= AcceptTranscription;
            _recordInstructionPanel.TryAgainButtonClicked -= ResetRecording;
        }

        private void OnManualInstructionSent(string instruction)
        {
            Debug.Log($"HandUITesting::RobotCommandState:OnManualInstructionSent {instruction}");
            _instruction = instruction;
            SendInstruction();
            if (_context.FeatureActivationHandler.IsFeatureActive(AppFeatures.MANUAL_STT_INSTRUCTION_PANEL))
            {
                SetFeatureChangeRequest(AppFeatures.MANUAL_STT_INSTRUCTION_PANEL, false);
            }
        }

        private void OnDefaultInstructionsReceived(List<string> obj)
        {
            _context.RobotHomeUI.SetDefaultInstructions(obj);
            _context.LeftHandUI.RecordInstructionPanel.DefaultInstructionsPanel.SetDefaultInstructions(obj);
        }

        private void OnReplanning()
        {
            _robotUI.ShowReplanning();
            _context.NavigationHighlightSystem.OnActionComplete();
            _context.DetectingObjectsHighlightSystem.OnActionComplete();
            _context.PlaceLocationHighlightSystem.OnActionComplete();
        }

        private void OnCancelling()
        {
            _robotUI.UpdateState(_context.PlannerData);
            _recordingState = RecordingState.Cancelling;
        }

        private void OnCancelled()
        {
            _robotUI.ShowReadyState();
            _recordingState = RecordingState.Ready;
        }

        // This function is used for HAR activity as a Notification
        // private void OnHumanActivity(HumanActivityMessage humanActivityMessage)
        // {
        //     _context.NotificationSystem.ShowNotification(humanActivityMessage.Activity, NotificationType.Info);
        // }

        private void OnHumanActivityPlanner(HumanActionData data)
        {
            Debug.Log($"Sending Planner Data {_context.PlannerData}");
            _robotUI.HAR(_context.PlannerData);
            _context.PlannerData.State = PlannerState.HumanActivity;
        }
        private void OnPlaceLocationReceived(PlaceLocationData highlightData)
        {
            try
            {
                _context.PlaceLocationHighlightSystem.OnHighlightReceived(highlightData);
                var semanticPlaceLocationData = new SemanticPlaceLocationData
                {
                    ActionId = highlightData.ActionId,
                    Position = highlightData.Position,
                    UpOrientation = highlightData.TargetOrientation,
                    TargetEuler = highlightData.TargetEuler
                };

                _context.SemanticPlaceHighlightSystem.OnHighlightReceived(semanticPlaceLocationData);
            }
            catch (Exception e)
            {
                Debug.LogError(e);
            }
        }

        private void OnNavigationHighlightReceived(LabelHighlightData highlightData)
        {
            _context.NavigationHighlightSystem.OnHighlightReceived(highlightData);
        }

        private void OnSelectingTargetsReceived(TargetObjectHighlight highlightData)
        {
            var isOpenOrClose = IsOpenOrClose(_context.PlannerData.CurrentAction);
            if (isOpenOrClose) return;
            _context.DetectingObjectsHighlightSystem.OnHighlightReceived(highlightData);
        }

        private void OnTargetFound(TargetObjectHighlight highlightData)
        {
            _context.DetectingObjectsHighlightSystem.HideAllMarkers();
            _context.TargetObjectHighlightSystem.OnHighlightReceived(highlightData);
        }

        private bool IsOpenOrClose(NextActionData currentAction)
        {
            if (currentAction == null) return false;
            if (currentAction.ActionType == PlannerActionType.Compound)
            {
                var childAction = currentAction.ChildActions[currentAction.ChildActionIndex];
                if (childAction.ActionType is PlannerActionType.Open or PlannerActionType.Close)
                {
                    return true;
                }
            }
            else
            {
                return currentAction.ActionType is PlannerActionType.Open or PlannerActionType.Close;
            }

            return false;
        }

        private void InitializeSpeechMessageHandlers()
        {
            _context.SpeechMessageHandler.OnTtsResult += OnTextToSpeechResult;
        }

        private void OnPlannerStateChanged(PlannerState state)
        {
            _robotUI.UpdateState(_context.PlannerData);
        }

        private void OnGripperStateReceived(GripperState state)
        {
            _lastState = _currentGripperState;
            _currentGripperState = state;

            switch (state)
            {
                case GripperState.Holding:
                    _context.TargetObjectHighlightSystem.ProcessGripperHolding();
                    _context.DetectingObjectsHighlightSystem.HideAllMarkers();
                    break;
                case GripperState.Open:
                    _context.PlaceLocationHighlightSystem.HideAllMarkers();
                    if (_lastState == GripperState.Holding && _currentGripperState == GripperState.Open)
                    {
                        _context.TargetObjectHighlightSystem.HideMarkerWithDelay(2f);
                    }

                    break;
                case GripperState.None:
                    break;
                case GripperState.Closed:
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(state), state, null);
            }
        }

        private void OnInstructionNotUnderstood(string obj)
        {
            _recordingState = RecordingState.Ready;
            _robotUI.ShowInstructionNotUnderstood();
        }

        private void OnInstructionFailed()
        {
            _recordingState = RecordingState.Ready;
            _robotUI.ShowInstructionFailed();
            InstructionFinished();
            _context.NotificationSystem.ShowNotification("Could not complete plan", NotificationType.Error);
        }

        private void InstructionFinished()
        {
            _context.NavigationHighlightSystem.OnInstructionComplete();
            _context.DetectingObjectsHighlightSystem.OnInstructionComplete();
            _context.TargetObjectHighlightSystem.OnInstructionComplete();
            _context.PlaceLocationHighlightSystem.OnInstructionComplete();
        }

        private void OnPlannerBusy(string obj)
        {
            if (_recordingState == RecordingState.Cancelling)
                return;

            _recordingState = RecordingState.None;
        }

        private void OnPlanningServiceUnavailable()
        {
            _recordingState = RecordingState.Error;
        }

        private void OnInstructionComplete()
        {
            _recordingState = RecordingState.Ready;
            _robotUI.ShowInstructionComplete();
            InstructionFinished();
            _context.NotificationSystem.ShowNotification("Plan Completed", NotificationType.Success);
        }

        private void OnInstructionReceived(Instruction instruction)
        {
            _recordInstructionPanel.SetTranscription(instruction.Data);
            ShowInstructionAcceptedState();
        }

        private void OnActionFeedbackUpdate(ActionFeedbackMessage message)
        {
            if (message.State != PlannerActionState.Complete)
            {
                if(message.State == PlannerActionState.Error)
                {
                    _context.TargetObjectHighlightSystem.HideAllMarkers();
                }
                _robotUI.UpdateActionFeedback(message);
            }
            else
            {
                _context.NavigationHighlightSystem.OnActionComplete();
                _context.DetectingObjectsHighlightSystem.OnActionComplete();
                var currentAction = _context.PlannerData.CurrentAction;
                if (currentAction is { ActionType: PlannerActionType.Place })
                {
                    _context.TargetObjectHighlightSystem.OnActionComplete();
                }

                _context.PlaceLocationHighlightSystem.OnActionComplete();
            }
        }

        private void OnNextActionReceived(NextActionDTO dto)
        {
            if (_instructionSendTime != null)
            {
                var roundTripTime = Time.time - _instructionSendTime.Value;
                RoundTripTime?.Invoke("Instruction", roundTripTime);
                if (_context.UseDebugUI)
                    _context.DebugUI.SetDebugText($"Instruction round trip time: {roundTripTime:0.00}s", 5f);
                _instructionSendTime = null;
            }

            if (string.IsNullOrWhiteSpace(dto.Action))
            {
                _robotUI.ShowReadyState();
                _manualControlsPanel.ShowStopButton(false);
                return;
            }
            _manualControlsPanel.ShowStopButton(true);
            _robotUI.UpdateState(_context.PlannerData);
            _recordingState = RecordingState.None;
            
        }

        public override void update(float deltaTime)
        {
            ConfigureRecordingUI();
            SetRobotPose();
            ProcessDebugInput();
        }

        private void ConfigureRecordingUI()
        {
            if (_isAudioInstructionsEnabled)
            {
                CheckForRecording();
                EvaluateRecordingState();
            }
            else
            {
                if (IsSendTextInstruction())
                {
                    _instruction = "Put the cup in the kitchen";
                    SendInstruction();
                }
            }
        }

        private void SendInstruction()
        {
            if (_instruction == null) return;
            _context.PlannerData.State = PlannerState.SendingInstruction;
            _context.SiroService.SendMessage(ServiceEvents.Instruction,
                new InstructionMessage(new Instruction(_instruction, "text", _context.UserIdString)));
            _instructionSendTime = Time.time;
            ShowInstructionAcceptedState();
        }

        private void ShowInstructionAcceptedState()
        {
            _recordingState = RecordingState.None;
            _recordInstructionPanel.SetState(_recordingState);
            _manualControlsPanel.SetRecordingState(_recordingState);
            _robotUI.ShowThinkingState();
            _instruction = null;
        }

        private void SetRobotPose()
        {
            if (!_data.IsTrackRobot) return;
            _robotTransform.localPosition =
                Vector3.SmoothDamp(_robotTransform.localPosition, _data.Position, ref _data.Velocity,
                    Time.deltaTime * SPEED);
            _robotTransform.localRotation = Quaternion.Slerp(_robotTransform.localRotation,
                Quaternion.Euler(_data.Rotation),
                Time.deltaTime * SPEED);

            _eePositionTransform.localPosition =
                Vector3.Lerp(_eePositionTransform.localPosition, _data.EEPosition, Time.deltaTime * SPEED);
            LerpYaw();
            LerpPitch();
            LerpRoll();
        }

        private void LerpRoll()
        {
            var roll = _data.Display.Roll.localEulerAngles;
            roll.z = Mathf.LerpAngle(roll.z, _data.EEYPR.Roll, Time.deltaTime * SPEED);
            _data.Display.Roll.localEulerAngles = roll;
        }

        private void LerpPitch()
        {
            var pitch = _data.Display.Pitch.localEulerAngles;
            pitch.x = Mathf.LerpAngle(pitch.x, _data.EEYPR.Pitch, Time.deltaTime * SPEED);
            _data.Display.Pitch.localEulerAngles = pitch;
        }

        private void LerpYaw()
        {
            var yaw = _data.Display.Yaw.localEulerAngles;
            yaw.y = Mathf.LerpAngle(yaw.y, _data.EEYPR.Yaw, Time.deltaTime * SPEED);
            _data.Display.Yaw.localEulerAngles = yaw;
        }

        private bool IsSendTextInstruction()
        {
            if (_context.PlannerData.State != PlannerState.ReadyForInstruction &&
                _context.PlannerData.State != PlannerState.InstructionComplete) return false;
            return !_isAudioInstructionsEnabled &&
                   (Input.GetKeyDown(KeyCode.Space) || _context.LeftHand.GetIndexFingerIsPinching());
        }

        private void SetRecordingFlag(bool flag)
        {
            switch (_recordingState)
            {
                case RecordingState.Transcribed:
                case RecordingState.AwaitingTranscriptionConfirm:
                    AcceptTranscription();
                    break;
                case RecordingState.Ready:
                case RecordingState.Recording:
                case RecordingState.Started:
                    _recordingFlag = flag;
                    break;
                case RecordingState.Silent:
                case RecordingState.Error:
                    _recordingState = RecordingState.Ready;
                    break;
                case RecordingState.Cancelled:
                    _recordingState = RecordingState.Ready;
                    break;
                case RecordingState.None:
                case RecordingState.Complete:
                case RecordingState.Accepted:
                case RecordingState.Cancelling:
                default:
                    break;
            }
        }

        private void StartRecording()
        {
            if (!_isAudioInstructionsEnabled || _context.IsUseStubService) return;
            _audioClip = Microphone.Start(DeviceName, false, (int)MAX_RECORDING_TIME_SECONDS, 16000);
        }

        private void StopRecording()
        {
            if (!_isAudioInstructionsEnabled || _context.IsUseStubService) return;
            Microphone.End(DeviceName);
        }

        private void CheckForRecording()
        {
            if (_currentRecordingTime > MAX_RECORDING_TIME_SECONDS)
            {
                _recordingFlag = false;
            }
        }

        // @TODO change this so it only runs when needed (not on update)
        private void EvaluateRecordingState()
        {
            // if (_lastRecordingStateChange != _recordingState)
            // {
                _recordInstructionPanel.SetState(_recordingState);
                _manualControlsPanel.SetRecordingState(_recordingState);
            // }
            _lastRecordingStateChange = _recordingState;
            switch (_recordingState)
            {
                case RecordingState.Ready:
                    if (_context.PlannerData.State != PlannerState.ReadyForInstruction) return;
                    if (_recordingFlag)
                    {
                        _currentRecordingTime = 0;
                        _recordingState = RecordingState.Started;
                    }
                    else
                    {
                        // _context.RobotHomeUI.ShowDefaultInstructions();
                    }

                    break;
                case RecordingState.Started:
                    if (!_recordingFlag)
                    {
                        _recordingState = RecordingState.Cancelled;
                        return;
                    }

                    StartRecording();

                    _recordingState = RecordingState.Recording;
                    break;
                case RecordingState.Recording:
                    _currentRecordingTime += Time.deltaTime;
                    _recordInstructionPanel.SetRecordingRatio(_currentRecordingTime / MAX_RECORDING_TIME_SECONDS);
                    if (!_recordingFlag)
                    {
                        StopRecording();
                        if (_currentRecordingTime < MIN_RECORDING_TIME_SECONDS)
                        {
                            _recordingState = RecordingState.Cancelled;
                            return;
                        }

                        _recordingState = RecordingState.Complete;
                        SendSpeechToTextRequest();
                    }

                    break;
                case RecordingState.Cancelled:
                    CleanUpRecording();
                    _recordingState = RecordingState.Ready;
                    break;
                case RecordingState.Transcribed:
                    _recordInstructionPanel.SetTranscription(_asrResult);
                    _recordingState = RecordingState.AwaitingTranscriptionConfirm;
                    break;
                case RecordingState.Accepted:
                    _instruction = _asrResult;
                    _recordingState = RecordingState.None;
                    SendInstruction();
                    break;
                case RecordingState.Complete:
                case RecordingState.None:
                case RecordingState.Error:
                case RecordingState.Silent:
                case RecordingState.AwaitingTranscriptionConfirm:
                case RecordingState.Cancelling:
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }

            
        }

        private void SendSpeechToTextRequest()
        {
            if (SilenceChecker.IsSilent(_audioClip))
            {
                _recordingState = RecordingState.Silent;
                return;
            }

            var id = Guid.NewGuid().ToString();
            string audioBase64 = null;
            if (_context.UseSpeechRecognition && !_context.IsUseStubService)
            {
                var byteData = WavUtility.FromAudioClip(_audioClip);
                audioBase64 = Convert.ToBase64String(byteData);
            }

            _context.SiroService.SendMessage(ServiceEvents.SpeechToText,
                new SttRequestMessage(
                    new SttRequestData
                    {
                        id = id,
                        UserId = _context.UserIdString,
                        Audio = audioBase64,
                        SourceLanguage = "eng",
                        TargetLanguage = "eng"
                    }));
            _speechToTextSendTimes[id] = Time.time;
            _context.SpeechMessageHandler.OnSttResult += OnSpeechToTextResult;
        }

        private void CleanUpRecording()
        {
            _currentRecordingTime = 0;
            _recordInstructionPanel.SetRecordingRatio(0);
        }

        private void AcceptTranscription()
        {
            _recordingState = RecordingState.Accepted;
        }

        private void RejectTranscription()
        {
            _context.SpeechMessageHandler.OnSttResult -= OnSpeechToTextResult;
            _recordingState = RecordingState.Ready;
        }

        private void CancelPlan()
        {
            Debug.Log("CancelPlan");
            _recordingState = RecordingState.Cancelling;
            _context.SiroService.SendRequest(ServiceEvents.Cancel);
        }

        private void ResetRecording()
        {
            _recordingState = RecordingState.Ready;
        }

        private void ProcessDebugInput()
        {
            if (Input.GetKeyDown(KeyCode.O))
            {
                OnCGVisualisationChangeRequest(true);
            }
            else if (Input.GetKeyDown(KeyCode.P))
            {
                OnCGVisualisationChangeRequest(false);
            }

            // switch (_recordingState)
            // {
            //     case RecordingState.Ready:
            //     case RecordingState.None:
            //     {
            //         if (Input.GetKeyDown(KeyCode.Space))
            //         {
            //             _recordingFlag = true;
            //         }
            //
            //         break;
            //     }
            //    
            //     case RecordingState.Started:
            //     case RecordingState.Recording:
            //     {
            //         if (Input.GetKeyUp(KeyCode.Space))
            //         {
            //             _recordingFlag = false;
            //         }
            //
            //         break;
            //     }
            //     case RecordingState.AwaitingTranscriptionConfirm:
            //     {
            //         if (Input.GetKeyUp(KeyCode.Space))
            //         {
            //             AcceptTranscription();
            //         }
            //         else if (Input.GetKeyUp(KeyCode.X))
            //         {
            //             RejectTranscription();
            //         }
            //
            //         break;
            //     }
            // }
        }
    }
}