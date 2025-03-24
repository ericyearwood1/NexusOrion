using System;
using System.Threading.Tasks;
using Data;
using IX.Saga;
using Meta.XR.MRUtilityKit;
using Multiplayer.Runtime.Config;
using Multiplayer.Runtime.Data;
using Multiplayer.Runtime.Messages.Client;
using Multiplayer.Runtime.Pools;
using Multiplayer.Runtime.Services;
using Multiplayer.Runtime.Systems;
using Multiplayer.Runtime.View;
using Notifications.Runtime.Systems;
using Oculus.Interaction.Input;
using Prime31.StateKit;
using Robot.Runtime.Config;
using Robot.Runtime.Data;
using Robot.Runtime.Data.Planner;
using Robot.Runtime.Data.Robot;
using Robot.Runtime.ServiceHandlers;
using Robot.Runtime.Systems;
using Robot.Runtime.View;
using SiroComms.Runtime.Services;
using Speech.Config;
using Speech.ServiceHandlers;
using States;
using Systems;
using UnityEngine;
using View;
using SagaPluginConfiguration = Multiplayer.Runtime.Data.SagaPluginConfiguration;

public class Root : MonoBehaviour
{
    [Header("Development Settings")] [SerializeField]
    private bool _isTrackRobot = true;

    [SerializeField] private bool _isUseStubSiroService;
    [SerializeField] private bool _useSpatialAnchors = true;
    [SerializeField] private bool _useServiceDiscovery;
    [SerializeField] private bool _useSpeechRecognition;
    [SerializeField] private bool _useDebugUI;
    [SerializeField] private bool _isShowDebugObjects;

    [Header("XR References")] [SerializeField]
    private HandRef _leftHand;

    [SerializeField] private MRUK _mruk;
    [SerializeField] private HandRef _rightHand;
    [SerializeField] private OVRCameraRig _ovrCameraRig;
    [SerializeField] private Transform _rightHandPointerPose;

    [Header("Saga Plugin")] [SerializeField]
    private SagaPlugin _sagaPlugin;

    // force enable will take priority here
    [SerializeField] private SagaPluginConfiguration _sagaPluginConfiguration = SagaPluginConfiguration.Normal;

    [Header("Configs")] [SerializeField]
    private MultiplayerPoolConfig _poolConfig;

    [SerializeField] private SpatialAnchorConfig _editorAnchorConfig;
    [SerializeField] private SpatialAnchorConfig _spatialAnchorConfig;
    [SerializeField] private ServiceConfig _siroServiceConfig;
    [SerializeField] private SpeechConfig _speechConfig;
    [SerializeField] private ColorConfig _colorConfig;
    [SerializeField] private WorldGraphObjectConfig _worldGraphObjectConfig;

    [Header("Prefabs")] [SerializeField]
    private GameObject _headEntityView;

    [SerializeField] private GameObject _pinchMarker;
    [SerializeField] private GameObject _navigationHighlightPrefab;
    [SerializeField] private GameObject _detectingObjectHighlightPrefab;
    [SerializeField] private GameObject _targetObjectHighlightPrefab;
    [SerializeField] private GameObject _placeLocationHighlightPrefab;
    [SerializeField] private GameObject _notificationPrefab;
    [SerializeField] private GameObject _semanticPlacePrefab;

    [Header("World Graph")] [SerializeField]
    private Transform _worldGraphContainer;

    [SerializeField] private GameObject _furnitureHighlightPrefab;
    [SerializeField] private GameObject _objectHighlightPrefab;
    [SerializeField] private GameObject _worldGraphIconPrefab;
    [SerializeField] private GameObject _worldGraphPanelPrefab;
    [SerializeField] private Material _semanticPlaceWorldUpMat;
    [SerializeField] private Material _semanticPlaceTargetMat;

    [Header("UI")] [SerializeField]
    private FullFocusCanvasUI _fullFocusCanvasUI;

    [SerializeField] private RobotHomeUI _robotHomeUI;
    [SerializeField] private HandUI _leftHandUI;
    [SerializeField] private DebugUI _debugUI;

    [Header("Multiplayer")] [SerializeField]
    private Transform _syncedContainer;

    [SerializeField] private bool _isSaveAnchorInEditor;

    [Header("Robot References")] [SerializeField]
    private Transform _cameraTransform;

    [SerializeField] private Transform _highlightContainer;
    [SerializeField] private Transform _robotHomeContainer;
    [SerializeField] private RobotDisplay _robotDisplay;
    [SerializeField] private RectTransform _notificationList;

    private SKStateMachine<AppData> _stateMachine;
    private MultiplayerPools _pools;
    private SpatialAnchorService _spatialAnchorService;
    private NotificationSystem _notificationSystem;
    private AppData _data;
    private Camera _mainCamera;
    private SyncedEntityView _trackedUserPanel;
    
    [Header("Colors")]
    [SerializeField] private Color _worldGraphColor = Color.white;
    [SerializeField] private Color _normalHighlightColor = Color.white;
    [SerializeField] private Color _semanticPlaceTargetArrowColor = Color.green;
    private static readonly int BaseColor = Shader.PropertyToID("_Color");

    private void Awake()
    {
        if (_sagaPlugin != null)
            _sagaPlugin.enabled = false;
        Logger.Runtime.DevLog.Initialise();
        InitialiseData();
        InitialiseNotifications();
    }
    
    private void Start()
    {
        _mainCamera = Camera.main;
        InitialiseServiceData();
        InitialiseSpatialAnchorService();
        InitialiseHandlers();
        InitialiseStateMachine();
        InitialiseHeadSyncedEntity();
        InitialiseHighlightSystems();
    }

    private void Update()
    {
        RunServicesTick();
        RunStateMachine();
        RunHighlightSystems();
        _notificationSystem.Tick();
    }
    
    private void LateUpdate()
    {
        SendSyncedEntityUpdate();
    }

    private void InitialiseNotifications()
    {
        _notificationSystem = new NotificationSystem(50, _notificationList, _notificationPrefab);
        _data.NotificationSystem = _notificationSystem;
    }

    private void InitialiseData()
    {
        _data = new AppData
        {
            MRUK = _mruk,
            UseServiceDiscovery = !_isUseStubSiroService && _useServiceDiscovery,
            UseSpeechRecognition = _useSpeechRecognition,
            UseDebugUI = _useDebugUI,
            IsUseStubService = _isUseStubSiroService,
            LeftHand = _leftHand,
            RightHand = _rightHand,
            RobotData = new RobotData
            {
                HomeContainer = _robotHomeContainer,
                Display = _robotDisplay,
                IsTrackRobot = _isTrackRobot
            },
            WorldGraphData = new WorldGraphData
            {
                FurnitureHighlightPrefab = _furnitureHighlightPrefab,
                ObjectHighlightPrefab = _objectHighlightPrefab,
                IconPrefab = _worldGraphIconPrefab,
                PanelPrefab = _worldGraphPanelPrefab,
                IconConfig = _worldGraphObjectConfig,
                ViewContainer = _worldGraphContainer
            },
            FullFocusCanvasUI = _fullFocusCanvasUI,
            LeftHandUI = _leftHandUI,
            DebugUI = _debugUI,
            RobotHomeUI = _robotHomeUI,
            SiroServiceConfig = _siroServiceConfig,
            SpeechConfig = _speechConfig,
            ColorConfig = _colorConfig,
            SpatialAnchorConfig = _spatialAnchorConfig,
            CameraTransform = _ovrCameraRig.centerEyeAnchor,
            SagaPlugin = _sagaPlugin,
            IsUseSpatialAnchors = _useSpatialAnchors,
            OVRCameraRig = _ovrCameraRig,
            RightHandPointerPose = _rightHandPointerPose,
            PinchMarker = _pinchMarker
        };
        if (!_useSpatialAnchors || Application.isEditor)
        {
            _data.SpatialAnchorConfig = _editorAnchorConfig;
        }
        _robotDisplay.Initialise(_cameraTransform, _isShowDebugObjects);
        _pools = new MultiplayerPools();
        _pools.Initialise(_poolConfig, _syncedContainer);
        var multiplayerData = new MultiplayerData
        {
            ColourOptions = _colorConfig.Options, IsUseSpatialAnchors = _useSpatialAnchors,
            SagaPluginConfiguration = _sagaPluginConfiguration
        };
        var user = new User
        {
            ServerId = "",
            Room = "siro"
        };
        multiplayerData.ThisUser = user;
        _data.MultiplayerData = multiplayerData;
        _leftHandUI.Initialise(_leftHand.Hand, _cameraTransform);
    }

    

    private void OnApplicationPause(bool pauseStatus)
    {
        Debug.Log($"OnApplicationPause {pauseStatus}");
    }

    private void OnApplicationFocus(bool hasFocus)
    {
        Debug.Log($"OnApplicationFocus {hasFocus}");
    }

    private void InitialiseHighlightSystems()
    {
        _data.NavigationHighlightSystem =
            CreateHighlightSystem<NavigationHighlightSystem, NavigationHighlightView, LabelHighlightData>(
                _navigationHighlightPrefab, _normalHighlightColor);

        _data.DetectingObjectsHighlightSystem =
            CreateHighlightSystem<DetectingObjectsHighlightSystem, DetectingObjectHighlightView, TargetObjectHighlight>(
                _detectingObjectHighlightPrefab, _normalHighlightColor);

        _data.TargetObjectHighlightSystem = new TargetObjectHighlightSystem();
        _data.TargetObjectHighlightSystem.Initialise(_ovrCameraRig.centerEyeAnchor, _robotHomeContainer,
            _targetObjectHighlightPrefab, _data.RobotData, _normalHighlightColor);

        _data.PlaceLocationHighlightSystem =
            CreateHighlightSystem<PlaceLocationHighlightSystem, PlaceLocationMarker, PlaceLocationData>(
                _placeLocationHighlightPrefab, _normalHighlightColor);
        _semanticPlaceWorldUpMat.SetColor(BaseColor, _normalHighlightColor);
        _semanticPlaceTargetMat.SetColor(BaseColor, _semanticPlaceTargetArrowColor);
        _data.SemanticPlaceHighlightSystem =
            CreateHighlightSystem<SemanticPlaceHighlightSystem, SemanticPlaceMarker, SemanticPlaceLocationData>(
                _semanticPlacePrefab, _normalHighlightColor);
        _data.SemanticPlaceHighlightSystem.Initialise(_ovrCameraRig.centerEyeAnchor, _data.RobotData);
        _data.WorldGraphVisualsSystem = new WorldGraphVisualsSystem();
        _data.WorldGraphVisualsSystem.Initialise(_cameraTransform,
            _data.WorldGraphMessageHandler, _data.WorldGraphData, _worldGraphColor);
        _data.FeatureActivationHandler.AddFeature(AppFeatures.WORLD_VISUALISATION, _data.WorldGraphVisualsSystem);

        var recordingPanel = _data.LeftHandUI.RecordInstructionPanel;
        _data.FeatureActivationHandler.AddFeature(AppFeatures.MANUAL_STT_INSTRUCTION_PANEL,
            new ManualInstructionPanelVisualisationSystem(_robotHomeUI, recordingPanel));
    }

    private S CreateHighlightSystem<S, V, D>(GameObject prefab, Color color) where S : HighlightSystem<V, D>, new()
        where V : HighlightView
        where D : HighlightData
    {
        var system = new S();
        int poolSize = 20;
        system.Initialise(_ovrCameraRig.centerEyeAnchor, _highlightContainer, prefab, color,poolSize);
        return system;
    }

    private void InitialiseHandlers()
    {
        _data.PlannerData = new PlannerData();
        _data.MultiplayerHandler =
            new MultiplayerMessageHandler(_data.MultiplayerData, _pools, _spatialAnchorService, _notificationSystem,
                _mainCamera);
        _data.PlannerHandler = new PlannerMessageHandler(_data.PlannerData);
        _data.SkillsMessageHandler = new SkillsMessageHandler(_data.RobotData);
        _data.WorldGraphMessageHandler = new WorldGraphMessageHandler(_data.WorldGraphData);
        _data.HumanActivityMessageHandler = new HumanActivityMessageHandler();
        _data.SpeechMessageHandler = new SpeechMessageHandler();
        _data.SiroMessageHandler = new SiroMessageHandler();

        _data.SiroService.AddHandler(MultiplayerMessageHandler.Type, _data.MultiplayerHandler);
        _data.SiroService.AddHandler(PlannerMessageHandler.Type, _data.PlannerHandler);
        _data.SiroService.AddHandler(SkillsMessageHandler.TYPE, _data.SkillsMessageHandler);
        _data.SiroService.AddHandler(WorldGraphMessageHandler.TYPE, _data.WorldGraphMessageHandler);
        _data.SiroService.AddHandler(HumanActivityMessageHandler.TYPE, _data.HumanActivityMessageHandler);
        _data.SiroService.AddHandler(SpeechMessageHandler.TYPE, _data.SpeechMessageHandler);
        _data.SiroService.AddHandler(SiroMessageHandler.TYPE, _data.SiroMessageHandler);

        _data.FeatureActivationHandler = new AppFeatureHandler(_notificationSystem, _data.MultiplayerHandler);
    }

    private void InitialiseSpatialAnchorService()
    {
        _spatialAnchorService = new SpatialAnchorService();
        _spatialAnchorService.Initialise(_data.SpatialAnchorConfig, _data.MultiplayerData, _cameraTransform,
            _robotHomeContainer, _syncedContainer, _notificationSystem, _mainCamera, _isShowDebugObjects);
        _data.SpatialAnchorService = _spatialAnchorService;
        _spatialAnchorService.OnAnchorSavedEvent += OnAnchorSavedEvent;
        _spatialAnchorService.OnAnchorSharedEvent += OnAnchorSharedEvent;
        _spatialAnchorService.OnAnchorShareErrorEvent += OnAnchorShareErrorEvent;
        _spatialAnchorService.OnAnchorSaveErrorEvent += OnAnchorSaveErrorEvent;
        _spatialAnchorService.OnFatalAnchorLoadError += OnFatalAnchorLoadError;
    }

    private void OnAnchorShareErrorEvent(SpatialAnchorView anchor)
    {
        _data.FatalError = $"Could not share anchor {anchor.Data.Name}. Quit the app, walk the room in headset and restart";
        _stateMachine.changeState<FatalErrorState>();
    }
    
    private void OnAnchorSaveErrorEvent(SpatialAnchorView anchor)
    {
        _data.FatalError = $"Could not share anchor {anchor.Data.Name}. Quit the app, walk the room in headset and restart";
        _stateMachine.changeState<FatalErrorState>();
    }
    
    private void OnFatalAnchorLoadError(string error)
    {
        _data.FullFocusCanvasUI.HideAll();
        _data.FatalError = error;
        _stateMachine.changeState<FatalErrorState>();
    }

    //@TODO move these to a better place
    private void OnAnchorSharedEvent(SpatialAnchorView anchor)
    {
        if (Application.isEditor && !_isSaveAnchorInEditor) return;
        _data.SiroService.SendMessage(SpatialAnchorMessage.Type,
            new SpatialAnchorMessage
            {
                EventName = MultiplayerMessageHandler.Type, EventType = SpatialAnchorMessage.Type, Data = anchor.Data,
                Room = _data.MultiplayerData.Room.Id
            });
    }

    private void OnAnchorSavedEvent(SpatialAnchorView anchor)
    {
        if (Application.isEditor && !_isSaveAnchorInEditor) return;
        _data.SiroService.SendMessage(SpatialAnchorMessage.Type,
            new SpatialAnchorMessage
            {
                EventName = MultiplayerMessageHandler.Type, EventType = SpatialAnchorMessage.Type, Data = anchor.Data,
                Room = _data.MultiplayerData.Room.Id
            });
    }

    private void SendSyncedEntityUpdate()
    {
        var trackedTransform = _trackedUserPanel.transform;
        trackedTransform.localPosition = _syncedContainer.InverseTransformPoint(_cameraTransform.position);
        trackedTransform.localRotation = _cameraTransform.rotation;
        if (!_data.MultiplayerData.IsSendSyncedUpdates()) return;
        var entityTransform = _data.MultiplayerData.HeadEntityView.transform;
        _data.SiroService.SendMessage(SyncedEntityMessage.Type,
            new SyncedEntityMessage
            {
                EventType = SyncedEntityMessage.Type,
                EventName = MultiplayerMessageHandler.Type,
                EntityId = _data.MultiplayerData.HeadEntityView.Id,
                Room = _data.MultiplayerData.Room.Id,
                Position = trackedTransform.localPosition,
                Rotation = trackedTransform.localRotation,
                Scale = entityTransform.localScale
            });
    }

    private void InitialiseHeadSyncedEntity()
    {
        var headGO = Instantiate(_headEntityView);
        var headId = Guid.NewGuid().ToString();
        headGO.name = headId;
        _data.MultiplayerData.HeadEntityView = headGO.GetComponent<SyncedEntityView>();
        _data.MultiplayerData.HeadEntityView.Initialise(headId);
        _data.MultiplayerData.ThisUser.HeadEntityId = headId;
        _trackedUserPanel = _data.MultiplayerData.HeadEntityView;
    }

    private void InitialiseStateMachine()
    {
        _stateMachine = new SKStateMachine<AppData>(_data, new WaitForCameraInitialisationState());
        _stateMachine.addState(new RequestMicrophonePermissionState());
        _stateMachine.addState(new ConnectToSiroServiceState());
        _stateMachine.addState(new RetrieveOVRDetailsState());
        _stateMachine.addState(new RetrieveEditorDetailsState());
        _stateMachine.addState(new EditUserDetailsState());
        _stateMachine.addState(new ConfirmUserDetailsState());

        _stateMachine.addState(new JoinRoomState());
        _stateMachine.addState(new InitialisingRoomState());
        _stateMachine.addState(new LoadWorldGraphState());
        _stateMachine.addState(new ConfigureWorldGraphPanelsState());
        _stateMachine.addState(new WaitingForWorldGraphPanelsState());
        _stateMachine.addState(new CreateRobotHomeAnchorState());
        _stateMachine.addState(new ThreeDotRobotSyncState());
        _stateMachine.addState(new WaitingForAnchorsState());
        _stateMachine.addState(new RobotCommandState());
        _stateMachine.addState(new FatalErrorState());
    }

    private void InitialiseServiceData()
    {
        BaseSiroService siroService = _isUseStubSiroService ? new StubSiroService(_data) : new SiroService();
        siroService.OnServiceConnectionError += OnServiceConnectionError;
        siroService.OnServiceConnected += OnServiceConnected;
        siroService.OnServiceDisconnected += OnServiceDisconnected;
        //***ERIC added the call below
        //OnServiceConnected();
        _data.SiroService = siroService;
    }

    private void RunHighlightSystems()
    {
        if (_data == null) return;
        _data.NavigationHighlightSystem?.Tick();
        _data.DetectingObjectsHighlightSystem?.Tick();
        _data.TargetObjectHighlightSystem?.Tick();
        _data.PlaceLocationHighlightSystem?.Tick();
        _data.WorldGraphVisualsSystem?.Tick();
        _data.SemanticPlaceHighlightSystem?.Tick();
    }

    private void RunStateMachine()
    {
        _stateMachine?.update(Time.deltaTime);
    }

    private void RunServicesTick()
    {
        if (_data == null) return;
        _data.SiroService?.Update();
        _data.SpatialAnchorService?.Update();
        _data.MultiplayerHandler.Tick();
    }

    private void OnServiceDisconnected()
    {
        Debug.Log("Root:OnServiceDisconnected");
        _data.MultiplayerData.IsConnected = false;
    }

    private void OnServiceConnected()
    {
        Debug.Log("Root:OnServiceConnected");
        _data.MultiplayerData.IsConnected = true;
    }

    private void OnServiceConnectionError()
    {
        _data.MultiplayerData.IsConnected = false;
        Debug.Log("Root:OnServiceConnectionError");
    }

    public void OnDestroy()
    {
        _stateMachine?.currentState?.end();
        _data?.SiroService?.Dispose();
        _data?.SpatialAnchorService?.Dispose();
    }
}