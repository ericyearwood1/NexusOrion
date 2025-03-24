using IX.Saga;
using Meta.XR.MRUtilityKit;
using Multiplayer.Runtime.Config;
using Multiplayer.Runtime.Data;
using Multiplayer.Runtime.Services;
using Multiplayer.Runtime.Systems;
using Notifications.Runtime.Systems;
using Oculus.Interaction.Input;
using Robot.Runtime.Data;
using Robot.Runtime.Data.Planner;
using Robot.Runtime.ServiceHandlers;
using Robot.Runtime.Systems;
using SiroComms.Runtime.Data;
using SiroComms.Runtime.Services;
using Speech.Config;
using Speech.ServiceHandlers;
using UnityEngine;
using View;
using SagaPluginConfiguration = Multiplayer.Runtime.Data.SagaPluginConfiguration;

namespace Data
{
    public class AppData
    {
        public string SagaIP { get; set; }
        public bool IsUseStubService { get; set; }
        public MRUK MRUK { get; set; }
        public HandRef LeftHand { get; set; }
        public HandRef RightHand { get; set; }
        public RobotData RobotData { get; set; }
        public SiroServiceData ServiceData { get; set; }
        public SpatialAnchorConfig SpatialAnchorConfig { get; set; }
        public ServiceConfig SiroServiceConfig { get; set; }
        public SpeechConfig SpeechConfig { get; set; }
        public ColorConfig ColorConfig { get; set; }
        public Transform CameraTransform { get; set; }
        public SpatialAnchorService SpatialAnchorService { get; set; }
        public BaseSiroService SiroService { get; set; }
        public MultiplayerData MultiplayerData { get; set; }
        public FullFocusCanvasUI FullFocusCanvasUI { get; set; }
        public HandUI LeftHandUI { get; set; }
        public DebugUI DebugUI { get; set; }
        public RobotHomeUI RobotHomeUI { get; set; }
        public string FatalError { get; set; }

        public MultiplayerMessageHandler MultiplayerHandler { get; set; }
        public PlannerMessageHandler PlannerHandler { get; set; }
        public SkillsMessageHandler SkillsMessageHandler { get; set; }
        public WorldGraphMessageHandler WorldGraphMessageHandler { get; set; }
        public HumanActivityMessageHandler HumanActivityMessageHandler { get; set; }
        public PlannerData PlannerData { get; set; }
        public bool UseServiceDiscovery { get; set; }
        public bool UseSpeechRecognition { get; set; }
        public bool UseDebugUI { get; set; }
        public SpeechMessageHandler SpeechMessageHandler { get; set; }
        public SiroMessageHandler SiroMessageHandler { get; set; }
        public NavigationHighlightSystem NavigationHighlightSystem { get; set; }
        public DetectingObjectsHighlightSystem DetectingObjectsHighlightSystem { get; set; }
        public TargetObjectHighlightSystem TargetObjectHighlightSystem { get; set; }
        public PlaceLocationHighlightSystem PlaceLocationHighlightSystem { get; set; }
        public SemanticPlaceHighlightSystem SemanticPlaceHighlightSystem { get; set; }
        // public DrawerHighlightSystem DrawerHighlightSystem { get; set; }
        public SagaPlugin SagaPlugin { get; set; }
        public NotificationSystem NotificationSystem { get; set; }
        public WorldGraphData WorldGraphData { get; set; }
        public bool IsUseSpatialAnchors { get; set; }
        public WorldGraphVisualsSystem WorldGraphVisualsSystem { get; set; }
        public AppFeatureHandler FeatureActivationHandler { get; set; }
        public bool IsUserSaveAvailable { get; set; }
        public Transform CoLocationRoot { get; set; }
        public OVRCameraRig OVRCameraRig { get; set; }
        public Transform RightHandPointerPose { get; set; }
        public GameObject PinchMarker { get; set; }
        public string UserIdString { get; set; }

        public void EnableSagaPlugin()
        {
            Debug.Log($"EnableSagaPlugin {SagaIP} | {MultiplayerData.SagaPluginConfiguration} | {MultiplayerData.ThisUser.UserType}");
            if(MultiplayerData.SagaPluginConfiguration == SagaPluginConfiguration.AlwaysDisabled) return;
            if (Application.isEditor) return;
            if (MultiplayerData.ThisUser.UserType == UserType.Guest || MultiplayerData.SagaPluginConfiguration == SagaPluginConfiguration.AlwaysEnabled)
            {
                if (!string.IsNullOrWhiteSpace(SagaIP))
                {
                    SagaPlugin._configuration.pixieServerIp = SagaIP;
                }
                Debug.Log($"ENABLING SAGA PLUGIN {SagaPlugin._configuration.pixieServerIp}");
                if (SagaPlugin != null)
                {
                    SagaPlugin.gameObject.SetActive(true);
                }
            }
        }
    }
}