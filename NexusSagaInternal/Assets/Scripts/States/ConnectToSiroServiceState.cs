using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Data;
using Multiplayer.Runtime.Data;
using Multiplayer.Runtime.Messages.Server;
using Prime31.StateKit;
using SiroComms.Runtime.Data;
using Systems;
using UnityEngine;
using View;

namespace States
{
    public class ConnectToSiroServiceState : SKState<AppData>
    {
        private const int SERVICE_DISCOVERY_PORT = 7400;
        private UdpClient _udpClient;
        private bool _isReadyForAutoConnection;
        private ServiceConnectionView _serviceConnectionView;
        private string _serverIp;

        public override void begin()
        {
            Debug.Log("ServerTest::Begin State:BootstrapState");
            base.begin();
            var savedIP = PlayerPrefs.GetString(Consts.PlayerPref_IPAddress, string.Empty);
            if (Application.isEditor)
            {
                savedIP = "localhost";
            }

            _isReadyForAutoConnection = false;
            _serviceConnectionView = _context.FullFocusCanvasUI.ShowServiceConnectionView(savedIP);
            if (!Application.isEditor && _context.UseServiceDiscovery)
            {
                StartListeningForDiscoveryBroadcast();
            }
            else
            {
                ShowManualConnection();
            }
        }

        private void ShowManualConnection()
        {
            _serviceConnectionView.ShowIPAddressForm();
            _serviceConnectionView.OnAddressEntered += OnAddressEntered;
        }

        private void OnAddressEntered(string address)
        {
            _serviceConnectionView.OnAddressEntered -= OnAddressEntered;
            _serviceConnectionView.ShowConnectingToServiceState();
            ConfigureServiceData(address);
            ConnectToSiroService();
        }

        private void StartListeningForDiscoveryBroadcast()
        {
            if (Application.isEditor)
            {
                ConfigureServiceData(_context.SiroServiceConfig.Address);
                ConnectToSiroService();
            }
            else
            {
                _serviceConnectionView.ShowRetrievingServiceDetailsState();
                var broadcastAddress = new IPEndPoint(IPAddress.Any, SERVICE_DISCOVERY_PORT);
                _udpClient = new UdpClient();
                _udpClient.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                _udpClient.ExclusiveAddressUse = false;
                _udpClient.Client.Bind(broadcastAddress);
                _udpClient.BeginReceive(OnServiceDetailsReceived, null);
            }
        }

        private void OnServiceDetailsReceived(IAsyncResult ar)
        {
            var remote_end = new IPEndPoint(IPAddress.Any, SERVICE_DISCOVERY_PORT);
            var receiveBytes = _udpClient.Receive(ref remote_end);
            var server_ip = Encoding.ASCII.GetString(receiveBytes);
            _serverIp = server_ip;
            _udpClient.Close();
            ConfigureServiceData(server_ip);
            _isReadyForAutoConnection = true;
            if (string.IsNullOrWhiteSpace(server_ip))
            {
                Debug.LogError($"ServerTest::Received empty ip {server_ip}");
                ShowManualConnection();
                return;
            }
        }

        private void ConfigureServiceData(string ip)
        {
            Debug.Log($"ConfigureServiceData:: {ip} | {_context.SiroServiceConfig.Port}");
            var serviceData = new SiroServiceData();
            serviceData.SetServiceURL(ip, _context.SiroServiceConfig.Port);
            _context.ServiceData = serviceData;
        }

        private void ConnectToSiroService()
        {
            _context.SiroService.OnServiceDisconnected += OnServiceDisconnected;
            _context.SiroService.OnServiceConnectionError += OnServiceConnectionError;
            _context.MultiplayerHandler.OnInitialStateReceived += OnInitialStateReceived;
            _serviceConnectionView.ShowConnectingToServiceState();
            _context.SiroService.Start(_context.ServiceData);
        }

        private void RemoveListeners()
        {
            _serviceConnectionView.OnAddressEntered -= OnAddressEntered;
            _context.SiroService.OnServiceDisconnected -= OnServiceDisconnected;
            _context.SiroService.OnServiceConnectionError -= OnServiceConnectionError;
            _context.MultiplayerHandler.OnInitialStateReceived -= OnInitialStateReceived;
        }

        private void OnInitialStateReceived(InitialStateMessage message)
        {
            if (!Application.isEditor)
            {
                PlayerPrefs.SetString(Consts.PlayerPref_IPAddress, _context.ServiceData.Host);
            }

            RemoveListeners();
            ConfigureApplication(message.Config);
            ProcessData(message);
        }

        private void ProcessData(InitialStateMessage message)
        {
            var data = _context.MultiplayerData;
            data.ThisUser.ServerId = message.SID;
            data.RoomList = message.list;
            if (data.RoomList == null || data.RoomList.Length == 0)
            {
                _context.FatalError = $"Empty or null room list received : {data.RoomList}";
                _machine.changeState<FatalErrorState>();
                return;
            }
            if (!Application.isEditor && _context.SpatialAnchorConfig.IsOVRTesting)
            {
                _machine.changeState<RetrieveOVRDetailsState>();
            }
            else
            {
                _machine.changeState<RetrieveEditorDetailsState>();
            }
        }

        private void ConfigureApplication(FeatureConfiguration featureConfiguration)
        {
            if (featureConfiguration == null)
            {
                Debug.Log("Feature configuration is null. using defaults");
                featureConfiguration = new FeatureConfiguration
                {
                    IsNotificationsActive = true,
                    IsShowSemanticPlace = true,
                    IsShowWorldObjectHighlights = true,
                    IsShowPlaceObjectHighlight = true,
                    IsShowTargetObjectHighlight = true,
                    IsWorldGraph2DSurfacesEnabled = true,
                    HARConfig = 1
                };
            }
            Debug.Log($"Feature configuration {featureConfiguration.IsNotificationsActive} | {featureConfiguration.IsShowPlaceObjectHighlight} | {featureConfiguration.IsShowSemanticPlace} | {featureConfiguration.HARConfig}");
            _context.SagaIP = featureConfiguration.SagaIP;
            _context.NotificationSystem.SetEnabled(featureConfiguration.IsNotificationsActive);
            _context.PlaceLocationHighlightSystem.SetEnabled(featureConfiguration.IsShowPlaceObjectHighlight);
            _context.SemanticPlaceHighlightSystem.SetEnabled(featureConfiguration.IsShowSemanticPlace);
            _context.TargetObjectHighlightSystem.SetEnabled(featureConfiguration.IsShowTargetObjectHighlight);
            _context.WorldGraphVisualsSystem.SetWorldGraphObjectsEnabled(featureConfiguration.IsShowWorldObjectHighlights);
            _context.WorldGraphVisualsSystem.SetWorldGraph2DSurfacesEnabled(featureConfiguration.IsWorldGraph2DSurfacesEnabled);

            switch (featureConfiguration.HARConfig)
            {
                case 1:
                    var harActivationSystem =
                        new ActiveHARWithWorldVisualisationSystem(_context.HumanActivityMessageHandler);
                    _context.FeatureActivationHandler.AddFeature(AppFeatures.WORLD_VISUALISATION, harActivationSystem);
                    break;

                case 2:
                    _context.HumanActivityMessageHandler.SetEnabled(false);
                    break;
                case 0:
                default:
                    _context.HumanActivityMessageHandler.SetEnabled(true);
                    break;
            }
        }

        public override void update(float deltaTime)
        {
            if (_context.ServiceData == null) return;
            if (_context.ServiceData.ServerConnectionState != ConnectionStatus.None &&
                _context.ServiceData.ServerConnectionState != ConnectionStatus.NotConnected)
                return;
            if (_isReadyForAutoConnection && !string.IsNullOrWhiteSpace(_serverIp))
            {
                _isReadyForAutoConnection = false;
                ConfigureServiceData(_serverIp);
                ConnectToSiroService();
            }
        }

        private void OnServiceDisconnected()
        {
            RemoveListeners();
            _serviceConnectionView.ShowConnectionErrorState();
            _context.FatalError = "Service Disconnected";
            _machine.changeState<FatalErrorState>();
        }

        private void OnServiceConnectionError()
        {
            RemoveListeners();
            _serviceConnectionView.ShowConnectionErrorState();
            _context.FatalError = "Service Disconnected";
            _machine.changeState<FatalErrorState>();
            Debug.LogError("OnServiceConnectionError");
        }

        public override void end()
        {
            Debug.Log("ConnectToSiroServiceState:End");
            base.end();
            _udpClient?.Close();
            _udpClient?.Dispose();
            _context.FullFocusCanvasUI.HideAll();
        }
    }
}