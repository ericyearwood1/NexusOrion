using System;

namespace SiroComms.Runtime.Data
{
    public class SiroServiceData
    {
        public Action<ConnectionStatus> OnServerConnectionStateChange;

        public ushort Port { get; private set; }
        public string Host { get; private set; }

        public string ServerUrl { get; private set; }
        public ConnectionStatus ServerConnectionState { get; set; }
        public string ConnectionError { get; set; }

        public void SetServiceURL(string ip, ushort port)
        {
            Host = ip;
            Port = port;
            ServerUrl = $"ws://{ip}:{port}";
        }
        
        public void OnStateChange()
        {
            if (OnServerConnectionStateChange != null)
            {
                OnServerConnectionStateChange(ServerConnectionState);
            }
        }
    }
}