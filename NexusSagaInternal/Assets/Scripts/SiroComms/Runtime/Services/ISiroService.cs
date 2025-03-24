using SiroComms.Runtime.Data;
using SiroComms.Runtime.Messages.Client;

namespace SiroComms.Runtime.Services
{
    public interface ISiroService
    {
        void Start(SiroServiceData data);
        void Stop();
        void Dispose();
        void Update();
        void SendMessage(string messageType, IClientMessage message);
        void SendRequest(string eventName);
    }
}