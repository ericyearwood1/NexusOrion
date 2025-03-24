namespace SiroComms.Runtime.Services
{
    public interface IMessageHandler
    {
        void HandleMessage(string messageType, string message);
    }
}