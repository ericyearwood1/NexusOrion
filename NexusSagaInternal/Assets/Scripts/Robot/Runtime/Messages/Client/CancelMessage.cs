using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Client;

namespace Robot.Runtime.Messages.Client
{
    public class CancelMessage : IClientMessage
    {
        [JsonProperty("service")]
        public string Service = "planner";

        [JsonProperty("data")]
        public object Data = new object();
        
    }
}