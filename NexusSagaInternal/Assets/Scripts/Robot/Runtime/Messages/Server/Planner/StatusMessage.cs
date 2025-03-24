using Newtonsoft.Json;

namespace Robot.Runtime.Messages.Server.Planner
{
    public class StatusMessage
    {
        [JsonProperty("status")]
        public string Status;
    }
}