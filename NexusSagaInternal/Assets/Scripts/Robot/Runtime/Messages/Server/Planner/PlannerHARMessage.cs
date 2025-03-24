using System;
using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Server;

namespace Robot.Runtime.Messages.Server.Planner
{
    [Serializable]
    public class PlannerHARMessage
    {
        [JsonProperty("action")]
        public string Action;
        
        [JsonProperty("is_replanned")]
        public bool IsReplanned;
        
        [JsonProperty("display_message")]
        public string DisplayMessage;

    }
}