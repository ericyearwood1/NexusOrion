using System;
using Newtonsoft.Json;
using Robot.Runtime.Data.Planner;
using SiroComms.Runtime.Messages.Server;

namespace Robot.Runtime.Messages.Server.Planner
{
    [Serializable]
    public class ActionFeedbackMessage 
    {
        [JsonProperty("action")]
        public string Action;
        
        [JsonProperty("feedback")]
        public string Feedback;
        
        [JsonProperty("status")]
        public PlannerActionState State;
    }
}