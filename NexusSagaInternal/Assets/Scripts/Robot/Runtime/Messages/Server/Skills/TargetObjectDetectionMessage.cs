using System;
using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Server;

namespace Robot.Runtime.Messages.Server.Skills
{
    [Serializable]
    public class TargetObjectDetectionMessage
    {
        [JsonProperty("label")]
        public string Label;

        [JsonProperty("targets")]
        public TargetObject[] Targets;

        [JsonProperty("action_id")]
        public string ActionId;
    }
}