using System;
using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Server;

namespace Robot.Runtime.Messages.Server.Planner
{
    [Serializable]
    public class PlannerBusyMessage : ServerMessage
    {
        [JsonProperty("current_instruction")]
        public string Instruction;
    }
}