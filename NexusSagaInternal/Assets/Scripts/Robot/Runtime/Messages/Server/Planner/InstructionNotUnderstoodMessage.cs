using System;
using Newtonsoft.Json;

namespace Robot.Runtime.Messages.Server.Planner
{
    [Serializable]
    public class InstructionNotUnderstoodMessage
    {
        [JsonProperty("instruction")]
        public string Instruction;
    }
}