using System;
using Newtonsoft.Json;
using Robot.Runtime.Data;
using SiroComms.Runtime.Messages.Client;

namespace Robot.Runtime.Messages.Client
{
    [Serializable]
    public class InstructionMessage : IClientMessage
    {
        [JsonProperty("data")]
        public Instruction Instruction;

        [JsonProperty("service")]
        public string Service = "planner";

        [JsonProperty("debug")]
        public object Debug = new object();
        
        public InstructionMessage(Instruction instruction)
        {
            Instruction = instruction;
        }
    }
}