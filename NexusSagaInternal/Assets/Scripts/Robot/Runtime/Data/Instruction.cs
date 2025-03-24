using System;
using Newtonsoft.Json;

namespace Robot.Runtime.Data
{
    [Serializable]
    public class Instruction
    {
        [JsonProperty("user")]
        public string UserId;
        
        [JsonProperty("data")]
        public string Data;
        
        [JsonProperty("type")]
        public string Type;

        public Instruction(string instruction, string type, string userId)
        {
            Data = instruction;
            Type = type;
            UserId = userId;
        }
    }
}