using System.Collections.Generic;
using Newtonsoft.Json;

namespace Robot.Runtime.Messages.Server
{
    public class DefaultInstructionsMessage
    {
        [JsonProperty("instructions")]
        public List<string> Instructions;
        
        public const string TYPE = "default_instructions";
    }
}