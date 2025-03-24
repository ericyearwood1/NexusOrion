using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Server;

namespace Robot.Runtime.Messages.Server.HumanActivity
{
    public class HumanActivityMessage
    {
        [JsonProperty("activity")] 
        public string Activity;
        [JsonProperty("object")]
        public string Object;
        [JsonProperty("receptacle")]
        public string Receptacle;
    }
}