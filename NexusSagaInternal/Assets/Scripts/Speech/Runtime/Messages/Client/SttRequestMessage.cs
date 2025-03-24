using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Client;
using Speech.Data;

namespace Speech.Messages.Client
{
    public class SttRequestMessage : IClientMessage
    {
        [JsonProperty("data")]
        public SttRequestData data;
        [JsonProperty("service")]
        public string service = "speech";
        
        public SttRequestMessage(SttRequestData data)
        {
            this.data = data;
        }
    }
}