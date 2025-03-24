using System;
using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Client;
using Speech.Data;

namespace Speech.Messages.Client
{
    [Serializable]
    public class TtsRequestMessage : IClientMessage
    {
        [JsonProperty("data")]
        public TtsRequestData Data;
        
        [JsonProperty("service")]
        public string Service = "speech";
        
        
        
    }
}