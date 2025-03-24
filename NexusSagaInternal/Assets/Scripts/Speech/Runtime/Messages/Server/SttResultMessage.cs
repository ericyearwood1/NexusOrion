using System;
using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Client;
using Speech.Data;

namespace Speech.Messages.Client
{
    [Serializable]
    public class SttResultMessage 
    {
        [JsonProperty("user_id")]
        public string UserId;
        [JsonProperty("id")]
        public string Id;
        [JsonProperty("text")]
        public string Text;
        [JsonProperty("source_language")]
        public string SourceLanguage;
        [JsonProperty("target_language")]
        public string TargetLanguage;
    }
}