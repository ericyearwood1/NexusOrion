using System;
using Newtonsoft.Json;

namespace Speech.Data
{
    [Serializable]
    public class SttResultData
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