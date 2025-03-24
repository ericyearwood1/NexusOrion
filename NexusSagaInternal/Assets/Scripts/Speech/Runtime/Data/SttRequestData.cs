using System;
using Newtonsoft.Json;

namespace Speech.Data
{
    [Serializable]
    public class SttRequestData
    {
        [JsonProperty("user_id")]
        public string UserId;
        [JsonProperty("id")]
        public string id;
        [JsonProperty("audio")]
        public string Audio;
        [JsonProperty("source_language")]
        public string SourceLanguage;
        [JsonProperty("target_language")]
        public string TargetLanguage;
    }
}