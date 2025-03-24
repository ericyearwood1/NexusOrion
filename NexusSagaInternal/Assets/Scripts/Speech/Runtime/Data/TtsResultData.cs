using System;
using Newtonsoft.Json;
using UnityEngine.Serialization;

namespace Speech.Data
{
    [Serializable]
    public class TtsResultData 
    {
        [JsonProperty("user_id")]
        public string UserId;
        [JsonProperty("id")]
        public string Id;
        [JsonProperty("audio")]
        public string Audio;
        
        
        
        
    }
}