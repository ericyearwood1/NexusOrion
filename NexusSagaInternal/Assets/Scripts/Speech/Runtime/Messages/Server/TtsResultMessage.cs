using System;
using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Client;
using Speech.Data;
using UnityEngine.Serialization;

namespace Speech.Messages.Client
{
    [Serializable]
    public class TtsResultMessage : IClientMessage
    {
        [JsonProperty("service")]
        public string service = "speech";
        [JsonProperty("id")]
        public string Id;
        [JsonProperty("audio")]
        public string Audio;

    }
}