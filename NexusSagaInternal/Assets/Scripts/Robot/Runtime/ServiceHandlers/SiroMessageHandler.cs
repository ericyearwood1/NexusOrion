using System;
using System.Collections.Generic;
using System.Linq;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using SiroComms.Runtime.Services;
using UnityEngine;

namespace Robot.Runtime.ServiceHandlers
{
    public class SiroMessageHandler : IMessageHandler
    {
        public const string TYPE = "siro";
        private const string DefaultInstructions = "default_instructions";
        
        public Action<List<string>> OnDefaultInstructionsReceived; 
        public void HandleMessage(string messageType, string message)
        {
            switch (messageType)
            {
                case DefaultInstructions:
                    HandleDefaultInstructions(message);
                    break;
                default:
                    Debug.LogError($"Unknown message type: {messageType}");
                    break;
            }
        }

        private void HandleDefaultInstructions(string message)
        {
            Debug.Log(message);
            JsonConvert.DeserializeObject<Dictionary<string, object>>(message).TryGetValue("data", out var data);
            
            JToken instructions;
            
            if (data is JObject dataObject)
            {
                dataObject.TryGetValue("instructions", out instructions);
            }
            else
            {
                Debug.LogError("Invalid message format");
                return;
            }
            
            if (instructions is JArray instructionsArray)
            {
                var instructionList = instructionsArray.Select(instruction => instruction.ToString()).ToList();
                OnDefaultInstructionsReceived?.Invoke(instructionList);
            }
            else
            {
                Debug.LogError("Invalid message format");
            }

            
        }
    }
}
