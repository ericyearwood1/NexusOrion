using System;
using SiroComms.Runtime.Messages.Server;
using SiroComms.Runtime.Services;
using Speech.Messages.Client;
using UnityEngine;

namespace Speech.ServiceHandlers
{
    public class SpeechMessageHandler : IMessageHandler
    {
        public const string TYPE = "speech";
        private const string TEXT_TO_SPEECH_RESULT = "tts_result";
        public const string SPEECH_TO_TEXT_RESULT = "asr_result";
        
        public Action<TtsResultMessage> OnTtsResult;
        public Action<SttResultMessage> OnSttResult;
        
        public void HandleMessage(string messageType, string message)
        {
            switch (messageType)
            {
                case SPEECH_TO_TEXT_RESULT:
                    HandleSttResult(message);
                    break;
                case TEXT_TO_SPEECH_RESULT:
                    HandleTtsResult(message);
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(messageType), messageType, null);
            }
        }
        
        private void HandleTtsResult(string message)
        {
            Debug.Log($"Handling TTS result: {message}");
            ProcessMessage(message, OnTtsResult);
        }
        
        private void HandleSttResult(string message)
        {
            Debug.Log($"Handling STT result: {message}");
            ProcessMessage(message, OnSttResult);
        }
        
        private static void ProcessMessage<T>(string message, Action<T> callback)
        {
            Debug.Log($"Processing message: {message}");
            var siroMessage = ServerMessage<T>.Deserialize(message).Data;
            
            Debug.Log($"SiroMessage: {siroMessage} | {message}");
            if (siroMessage == null)
            {
                Debug.LogError($"Could not process message {message}");
                return;
            }
            callback?.Invoke(siroMessage);
        }
        
    }
}