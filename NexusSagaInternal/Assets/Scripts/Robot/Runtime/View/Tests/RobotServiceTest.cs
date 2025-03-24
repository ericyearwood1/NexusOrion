using System;
using System.Buffers.Text;
using System.Collections.Generic;
using Notifications.Runtime.Systems;
using Robot.Runtime.Data;
using Robot.Runtime.Data.Planner;
using Robot.Runtime.Messages.Client;
using Robot.Runtime.Messages.Server.Planner;
using Robot.Runtime.ServiceHandlers;
using SiroComms.Runtime.Data;
using SiroComms.Runtime.Services;
using Speech.Data;
using Speech.Messages.Client;
using Speech.ServiceHandlers;
using TMPro;
using UnityEngine;

namespace Robot.Runtime.View
{
    [RequireComponent(typeof(AudioSource))]
    public class RobotServiceTest : MonoBehaviour
    {
        [SerializeField] private string _address = "0.0.0.0";
        [SerializeField] private ushort _port = 5432;
        [SerializeField] private TMP_Text _log;
        [SerializeField] private TMP_InputField _instruction;
    
        private AudioSource _audioSource;
        private SiroService _service;
        private SiroServiceData _data;
        private PlannerMessageHandler _plannerMessageHandler;
        private SkillsMessageHandler _skillsMessageHandler;
        private WorldGraphMessageHandler _worldGraphMessageHandler;
        private HumanActivityMessageHandler _humanActivityMessageHandler;
        private SpeechMessageHandler _speechMessageHandler;
        
        private bool _isInstructionInProgress;
        private string _voiceInstruction;
        private bool _isVoiceInstructionPending;
        
        private List<string> _messagesToSpeak = new List<string>();
        private List<byte[]> _audioBytesToConvert = new List<byte[]>();
        private List<AudioClip> _audioClipsToPlay = new List<AudioClip>();
        private List<string> _playedMessageIds = new List<string>();

        private List<string> _logText = new();
        private RobotData _robotData;
        private WorldGraphData _worldGraphData = new WorldGraphData();

        private double _recordStartTime;
        public double CurrentRecordTime => Time.time - _recordStartTime;
        private bool _isRecording;
        [SerializeField] private int _defaultRecordDuration = 5;
        private AudioClip _audioClip;
        
        public bool IsPlaying { get; }

        private void Awake()
        {
            _audioSource = GetComponent<AudioSource>();
            _data = new SiroServiceData();
            _data.SetServiceURL(_address, _port);
            _data.OnServerConnectionStateChange += OnServiceConnectionStateChanged;
            _robotData = new RobotData();
            _service = new SiroService();
            _plannerMessageHandler = new PlannerMessageHandler(new PlannerData());
            _plannerMessageHandler.OnNextActionReceived += OnNextActionReceived;
            _plannerMessageHandler.OnActionFeedbackUpdate += OnActionFeedbackUpdate;
            _plannerMessageHandler.OnInstructionComplete += OnInstructionComplete;
            _plannerMessageHandler.OnServiceUnavailable += OnPlanningServiceUnavailable;
            _plannerMessageHandler.OnPlannerBusy += OnPlannerBusy;
            _plannerMessageHandler.OnInstructionNotUnderstood += OnInstructionNotUnderstood;
            
            _skillsMessageHandler = new SkillsMessageHandler(_robotData);
            
            _worldGraphMessageHandler = new WorldGraphMessageHandler(_worldGraphData);
            
            _humanActivityMessageHandler = new HumanActivityMessageHandler();
            
            _speechMessageHandler = new SpeechMessageHandler();
            _speechMessageHandler.OnTtsResult += HandleTtsResult;
            _speechMessageHandler.OnSttResult += HandleAsrResult;
            
            _service.AddHandler(PlannerMessageHandler.Type, _plannerMessageHandler);
            _service.AddHandler(SkillsMessageHandler.TYPE, _skillsMessageHandler);
            _service.AddHandler(WorldGraphMessageHandler.TYPE, _worldGraphMessageHandler);
            _service.AddHandler(HumanActivityMessageHandler.TYPE, _humanActivityMessageHandler);
            _service.AddHandler(SpeechMessageHandler.TYPE, _speechMessageHandler);
            
            _service.OnServiceConnectionError += OnServiceConnectionError;
        }

        private void Start()
        {
            _service.Start(_data);
        }

        private void Update()
        {
            if (_service == null) return;
            try
            {
                _service.Update();
            }
            catch (Exception e)
            {
                Debug.LogError(e);
            }
            
            UpdateLog();

            if (_isVoiceInstructionPending)
            {
                _isVoiceInstructionPending = false;
                SendInstruction(_voiceInstruction);
            }
            
            if (_messagesToSpeak.Count > 0)
            {
                var message = _messagesToSpeak[0];
                _messagesToSpeak.RemoveAt(0);
                TextToSpeech(message);
            }
            if(_audioBytesToConvert.Count > 0)
            {
                var audioBytes = _audioBytesToConvert[0];
                _audioBytesToConvert.RemoveAt(0);
                var audioClip = WavUtility.ToAudioClip(audioBytes, 0, "tts");
                _audioClipsToPlay.Add(audioClip);
            }
            
            if (_audioClipsToPlay.Count > 0 && !IsPlaying)
            {
                var audioClip = _audioClipsToPlay[0];
                _audioClipsToPlay.RemoveAt(0);
                
                PlayAudioClip(audioClip);
            }
            
            if (_isRecording)
            {
                if (CurrentRecordTime >= _defaultRecordDuration || Microphone.IsRecording(null) == false)
                {
                    StopRecording();
                }
            }
        }

        private void PlayAudioClip(AudioClip audioClip)
        {
            _audioSource.clip = audioClip;
            _audioSource.Play();
        }

        private void TextToSpeech(string message)
        {
            Debug.Log($"Text to speech: {message}");
            Log($"\nText to speech: {message}");
            _service.SendMessage(ServiceEvents.TextToSpeech, new TtsRequestMessage
            {
                Data = new TtsRequestData()
                {
                    text = message,
                    SourceLanguage = "eng",
                    TargetLanguage = "eng"
                }
            });
        }


        private void HandleAsrResult(SttResultMessage asrresult)
        {
            _voiceInstruction = asrresult.Text;
            Debug.Log($"ASR Result: {_voiceInstruction}");
            Log($"\nASR Result: {_voiceInstruction}");
            _isVoiceInstructionPending = true;
        }

        private void HandleTtsResult(TtsResultMessage ttsresult)
        {
            _audioBytesToConvert.Add(Convert.FromBase64String(ttsresult.Audio));
        }

        private void OnActionFeedbackUpdate(ActionFeedbackMessage obj)
        {
            Log($"\n<color=\"grey\">{obj.Action} | {obj.Feedback}</color>");
            var feedbackMessages = obj.Feedback.Split("-");
            foreach (var message in feedbackMessages)
            {
                Debug.Log($"Feedback: {message}");
                // _textToSpeech.Speak(message.Replace("_", " ").Trim());
                _messagesToSpeak.Add(message.Replace("_", " ").Trim());
            }
        }

        private void OnNextActionReceived(NextActionDTO obj)
        {
            Log($"\n<color=\"blue\">{obj.Action}</color>");
        }

        private void OnInstructionComplete()
        {
            _isInstructionInProgress = false;
            Log($"\nInstruction Complete");
        }
        
        private void OnPlanningServiceUnavailable()
        {
            _isInstructionInProgress = false;
            Log($"\n<color=\"red\">Planning Service is unavaiable</color>");
        }
        
        private void OnInstructionNotUnderstood(string instruction)
        {
            _isInstructionInProgress = false;
            Log($"\n<color=\"red\">Instruction was not understood {instruction}</color>");
        } 
        
        private void OnPlannerBusy(string currentInstruction)
        {
            _isInstructionInProgress = true;
            Log($"\n<color=\"red\">Planner is busy with another instruction: {currentInstruction}</color>");
        }

        public void StartService()
        {
            _service.Start(_data);
        }
        
        public void StopService()
        {
            _service.Stop();
        }

        private void OnServiceConnectionStateChanged(ConnectionStatus status)
        {
            Log($"\nOnServiceConnectionStateChanged: {status}");
        }

        private void OnServiceConnectionError()
        {
            Log($"\nOnServiceConnectionError: {_data.ConnectionError}");
        }

        public void OnDestroy()
        {
            if (_service == null) return;
            _service.Dispose();
        }

        public void SendInstruction()
        {
            var instruction = _instruction.text;
            if (string.IsNullOrWhiteSpace(instruction))
            {
                Debug.LogError("Instruction is empty. Ignoring");
                return;
            }

            SendInstruction(instruction);
        }

        public void SendBadInstruction()
        {
            SendInstruction("test_bad");
        }

        public void RecordMicrophone()
        {
            RecordMicrophone(_defaultRecordDuration);
        }

        public void RecordMicrophone(int duration)
        {
            if (_isRecording)
            {
                Debug.LogError("Already recording");
                return;
            }
            _isRecording = true;
            _recordStartTime = Time.time;
            Log($"\nRecording for {duration} seconds");
            _audioClip = Microphone.Start(null, false, duration, 16000);
        }

        public void StopRecording()
        {
            Microphone.End(null);
            _isRecording = false;
            Log($"\nRecording stopped");
            SpeechToText(_audioClip);
            _audioClip = null;
        }

        public void SpeechToText(AudioClip audioClip)
        {
            var audioBytes = WavUtility.FromAudioClip(audioClip);
            var audioBase64 = Convert.ToBase64String(audioBytes);
            // Log($"\nSending audio: {audioBase64}");
            _service.SendMessage(ServiceEvents.SpeechToText, new SttRequestMessage(
                new SttRequestData()
                {
                    Audio = audioBase64,
                    SourceLanguage = "eng",
                    TargetLanguage = "eng"
                }
            ));
        }
        
        private void Log(string message)
        {
            Debug.Log(message);
            try
            {
                _logText.Add(message);
            }
            catch (Exception e)
            {
                Debug.LogError(e);
            }
            
        }

        private void UpdateLog()
        {
            _log.text = string.Join("", _logText);
        }
        
        private void SendInstruction(string instruction)
        {
            Debug.Log($"Sending instruction: {instruction}");
            if(string.IsNullOrWhiteSpace(instruction))
            {
                Debug.LogError("Instruction is empty. Ignoring");
                Log("\nInstruction is empty, sending bad instruction");
                SendBadInstruction();
                return;
            }
            if (_isInstructionInProgress)
            {
                Debug.LogError("Planner is busy. Ignoring");
                Log("\nPlanner is busy");
                return;
            }

            _isInstructionInProgress = true;
            Debug.Log("Sending instruction...");
            Log($"\n\nSending instruction {instruction}");
            Debug.Log("Thinking...");
            Log($"\nThinking...");
            _service.SendMessage(ServiceEvents.Instruction, new InstructionMessage(new TextInstruction(instruction, "23423522")));
        }
    }
}

