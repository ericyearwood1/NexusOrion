using System;
using Robot.Runtime.Data;
using TMPro;
using UnityEngine;
using View;

public class SendInstructionsPanel : SiroUIView
{
    [SerializeField] private TMP_Text _recordingState;

    public void Initialise()
    {
        _recordingState.text = "";
    }

    public void SetState(RecordingState state, string message = "")
    {
        switch (state)
        {
            case RecordingState.Started:
                _recordingState.text = "RECORDING";
                break;
            case RecordingState.Recording: 
                _recordingState.text = "RECORDING";
                break;
            case RecordingState.Complete: 
                _recordingState.text = "SENDING INSTRUCTION";
                break;
            case RecordingState.Error: 
                _recordingState.text = "RECORDING ERROR";
                break;
            case RecordingState.Cancelled: 
                _recordingState.text = "RECORDING CANCELLED";
                break;
            case RecordingState.Transcribed: 
                Debug.Log($"Setting recording state to TRANSCRIBED: {message}");
                _recordingState.text = message;
                break;
            case RecordingState.None:
                break;
            case RecordingState.Ready:
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(state), state, null);
        }
    }
}
