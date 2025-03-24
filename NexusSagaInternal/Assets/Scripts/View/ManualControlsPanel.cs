using System;
using Data;
using Oculus.Interaction;
using Oculus.Interaction.Input;
using Robot.Runtime.Data;
using UIComponents.Runtime;
using UnityEngine;
using UnityEngine.UI;

namespace View
{
    public class ManualControlsPanel : SiroUIView
    {
        public Action StopButtonClicked;
        public Action OnBackRequest;
        public Action<bool> OnCGVisualisation;
        public Action<bool> OnManualSTT;
        [SerializeField] private Handedness _disabledHandedness = Handedness.Left;
        [SerializeField] private Button _stopButton;
        [SerializeField] private SiroLabelToggle _cgVisualisationToggle;
        [SerializeField] private SiroLabelToggle _manualSTTToggle;
        public void Initialise(bool isCGVisualisationOn, bool isSTTOn)
        {
            _cgVisualisationToggle.Initialise(isCGVisualisationOn);
            _manualSTTToggle.Initialise(isSTTOn);
        }
        
        public void OnBackClicked(PointerEvent pointerEvent)
        {
            if(!IsShown) return;
            if (RecordInstructionPanel.PointerEventIsHand(pointerEvent, _disabledHandedness)){
                return;
            }
            OnBackRequest?.Invoke();
        }

        public void OnCGVisualisationToggle(bool isOn)
        {
            Debug.Log($"OnCGVisualisationToggle: {isOn}");
            OnCGVisualisation?.Invoke(_cgVisualisationToggle.isOn);
        }
        
        public void OnManualSTTToggle(bool isOn)
        {
            Debug.Log($"OnManualSTTToggle: {isOn}");
            OnManualSTT?.Invoke(_manualSTTToggle.isOn);
        }

        public void UpdateManualUIPanel(string feature, bool isActive)
        {
            switch (feature)
            {
                case AppFeatures.WORLD_VISUALISATION:
                    _cgVisualisationToggle.SetState(isActive);
                    break;
                case AppFeatures.MANUAL_STT_INSTRUCTION_PANEL:
                    _manualSTTToggle.SetState(isActive);
                    break;
            }
        }

        private void Update()
        {
            if (!IsShown) return;
            if (!Application.isEditor) return;
            if (Input.GetKeyDown(KeyCode.W))
            {
                _cgVisualisationToggle.isOn = !_cgVisualisationToggle.isOn;
            }
            else if (Input.GetKeyDown(KeyCode.Q))
            {
                _manualSTTToggle.isOn = !_manualSTTToggle.isOn;
            }
            else if (Input.GetKeyDown(KeyCode.S))
            {
                StopButtonClicked?.Invoke();
            }
            else if (Input.GetKeyDown(KeyCode.B))
            {
                OnBackClicked();
            }
        }

        public void SetRecordingState(RecordingState state)
        {
            switch (state)
            {
                case RecordingState.None:
                    ShowStopButton(true);
                    break;
                case RecordingState.Accepted:
                    ShowStopButton(true);
                    break;
                default:
                    ShowStopButton(false);
                    break;
            }
        }
        
        public void ShowStopButton(bool show)
        {
            _stopButton.enabled = show;
        }
        
        public void OnStopButtonClicked(PointerEvent pointerEvent)
        {
            if(!IsShown) return;
            if (PointerEventIsHand(pointerEvent, _disabledHandedness))
                return;
            StopButtonClicked?.Invoke();
            OnBackRequest?.Invoke();
        }
        
        public static bool PointerEventIsHand(PointerEvent pointerEvent, Handedness handedness)
        {
            var pokeInteractor = pointerEvent.Data as PokeInteractor;
            var handRef = pokeInteractor?.GetComponentInParent<HandRef>();
            return handRef != null && handRef.Hand.Handedness == handedness;
        }
        
        public void OnStopButtonClicked()
        {
            if(!IsShown) return;
            StopButtonClicked?.Invoke();
            OnBackRequest?.Invoke();
        }
        
        public void OnBackClicked()
        {
            if(!IsShown) return;
            OnBackRequest?.Invoke();
        }
    }
}