using System;
using Oculus.Interaction;
using Oculus.Interaction.Input;
using Robot.Runtime.Data;
using TMPro;
using UIComponents.Runtime;
using UnityEngine;
using UnityEngine.UI;

namespace View
{
    public class RecordInstructionPanel : SiroUIView
    {
        public Action<bool> OnRecordButtonClicked;
        public Action ConfirmButtonClicked;
        public Action CancelButtonClicked;
        public Action StopButtonClicked;
        public Action TryAgainButtonClicked;
        public Action OnManualControlsRequested;
        [SerializeField] private DefaultInstructionsPanel _defaultInstructionsPanel;
        [SerializeField] private SimpleAnimatedCanvasGroup _audioRecordingPanel;
        [SerializeField] private SimpleAnimatedCanvasGroup _manualInstructionListPanel;
        [SerializeField] private Image _recordButtonImage;
        [SerializeField] private TMP_Text _titleText;
        [SerializeField] private TMP_Text _informationText;
        [SerializeField] private TMP_Text _transciptionText;
        [SerializeField] private TMP_Text _timingsText;
        
        [SerializeField] private Transform _recordingButton;
        [SerializeField] private Transform _confirmButtonTransform;
        [SerializeField] private Transform _cancelButtonTransform;
        [SerializeField] private Transform _manualControlsButtonTransform;
        [SerializeField] private Transform _stopButtonTransform;
        [SerializeField] private Transform _tryAgainButtonTransform;
        
        [SerializeField] private AudioVisualiser _audioVisualiser;
        [SerializeField] private RecordingTimer _recordingTimer;
        
        [SerializeField] private Sprite _defaultRecordButtonSprite;
        [SerializeField] private Sprite _recordingRecordButtonSprite;
        [SerializeField] private Sprite _processingRecordButtonSprite;
        [SerializeField] private Sprite _errorRecordButtonSprite;
        [SerializeField] private Sprite _thinkingRecordButtonSprite;
        [SerializeField] private Sprite _completeRecordButtonSprite;

        [SerializeField] private GameObject _recordingRingAnimation;
        
        [SerializeField] private Handedness _disabledHandedness = Handedness.Left;
        private bool _isShowManualInstructionList;
        
        private const string TITLE_DEFAULT = "Press to speak";
        private const string TITLE_RECORDING = "Recording...";
        private const string TITLE_PROCESSING = "Thinking...";
        private const string TITLE_TRANSCRIBED = "Is that right?";
        private const string TITLE_BUSY = "Executing";
        private const string TITLE_ERROR = "Error";
        private const string TITLE_SILENT = "Were you speaking?";
        private const string TITLE_CANCELLED = "Cancelled";
        private const string TITLE_CANCELLING = "Cancelling";
        private const string TITLE_COMPLETE = "Done!";
        
        private const string INFO_DEFAULT = "";
        private const string INFO_RECORDING = "";
        private const string INFO_PROCESSING = "";
        private const string INFO_BUSY = "";
        private const string INFO_ERROR = "Error recording audio";
        private const string INFO_SILENT = "We couldn't hear you";
        private const string INFO_CANCELLED = "Recording too short, hold for longer";
        private const string INFO_CANCELLING = "";
        private const string INFO_COMPLETE = "";


        
        private bool _buttonDown;
        private RecordingState _recordingState;
        public bool ShowTimings { get; set; }
        public DefaultInstructionsPanel DefaultInstructionsPanel => _defaultInstructionsPanel;

        public void Awake()
        {
            SetState(RecordingState.None);
            SetTranscription("");
        }
        
        private void ShowCancelButton(bool show)
        {
            _cancelButtonTransform.gameObject.SetActive(show);
            _informationText.gameObject.SetActive(!show);
        }

        private void ShowManualControlsButton(bool show)
        {
            _manualControlsButtonTransform.gameObject.SetActive(show);
            _informationText.gameObject.SetActive(!show);
        }
        
        private void ShowAudioVisualiser(bool show)
        {
            if (show)
                _audioVisualiser.Show();
            else
                _audioVisualiser.Hide();

            _titleText.gameObject.SetActive(!show);
        }
        
        private void ShowConfirmButton(bool show)
        {
            _confirmButtonTransform.gameObject.SetActive(show);
            _informationText.gameObject.SetActive(!show);
        }
        
        private void ShowStopButton(bool show)
        {
            // _stopButtonTransform.gameObject.SetActive(show);
            _informationText.gameObject.SetActive(!show);
        }
        
        private void ShowRecordingTimer(bool show)
        {
            _recordingTimer.gameObject.SetActive(show);
            _informationText.gameObject.SetActive(!show);
        }
        
        private void ShowTryAgainButton(bool show)
        {
            _tryAgainButtonTransform.gameObject.SetActive(show);
            _informationText.gameObject.SetActive(!show);
        }

        public void SetState(RecordingState state)
        {
            _timingsText.text = state.ToString();
            _timingsText.gameObject.SetActive(ShowTimings);
            ShowConfirmButton(false);
            ShowCancelButton(false);
            ShowStopButton(false);
            ShowTryAgainButton(false);
            ShowManualControlsButton(false);
            ShowAudioVisualiser(false);
            ShowRecordingTimer(false);
            _recordingState = state;
            if(!IsShown) return;
            switch (state)
            {
                case RecordingState.None:
                    _recordButtonImage.sprite = _thinkingRecordButtonSprite;
                    _titleText.text = TITLE_BUSY;
                    _informationText.text = INFO_BUSY;
                    _recordingRingAnimation.SetActive(false);
                    ShowManualControlsButton(true);
                    ShowStopButton(true);
                    break;
                case RecordingState.Ready:
                    _recordButtonImage.sprite = _defaultRecordButtonSprite;
                    _titleText.text = TITLE_DEFAULT;
                    _informationText.text = INFO_DEFAULT;
                    _recordingRingAnimation.SetActive(false);
                    ShowManualControlsButton(true);
                    SetTranscription("");
                    break;
                case RecordingState.Started:
                    _recordButtonImage.sprite = _recordingRecordButtonSprite;
                    _titleText.text = TITLE_RECORDING;
                    _informationText.text = INFO_RECORDING;
                    SetRecordingRatio(0);
                    ShowAudioVisualiser(true);
                    break;
                case RecordingState.Recording:
                    _recordButtonImage.sprite = _recordingRecordButtonSprite;
                    _titleText.text = TITLE_RECORDING;
                    _informationText.text = INFO_RECORDING;
                    ShowAudioVisualiser(true);
                    ShowRecordingTimer(true);
                    break;
                case RecordingState.Complete:
                    _recordButtonImage.sprite = _processingRecordButtonSprite;
                    _recordingRingAnimation.SetActive(true);
                    _titleText.text = TITLE_PROCESSING;
                    _informationText.text = INFO_PROCESSING;
                    ShowCancelButton(true);
                    break;
                case RecordingState.AwaitingTranscriptionConfirm:
                case RecordingState.Transcribed:
                    _recordButtonImage.sprite = _completeRecordButtonSprite;
                    _titleText.text = TITLE_TRANSCRIBED;
                    ShowCancelButton(true);
                    _recordingRingAnimation.SetActive(false);
                    break;
                case RecordingState.Cancelled:
                    _recordButtonImage.sprite = _errorRecordButtonSprite;
                    _titleText.text = TITLE_CANCELLED;
                    _informationText.text = INFO_CANCELLED;
                    _recordingRingAnimation.SetActive(false);
                    ShowTryAgainButton(true);
                    break;
                case RecordingState.Error:
                    _recordButtonImage.sprite = _errorRecordButtonSprite;
                    _titleText.text = TITLE_ERROR;
                    _informationText.text = INFO_ERROR;
                    _recordingRingAnimation.SetActive(false);
                    ShowTryAgainButton(true);
                    break;
                case RecordingState.Accepted:
                    _recordButtonImage.sprite = _completeRecordButtonSprite;
                    _titleText.text = TITLE_COMPLETE;
                    _informationText.text = INFO_COMPLETE;
                    _recordingRingAnimation.SetActive(false);
                    SetTranscription("");
                    ShowStopButton(true);
                    break;
                case RecordingState.Silent:
                    _recordButtonImage.sprite = _errorRecordButtonSprite;
                    _titleText.text = TITLE_SILENT;
                    _informationText.text = INFO_SILENT;
                    ShowTryAgainButton(true);
                    _recordingRingAnimation.SetActive(false);
                    _informationText.gameObject.SetActive(true);
                    break;
                case RecordingState.Cancelling:
                    _recordButtonImage.sprite = _thinkingRecordButtonSprite;
                    _titleText.text = TITLE_CANCELLING;
                    _informationText.text = INFO_CANCELLING;
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(state), state, null);
            }
        }


        public void OnPointerSelect(PointerEvent pointerEvent)
        {
            if(!IsShown) return;
            try
            {
                if (PointerEventIsHand(pointerEvent, _disabledHandedness))
                    return;
                OnRecordButtonDown();
            }
            catch (Exception e)
            {
                Debug.LogError(e);
            }
        }
        
        public void OnPointRelease(PointerEvent pointerEvent)
        {
            try
            {
                if (!IsShown) return;
                
                if (PointerEventIsHand(pointerEvent, _disabledHandedness))
                    return;
                OnRecordButtonUp();
            }
            catch (Exception e)
            {
                Debug.LogError(e);
            }
        }

        public static bool PointerEventIsHand(PointerEvent pointerEvent, Handedness handedness)
        {
            var pokeInteractor = pointerEvent.Data as PokeInteractor;
            var handRef = pokeInteractor?.GetComponentInParent<HandRef>();
            return handRef != null && handRef.Hand.Handedness == handedness;
        }
        
        public void OnRecordButtonDown()
        {
            Debug.Log("HandUITesting::OnRecordButtonDown");
            OnRecordButtonClicked?.Invoke(true);
        }

        public void OnRecordButtonUp()
        {
            Debug.Log("HandUITesting::OnRecordButtonUp");
            OnRecordButtonClicked?.Invoke(false); 
        }
        
        public void OnCancelButtonClicked(PointerEvent pointerEvent)
        {
            if(!IsShown) return;
            if (PointerEventIsHand(pointerEvent, _disabledHandedness))
                return;
            CancelButtonClicked?.Invoke();
        }
        
        public void OnStopButtonClicked(PointerEvent pointerEvent)
        {
            // if(!IsShown) return;
            // if (PointerEventIsHand(pointerEvent, _disabledHandedness))
            //     return;
            // StopButtonClicked?.Invoke();
        }
        
        public void OnTryAgainButtonClicked(PointerEvent pointerEvent)
        {
            if(!IsShown) return;
            if (PointerEventIsHand(pointerEvent, _disabledHandedness))
                return;
            TryAgainButtonClicked?.Invoke();
        }
        
        public void OnConfirmButtonClicked(PointerEvent pointerEvent)
        {
            if(!IsShown) return;
            if (PointerEventIsHand(pointerEvent, _disabledHandedness))
                return;
            ConfirmButtonClicked?.Invoke();
        }

        public void OnManualControlsClicked(PointerEvent pointerEvent)
        {
            if(!IsShown) return;
            if (PointerEventIsHand(pointerEvent, _disabledHandedness))
                return;
            OnManualControlsRequested?.Invoke();
        }

        public void SetRecordingRatio(float value)
        {
            _recordingTimer.SetValue(value);
        }
        public void SetTimings(string timings)
        {
            _timingsText.text = timings;
        }

        private void OnDestroy()
        {
            OnManualControlsRequested = null;
        }
        
        public override void Show()
        {
            if(IsShown) return;
            base.Show();
            if (!_isShowManualInstructionList)
            {
                _recordingButton.gameObject.SetActive(true);
            }
            SetState(RecordingState.None);
        }
        
        public override void Hide()
        {
            base.Hide();
            if(_recordingButton.gameObject.activeSelf) _recordingButton.gameObject.SetActive(false);
        }

        public void SetTranscription(string transcription)
        {
            _transciptionText.text = transcription;
        }

        public void ShowDefaultInstructions()
        {
            Debug.Log("RecordingInstructionPanel::ShowDefaultInstructions");
            _isShowManualInstructionList = true;
            _manualInstructionListPanel.gameObject.SetActive(true);
            _manualInstructionListPanel.ShowImmediate();
            _audioRecordingPanel.HideImmediate();
            _recordingButton.gameObject.SetActive(false);
            _audioRecordingPanel.gameObject.SetActive(false);
        }

        public void HideDefaultInstructions()
        {
            _isShowManualInstructionList = false;
            _manualInstructionListPanel.gameObject.SetActive(false);
            _manualInstructionListPanel.HideImmediate();
            _audioRecordingPanel.ShowImmediate();
            _audioRecordingPanel.gameObject.SetActive(true);
            if (IsShown)
            {
                _recordingButton.gameObject.SetActive(true);
            }
        }
        
        public void OnCancelButtonClicked()
        {
            if(!IsShown) return;
            CancelButtonClicked?.Invoke();
        }
        
        public void OnStopButtonClicked()
        {
            // if(!IsShown) return;
            // StopButtonClicked?.Invoke();
        }
        
        public void OnTryAgainButtonClicked()
        {
            if(!IsShown) return;
            TryAgainButtonClicked?.Invoke();
        }
        
        public void OnConfirmButtonClicked()
        {
            if(!IsShown) return;
            ConfirmButtonClicked?.Invoke();
        }

        public void OnManualControlsClicked()
        {
            Debug.Log($"OnManualControlsClicked:: {IsShown}");
            if(!IsShown) return;
            OnManualControlsRequested?.Invoke();
        }
        
        private void Update()
        {
            if (!Application.isEditor) return;
            if (!IsShown) return;
            if (Input.GetKeyDown(KeyCode.M))
            {
                OnManualControlsClicked();
            }
            switch (_recordingState)
            {
                case RecordingState.Ready:
                case RecordingState.None:
                {
                    if (Input.GetKeyDown(KeyCode.Space))
                    {
                        OnRecordButtonDown();
                    }

                    break;
                }
               
                case RecordingState.Started:
                case RecordingState.Recording:
                {
                    if (Input.GetKeyUp(KeyCode.Space))
                    {
                        OnRecordButtonUp();
                    }

                    break;
                }
                case RecordingState.Silent:
                {
                    if (Input.GetKeyUp(KeyCode.Space))
                    {
                        OnTryAgainButtonClicked();
                    }

                    break;
                }
                case RecordingState.AwaitingTranscriptionConfirm:
                {
                    if (Input.GetKeyUp(KeyCode.Space))
                    {
                        OnConfirmButtonClicked();
                    }
                    else if (Input.GetKeyUp(KeyCode.X))
                    {
                        OnCancelButtonClicked();
                    }

                    break;
                }
            }
        }
    }
}
