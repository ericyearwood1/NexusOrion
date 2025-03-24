using System;
using System.Collections.Generic;
using Oculus.Interaction.Input;
using UnityEngine;

namespace View
{
    public class HandUI : MonoBehaviour
    {
        public Action<bool> OnManualSTTChange;
        public Action<bool> OnCGVisualisationChange;
        public RecordInstructionPanel RecordInstructionPanel;
        [SerializeField] private Canvas _canvas;
        [SerializeField] private ManualControlsPanel _manualControlsPanel;
        [SerializeField] private Transform _leftHandTransform;
        [SerializeField] private bool setVisibilityBasedOnAngle = true;
        [SerializeField] private bool _isFollowHandRotation = true;
        [SerializeField] private Vector3 localSpaceOffset = new Vector3( 0.075f,0.0425f, -0.005f);
        [SerializeField] private Vector3 _worldSpaceOffset = new Vector3( 0, 0, 0);
        [SerializeField] private Quaternion rotationOffset = new Quaternion(0.6403418f, 0.06676514f, -0.056022633f, 0.76312923f);
        
        private SiroUIView _currentView;
        private List<SiroUIView> _allViews;
        private Transform _mainCamera;
        private IHand _leftHand;
        private bool _isEnabled;

        public ManualControlsPanel ManualControlsPanel => _manualControlsPanel;

        public void Initialise(IHand hand, Transform cameraTransform)
        {
            Debug.Log($"HandUI::Initialise");
            _leftHand = hand;
            _allViews = new List<SiroUIView>
            {
                RecordInstructionPanel,
                _manualControlsPanel
            };
            _mainCamera = cameraTransform;
            RecordInstructionPanel.OnManualControlsRequested += OnManualControlsRequested;
            _manualControlsPanel.OnBackRequest += OnBackRequest;
            _manualControlsPanel.OnCGVisualisation += OnCGVisualisationChangeRequest;
            _manualControlsPanel.OnManualSTT += OnManualSTTChangeRequest;
            _currentView = RecordInstructionPanel;
            HideAll();
        }

        public void Enable()
        {
            _isEnabled = true;
            ShowCurrent();
        }

        public void Disable()
        {
            _isEnabled = false;
            HideAll();
        }

        private void OnCGVisualisationChangeRequest(bool isOn)
        {
            Debug.Log($"HandUI::OnCGVisualisationOnRequest");
            OnCGVisualisationChange?.Invoke(isOn);
        }

        private void OnManualSTTChangeRequest(bool isOn)
        {
            Debug.Log($"HandUI::OnManualSTTChangeRequest");
            OnManualSTTChange?.Invoke(isOn);
        }

        private void OnBackRequest()
        {
            Debug.Log($"HandUI::OnBackRequest");
            RecordInstructionPanel.Show();
            SwitchToView(RecordInstructionPanel);
        }

        private void OnManualControlsRequested()
        {
            Debug.Log($"HandUI::OnManualControlsRequested");
            _manualControlsPanel.Show();
            SwitchToView(_manualControlsPanel);
        }

        private void Update()
        {
            transform.position = _leftHandTransform.TransformPoint(localSpaceOffset) + _worldSpaceOffset;
            if (_isFollowHandRotation)
            {
                transform.rotation = _leftHandTransform.rotation * rotationOffset;
            }
            else
            {
                transform.rotation = rotationOffset;
            }

            if (!Application.isEditor && setVisibilityBasedOnAngle)
            {
                if (_isEnabled)
                {
                    var dot = Vector3.Dot(_canvas.transform.forward, _mainCamera.forward);
                    if (dot >= 0)
                    {
                        ShowCurrent();
                    }
                    else
                    {
                        HideAll();
                    }
                    
                }
                else
                {
                    HideAll();
                }
            }

            ListenForDebugInput();
        }

        private void ListenForDebugInput()
        {
            if (Input.GetKeyDown(KeyCode.B))
            {
                OnBackRequest();
            }
        }

        public RecordInstructionPanel ShowRecordInstructionPanel()
        {
            var recordInstructionPanel = RecordInstructionPanel;
            recordInstructionPanel.Show();
            SwitchToView(recordInstructionPanel);
            return recordInstructionPanel;
        }

        private void SwitchToView(SiroUIView nextView)
        {
            Debug.Log($"HandUI::SwitchToView {_currentView} | {nextView}");
            if (_currentView != null && _currentView != nextView)
            {
                _currentView.Hide();
            }

            _currentView = nextView;
        }

        public void HideAll()
        {
            foreach (var view in _allViews)
            {
                view.Hide();
            }
        }

        private void ShowCurrent()
        {
            if(!_isEnabled) return;
            if (_currentView != null)
            {
                _currentView.Show();
            }
        }

        public void SetFeatureButtons(bool isCGActive, bool isManualSTTActive)
        {
            _manualControlsPanel.Initialise(isCGActive, isManualSTTActive);
        }

        public void UpdateManualUIPanel(string feature, bool isActive)
        {
            _manualControlsPanel.UpdateManualUIPanel(feature, isActive);
        }
    }
}