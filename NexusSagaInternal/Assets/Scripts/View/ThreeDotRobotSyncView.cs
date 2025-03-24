using System;
using System.Collections.Generic;
using Oculus.Interaction.Input;
using UnityEngine;

namespace View
{
    public class ThreeDotRobotSyncView : SiroUIView
    {
        public Action<Vector3, Quaternion> OnComplete;
        [SerializeField] private HandRef _rightHandRef;
        [SerializeField] private HandRef _leftHandRef;
        [SerializeField] private GameObject _pinchMarkerPrefab;
        [SerializeField] private float PinchTriggerTime = 0.5f;
        [SerializeField] protected GameObject _demoCube;
        [SerializeField] protected Transform _rotationOffset;
        [SerializeField] private CanvasGroup _confirmPanel;
        [SerializeField] private CanvasGroup[] _instructions;
        private int _maxTaps = 3;
        private Vector3[] _dummyMarkers = new[] { new Vector3(3, 1, 1), new Vector3(4, 1, 1), new Vector3(4, 1, 2) };
        private GameObject _marker;
        private int _currentNumberTaps;
        private List<Transform> _markers = new List<Transform>();
        private PinchState _pinchState;

        private enum SyncState
        {
            None,
            PlacingMarkers,
            Confirm
        }

        private enum PinchState
        {
            None,
            ReadyForPinch,
            Pinching,
            WaitingForRelease
        }
        
        private GameObject _syncMarker;
        private bool _isPinching;
        private float _currentPinchTime;
        private SyncState _currentState;
        private bool _isInitialised;

        public void Initialise()
        {
            ResetView();
            ResetState();
            CreateMarker();
            _isInitialised = true;
        }

        private void ResetView()
        {
            _syncMarker = new GameObject("Robot Sync Markers")
            {
                transform =
                {
                    position = Vector3.zero,
                    rotation = Quaternion.identity
                }
            };
            _demoCube.SetActive(false);
            _rotationOffset.gameObject.SetActive(false);
            foreach (var marker in _markers)
            {
                Destroy(marker.gameObject);
            }
            _markers.Clear();
            if (_marker != null)
            {
                Destroy(_marker.gameObject);
            }
        }

        private void ResetState()
        {
            _maxTaps = 3;
            _currentNumberTaps = 0;
            _pinchState = PinchState.ReadyForPinch;
            Hide(_confirmPanel);
            _currentPinchTime = 0f;
            _isPinching = false;
            ShowInstructionPanel(0);
            _demoCube.gameObject.SetActive(false);
            _rotationOffset.gameObject.SetActive(false);
            _currentState = SyncState.PlacingMarkers;
        }

        private void ShowInstructionPanel(int index)
        {
            foreach (var canvasGroup in _instructions)
            {
                canvasGroup.alpha = 0;
            }

            _instructions[index].alpha = 1;
        }

        private void Update()
        {
            if (!_isInitialised) return;
            switch (_currentState)
            {
                case SyncState.PlacingMarkers: 
                    if (_marker == null) return;
                    CheckForPlaceMarker();
                    break;
                case SyncState.Confirm: 
                    CheckEditorConfirmPanelInput();
                    break;
            }
        }

        private void CheckEditorConfirmPanelInput()
        {
            if (!Application.isEditor) return;
            if (_confirmPanel.alpha < 1) return;
            if (Input.GetKeyDown(KeyCode.R))
            {
                OnRedoClicked();
            }
            else if (Input.GetKeyDown(KeyCode.Space) || Input.GetKeyDown(KeyCode.C))
            {
                OnConfirmClicked();
            }
        }

        private void CheckForPlaceMarker()
        {
            switch (_pinchState)
            {
                case PinchState.ReadyForPinch : 
                    if (_rightHandRef.GetJointPose(HandJointId.HandIndexTip, out var pose))
                    {
                        _marker.transform.position = pose.position;
                    }
                    if (Input.GetKeyDown(KeyCode.Space))
                    {
                        OnPinch();
                    }
                    if (IsPlaceMarkerGesture())
                    {
                        Debug.Log("ThreeDotRobotSyncView::Is pinching true");
                        _isPinching = true;
                        _pinchState = PinchState.Pinching;
                        _currentPinchTime = 0f;
                    }
                    break;
                case PinchState.Pinching :
                    if(IsPlaceMarkerGesture())
                    {
                        _currentPinchTime += Time.deltaTime;
                        if (_currentPinchTime >= PinchTriggerTime)
                        {
                            OnPinch();
                        }
                
                    }
                    else
                    {
                        _isPinching = false;
                        _currentPinchTime = 0f;
                        _pinchState = PinchState.ReadyForPinch;
                    }
                    break;
                case PinchState.WaitingForRelease:
                    if (!IsPlaceMarkerGesture())
                    {
                        _pinchState = PinchState.ReadyForPinch;
                    }
                    break;
                default:
                    _currentPinchTime = 0f;
                    break;
            }
            // if (!_isPinching)
            // {
            //     if (_rightHandRef.GetJointPose(HandJointId.HandIndexTip, out var pose))
            //     {
            //         _marker.transform.position = pose.position;
            //     }
            //
            //     if (_pinchState != PinchState.ReadyForPinch) return;
            //     if (Input.GetKeyDown(KeyCode.Space))
            //     {
            //         OnPinch();
            //     }
            //     if (IsPlaceMarkerGesture())
            //     {
            //         Debug.Log("ThreeDotRobotSyncView::Is pinching true");
            //         _isPinching = true;
            //         _currentPinchTime = 0f;
            //     }
            //     else
            //     {
            //         _currentPinchTime = 0f;
            //         _isPinching = false;
            //     }
            // }
            // else if(IsPlaceMarkerGesture())
            // {
            //     _currentPinchTime += Time.deltaTime;
            //     if (_currentPinchTime >= PinchTriggerTime)
            //     {
            //         OnPinch();
            //     }
            //     
            // }
            // else
            // {
            //     _isPinching = false;
            //     _currentPinchTime = 0f;
            // }
        }

        private bool IsPlaceMarkerGesture()
        {
            return _rightHandRef.GetIndexFingerIsPinching() || _leftHandRef.GetIndexFingerIsPinching() ||
                   Input.GetKey(KeyCode.P);
        }

        private void CreateMarker()
        {
            Debug.Log("ThreeDotRobotSyncView::Create marker");
            _marker = Instantiate(_pinchMarkerPrefab, _syncMarker.transform);
        }

        public void OnRedoClicked()
        {
            Debug.Log("ThreeDotRobotSyncView::OnRedoClicked");
            ResetView();
            ResetState();
            CreateMarker();
        }

        private void OnPinch()
        {
            Debug.Log($"ThreeDotRobotSyncView::OnPinch {_currentNumberTaps}");
#if UNITY_EDITOR
            _marker.transform.position = _dummyMarkers[_currentNumberTaps];
#endif
            _markers.Add(_marker.transform);
            _currentNumberTaps++;
            _marker = null;
            if (_currentNumberTaps < _maxTaps)
            {
                ShowInstructionPanel(_currentNumberTaps);
            }

            if (_currentNumberTaps < _maxTaps)
            {
                CreateMarker();
            }
            else 
            {
                _currentState = SyncState.Confirm;
                SetBox();
                Show(_confirmPanel);
            }
            _isPinching = false;
            _pinchState = PinchState.WaitingForRelease;
        }

        private void Show(CanvasGroup group)
        {
            group.alpha = 1;
            group.interactable = true;
            group.blocksRaycasts = true;
        }
        
        private void Hide(CanvasGroup group)
        {
            group.alpha = 0;
            group.interactable = false;
            group.blocksRaycasts = false;
        }

        public void OnConfirmClicked()
        {
            var position = _rotationOffset.position;
            var rotation = _rotationOffset.rotation.eulerAngles;
            rotation.x = rotation.z = position.y = 0;
            _demoCube.SetActive(false);
            OnComplete?.Invoke(position, Quaternion.Euler(rotation));
        }

        private void SetBox()
        {
            var backLeft = _markers[0].position;
            var backRight = _markers[1].position;
            var frontRight = _markers[2].position;
            var yPos = (backLeft.y + frontRight.y) / 2f;
            backLeft.y = backRight.y = frontRight.y = 0f;

            var center = (frontRight + backLeft) / 2f;
            var backCenter = (backRight + backLeft) / 2f;
            var direction = (center - backCenter).normalized;
            center.y = yPos / 2f;

            _rotationOffset.localPosition = center;
            _rotationOffset.forward = direction;
            var scale = new Vector3(Vector3.Distance(backLeft, backRight), yPos,
                Vector3.Distance(frontRight, backRight));
            
            _demoCube.transform.localScale = scale;
            _demoCube.gameObject.SetActive(true);
            _rotationOffset.gameObject.SetActive(true);
        }

        public void Reset()
        {
            ResetView();
        }
    }
}