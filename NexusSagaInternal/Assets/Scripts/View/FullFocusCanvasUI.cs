using System.Collections.Generic;
using UnityEngine;
using View;

public class FullFocusCanvasUI : MonoBehaviour
{
    [SerializeField] private ConfigureUserDetailsView _editUserDetails;
    [SerializeField] private ConfirmExistingUserDetailsView _confirmExistingUserDetailsView;
    [SerializeField] private FatalErrorView _fatalError;
    [SerializeField] private ThreeDotRobotSyncView _syncWithRobotView;
    [SerializeField] private ConfigureWorldGraphPanelsView _configureWorldGraphPanelsView;
    [SerializeField] private CoLocationAnchorSetupView _coLocatingView;
    [SerializeField] private SiroUIView _joiningRoomView;
    [SerializeField] private SiroUIView _waitingForRobotHome;
    [SerializeField] private SiroUIView _waitingForWorldGraphAnchors;
    [SerializeField] private SiroUIView _creatingRobotHomeAnchorView;
    [SerializeField] private SiroUIView _retrievingPlatformDetails;
    [SerializeField] private SendInstructionsPanel _instructionRecordingView;
    [SerializeField] private ServiceConnectionView _serviceConnectionView;
    [SerializeField] private float _panelDistance = 3f;
    [SerializeField] private float _smoothTime = 1f;
    [SerializeField] private Transform _cameraTransform;
    [SerializeField] private OVRCameraRig _ovrCameraRig;
    private Vector3 _velocity = Vector3.zero;
    private SiroUIView _currentView;
    private List<SiroUIView> _allViews;

    private void Awake()
    {
        _allViews = new List<SiroUIView>
        {
            _editUserDetails,
            _confirmExistingUserDetailsView,
            _retrievingPlatformDetails,
            _fatalError,
            _syncWithRobotView,
            _coLocatingView,
            _joiningRoomView,
            _waitingForRobotHome,
            _waitingForWorldGraphAnchors,
            _creatingRobotHomeAnchorView,
            _instructionRecordingView
        };

        if (_cameraTransform != null) return;
        _cameraTransform = Camera.main.transform;
    }

    public void ForceUpdatePose()
    {
        var targetPosition = _cameraTransform.TransformPoint(new Vector3(0, 0, _panelDistance));
        targetPosition.y = _ovrCameraRig.centerEyeAnchor.position.y - 0.4f;

        var cameraPosition = _cameraTransform.transform.position;
        cameraPosition.y = targetPosition.y;

        var lookRotation = Quaternion.LookRotation(targetPosition - cameraPosition);
        transform.rotation = Quaternion.Slerp(transform.rotation, lookRotation, _smoothTime);
        transform.position = targetPosition;
    }
    private void LateUpdate()
    {
        if (_cameraTransform == null)
            return;

        var targetPosition = _cameraTransform.TransformPoint(new Vector3(0, 0, _panelDistance));
        targetPosition.y = _ovrCameraRig.centerEyeAnchor.position.y - 0.4f;

        var position = transform.position;
        position = Vector3.SmoothDamp(position, targetPosition, ref _velocity, _smoothTime);

        var cameraPosition = _cameraTransform.transform.position;
        cameraPosition.y = position.y;

        var lookRotation = Quaternion.LookRotation(position - cameraPosition);
        transform.rotation = Quaternion.Slerp(transform.rotation, lookRotation, _smoothTime);
        transform.position = position;
    }

    private void SwitchToView(SiroUIView nextView)
    {
        if (nextView == _currentView && nextView.IsShown) return;
        Debug.Log($"FullFocusCanvasUI::SwitchToView {nextView}");
        nextView.gameObject.SetActive(true);
        nextView.Show();
        if (nextView != _currentView)
        {
            if (_currentView != null)
            {
                _currentView.Hide();
                _currentView.gameObject.SetActive(false);
            } 
        }

        _currentView = nextView;
    }

    public void ShowFatalErrorState(string error)
    {
        _fatalError.Initialise(error);
        SwitchToView(_fatalError);
    }

    public ConfigureUserDetailsView ShowUserDetailsView()
    {
        SwitchToView(_editUserDetails);
        return _editUserDetails;
    }
    
    public void ShowRetrievingPlatformDetails()
    {
        SwitchToView(_retrievingPlatformDetails);
    }
    
    public ConfirmExistingUserDetailsView ShowConfirmExistingUserDetails()
    {
        SwitchToView(_confirmExistingUserDetailsView);
        return _confirmExistingUserDetailsView;
    }

    public CoLocationAnchorSetupView ShowCoLocationView()
    {
        SwitchToView(_coLocatingView);
        return _coLocatingView;
    }

    public void ShowJoiningRoomView()
    {
        SwitchToView(_joiningRoomView);
    }

    public void ShowWaitingForRobotHomeView()
    {
        SwitchToView(_waitingForRobotHome);
    }

    public ThreeDotRobotSyncView ShowSyncWithRobotView()
    {
        _syncWithRobotView.Initialise();
        SwitchToView(_syncWithRobotView);
        return _syncWithRobotView;
    }

    public SendInstructionsPanel ShowInstructionsPanel()
    {
        _instructionRecordingView.Initialise();
        SwitchToView(_instructionRecordingView);
        return _instructionRecordingView;
    }

    public void ShowCreatingRobotHomeAnchor()
    {
        SwitchToView(_creatingRobotHomeAnchorView);
    }

    public void HideAll()
    {
        foreach (var view in _allViews)
        {
            view.Hide();
        }
    }

    public ServiceConnectionView ShowServiceConnectionView(string savedIP)
    {
        _serviceConnectionView.Initialise(savedIP);
        SwitchToView(_serviceConnectionView);
        return _serviceConnectionView;
    }

    public ConfigureWorldGraphPanelsView ShowSetUpWorldGraphPanelsView()
    {
        _configureWorldGraphPanelsView.ShowInitialState();
        SwitchToView(_configureWorldGraphPanelsView);
        return _configureWorldGraphPanelsView;
    }

    public void ShowWaitingForWorldGraphAnchorsView()
    {
        SwitchToView(_waitingForWorldGraphAnchors);
    }
}