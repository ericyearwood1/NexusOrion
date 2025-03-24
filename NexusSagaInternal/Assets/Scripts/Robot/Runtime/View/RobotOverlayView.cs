using System.Collections.Generic;
using Robot.Runtime.Data.Planner;
using Robot.Runtime.Messages.Server.Planner;
using Robot.Runtime.View;
using TMPro;
using UIComponents.Runtime;
using UnityEngine;

public class RobotOverlayView : MonoBehaviour
{
    private const float TimeCompleteToReady = 4;
    [SerializeField] private Transform _actionListContainer;
    [SerializeField] private Transform _HARListContainer;
    [SerializeField] private GameObject _labelListItemPrefab;
    [SerializeField] private GameObject _HARListItemPrefab;
    [SerializeField] private TMP_Text _feedbackText;
    [SerializeField] private SimpleAnimatedCanvasGroup _completeState;
    [SerializeField] private SimpleAnimatedCanvasGroup _inProgressState;
    [SerializeField] private SimpleAnimatedCanvasGroup _thinkingState;
    [SerializeField] private SimpleAnimatedCanvasGroup _readyState;
    [SerializeField] private SimpleAnimatedCanvasGroup _failedState;
    [SerializeField] private SimpleAnimatedCanvasGroup _replanningState;
    [SerializeField] private SimpleAnimatedCanvasGroup _notUnderstoodState;
    [SerializeField] private SimpleAnimatedCanvasGroup _humanActionState;
    private float _currentTransitionTime = 0;
    private float _baseCanvasScale;
    private Transform _cameraTransform;
    private CanvasGroup _transitionFrom;
    private bool _isTransitionToReady;
    private Queue<LabelListItem> _listItemPool = new();
    private List<LabelListItem> _activeListItems = new();

    private SimpleAnimatedCanvasGroup[] _views;
    private SimpleAnimatedCanvasGroup _currentView;
    // Variables to track the current container and prefab
    private Transform _currentActionListContainer;
    private GameObject _currentLabelListItemPrefab;

    private PlannerData plannerDataInstance;
    private bool isHARStateActive = false;
    private float HARStateTimer = 0f;

    private PlannerData _storedPlannerData;

    // Initialize the default container and prefab
    private void Awake()
    {
        _currentActionListContainer = _actionListContainer;
        _currentLabelListItemPrefab = _labelListItemPrefab;
    }

    public void Initialise(Transform cameraTransform)
    {
        _cameraTransform = cameraTransform;
        _views = new[]
        {
            _completeState,
            _inProgressState,
            _thinkingState,
            _readyState,
            _failedState,
            _replanningState,
            _notUnderstoodState,
            _humanActionState
        };
        _currentView = _readyState;
        HideAllViewsImmediately();
        for (var i = 0; i < 10; ++i)
        {
            var itemGO = Instantiate(_labelListItemPrefab, _actionListContainer, false);
            itemGO.transform.localScale = Vector3.one;
            itemGO.transform.localRotation = Quaternion.identity;
            itemGO.transform.localPosition = Vector3.zero;
            var item = itemGO.GetComponent<LabelListItem>();
            itemGO.SetActive(false);
            _listItemPool.Enqueue(item);
        }
    }

    public void Show()
    {
        _currentView.Show();
    }
    // Switch the action list container action list or HAR list based on the prefab
    public void SetActionListContainer(int containerIndex)
    {
        switch (containerIndex)
        {
            case 0:
                _currentActionListContainer = _actionListContainer;
                _currentLabelListItemPrefab = _labelListItemPrefab;
                break;

            case 1:
                _currentActionListContainer = _HARListContainer;
                _currentLabelListItemPrefab = _HARListItemPrefab;
                break;
            default :
                Debug.Log("Encountered Default Case");
                break;
        }

        // After setting the container, reset the list items
        ResetListItems();
    }

    // Reset the action list items to the currently active container
    private void ResetListItems()
    {
        // Clear active list items and reset
        foreach (var activeListItem in _activeListItems)
        {
            _listItemPool.Enqueue(activeListItem);
            activeListItem.gameObject.SetActive(false);
        }
        _activeListItems.Clear();

        // Reinstantiate the items using the correct prefab and container
        for (var i = 0; i < 10; ++i)
        {
            var itemGO = Instantiate(_currentLabelListItemPrefab, _currentActionListContainer, false);
            itemGO.transform.localScale = Vector3.one;
            itemGO.transform.localRotation = Quaternion.identity;
            itemGO.transform.localPosition = Vector3.zero;
            var item = itemGO.GetComponent<LabelListItem>();
            itemGO.SetActive(false);
            _listItemPool.Enqueue(item);
        }
    }

    // HAR method - handle human action state
    public void HAR(PlannerData plannerData)
    {
        CancelReadyTransition();
        // Set action list container to HAR (1 for the second container)
        SetActionListContainer(1); // Use the HAR list container (set to 1 for now)
        var currentAction = plannerData.humanActionData;
        SwitchToView(_humanActionState);
        _activeListItems.Clear();
        Debug.Log($"THE ACTION AND MESAGES ARE {currentAction.Action}");
        AddActionListItem(currentAction.Action, currentAction.DisplayMessage, false);
    }


    public void UpdateState(PlannerData plannerData)
    {
        Debug.Log($"PlannerMessageHandler::UpdateState:{plannerData.State} | {plannerData.CurrentAction}");
        var currentAction = plannerData.CurrentAction;
        switch (plannerData.State)
        {
            case PlannerState.PlannerBusy:
                var action = plannerData.CurrentAction;
                if (action.ActionType == PlannerActionType.Compound && action.ChildActions.Count == 0 ||
                    action.ActionType != PlannerActionType.Compound && string.IsNullOrWhiteSpace(action.Action)) return;
                SetNextActionInfo(plannerData);
                ConfigureListColours(currentAction);
                SwitchToView(_inProgressState);
                break;
            case PlannerState.ReceivedInstruction:
            case PlannerState.SendingInstruction:
                ShowThinkingState();
                break;
            case PlannerState.ServiceUnavailable:
                HideAllViews();
                break;
        }
    }
    
    private void HideAllViewsImmediately()
    {
        Debug.Log($"HideAllViewsImmediately {_views}");
        foreach (var view in _views)
        {
            view.HideImmediate();
        }
    }

    private void HideAllViews()
    {
        foreach (var view in _views)
        {
            view.Hide();
        }
    }

    private void SwitchToView(SimpleAnimatedCanvasGroup view)
    {
        Debug.Log($"PlannerMessageHandler::SwitchToView:{view}");
        if (_currentView != null && _currentView != view)
        {
            _currentView.Hide();
        }

        view.Show();

        _currentView = view;
    }

    private void ConfigureListColours(NextActionData currentAction)
    {
        if (currentAction.ActionType != PlannerActionType.Compound) return;
        for (var i = 0; i < _activeListItems.Count; ++i)
        {
            if (currentAction.ChildActionIndex == i) _activeListItems[i].SetActionActive();
            else _activeListItems[i].SetActionInactive();
        }
    }

    public void SetNextActionInfo(PlannerData plannerData)
    {
        Debug.Log($"PlannerMessageHandler::SetNextActionInfo:{plannerData.State} | {plannerData.CurrentAction}");
        //@TODO clean this up once have final designs 
        var currentAction = plannerData.CurrentAction;
        //Resetting ActionListContainer to display planner actions
        SetActionListContainer(0);

        foreach (var activeListItem in _activeListItems)
        {
            _listItemPool.Enqueue(activeListItem);
            activeListItem.gameObject.SetActive(false);
        }

        _activeListItems.Clear();
        if (currentAction != null)
        {
            if (currentAction.ActionType == PlannerActionType.Compound)
            {
                var children = currentAction.ChildActions;
                for (var i = 0; i < children.Count; ++i)
                {
                    var childAction = children[i];
                    AddActionListItem(childAction.Action, childAction.DisplayMessage, i == currentAction.ChildActionIndex);
                }
            }
            else
            {
                AddActionListItem(currentAction.Action, currentAction.DisplayMessage, false);
            }
        }
    }

    private void AddActionListItem(string action, string displayMessage, bool showActiveColour)
    {
        var activeListItem = _listItemPool.Dequeue();
        activeListItem.gameObject.SetActive(true);
        _activeListItems.Add(activeListItem);
        var actionMessage = string.IsNullOrWhiteSpace(displayMessage) ? action : displayMessage; 
        var label = actionMessage?.Replace("[", "\n[");
        activeListItem.SetLabel(label);
        if (showActiveColour)
        {
            activeListItem.SetActionActive();
        }
        else
        {
            activeListItem.SetActionInactive();
        }
        //reswitching the action container to the parent 
        activeListItem.transform.SetParent(_currentActionListContainer, false);

    }

    private void Update()
    {
        DoCompleteToReadyTransition();
    }

    private void DoCompleteToReadyTransition()
    {
        if (!_isTransitionToReady) return;
        _currentTransitionTime += Time.deltaTime;
        if (_currentTransitionTime > TimeCompleteToReady)
        {
            _currentTransitionTime = TimeCompleteToReady;
            _isTransitionToReady = false;
            SwitchToView(_readyState);
        }
    }

    public void ShowThinkingState()
    {
        CancelReadyTransition();
        SwitchToView(_thinkingState);
    }

    public void ShowInstructionComplete()
    {
        CancelReadyTransition();
        TransitionToReadyState();
        
    }

    public void ShowInstructionFailed()
    {
        CancelReadyTransition();
        SwitchToView(_failedState);
        TransitionToReadyState();
    }
    
    public void ShowInstructionNotUnderstood()
    {
        CancelReadyTransition();
        SwitchToView(_notUnderstoodState);
        TransitionToReadyState();
    }

    public void ShowReadyState()
    {
        SwitchToView(_readyState);
    }

    private void TransitionToReadyState()
    {
        _currentTransitionTime = 0;
        _isTransitionToReady = true;
    }

    public void UpdateActionFeedback(ActionFeedbackMessage feedback)
    {
    }

    public void ShowReplanning()
    {
        CancelReadyTransition();
        SwitchToView(_replanningState);
    }

    private void CancelReadyTransition()
    {
        if (!_isTransitionToReady) return;
        _isTransitionToReady = false;
        _currentTransitionTime = 0;
        HideAllViews();
    }

}