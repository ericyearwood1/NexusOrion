using System;
using TMPro;
using UIComponents.Runtime;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;

namespace View
{
    public class ConfigureWorldGraphPanelsView : SiroUIView
    {
        public Action OnOverrideAnchorsRequested; 
        public Action OnReuseAnchorsRequested; 
        public Action OnNextPanelRequest; 
        public Action OnPreviousPanelRequest; 
        public Action OnSaveRequest;
        [FormerlySerializedAs("_continueButton")] [SerializeField] private Button _saveButton;
        [SerializeField] private Button _nextButton;
        [SerializeField] private Button _backButton;
        [SerializeField] private SimpleAnimatedCanvasGroup _overrideExistingAnchorsPanel;
        [SerializeField] private GeneralMessagePanel _generalMessagePanel;
        [SerializeField] private GeneralMessagePanel _errorMessagePanel;
        [SerializeField] private SimpleAnimatedCanvasGroup _savingAnchorsPanel;
        [SerializeField] private SimpleAnimatedCanvasGroup _editAnchorsPanel;
        [SerializeField] private SimpleAnimatedCanvasGroup _initialisingDataPanel;
        [SerializeField] private TMP_Text _roomText;
        [SerializeField] private TMP_Text _titleText;
        private State _currentState;
        private SimpleAnimatedCanvasGroup[] _views;
        private SimpleAnimatedCanvasGroup _currentView;
        private int _maxPanels;
        private bool _isExistingAnchor;
        private int _index;

        private enum State
        {
            None,
            InitialisingData,
            GeneralMessage,
            OverrideExistingAnchors,
            EditRoomList,
            Error,
            Saving
        }

        public void ShowInitialState()
        {
            _saveButton.gameObject.SetActive(false);
            _views = new[]
            {
                _generalMessagePanel,
                _errorMessagePanel,
                _overrideExistingAnchorsPanel,
                _savingAnchorsPanel,
                _editAnchorsPanel,
                _initialisingDataPanel
            };
            foreach (var view in _views)
            {
                view.HideImmediate();
            }
            ShowGeneralMessage("Initialising world graph...");
        }

        public void ShowErrorState(string errorMessage)
        {
            _errorMessagePanel.SetMessage(errorMessage);
            _currentState = State.Error;
            SwitchView(_errorMessagePanel);
        }

        public void ShowGeneralMessage(string message)
        {
            _generalMessagePanel.SetMessage(message);
            _currentState = State.GeneralMessage;
            SwitchView(_generalMessagePanel);
        }

        private void SwitchView(SimpleAnimatedCanvasGroup focusView)
        {
            if (focusView == _currentView) return;
            Debug.Log($"ConfigureWorldGraphPanelsState::SwitchView {focusView.name}");
            foreach (var view in _views)
            {
                if(view != focusView) view.Hide();
            }
            focusView.Show();
            _currentView = focusView;
        }

        public void ShowEditRoomPanel(int index, int maxPanels, string room, bool isExistingAnchor)
        {
            UpdateEditPanel(index, maxPanels, room, isExistingAnchor);
            SwitchView(_editAnchorsPanel);
        }

        private void UpdateEditPanel(int index, int maxPanels, string room, bool isExistingAnchor)
        {
            _isExistingAnchor = isExistingAnchor;
            _index = index;
            _maxPanels = maxPanels;
            _titleText.text = $"({index+1}/{maxPanels})";
            _roomText.text = room;
            _currentState = State.EditRoomList;
            _nextButton.enabled = isExistingAnchor && (index < maxPanels - 1);
            _backButton.enabled = index > 0;
            Debug.Log($"WorldGraphPanelFix::UpdateEditPanel:: {index} | {_backButton.enabled} | {_nextButton.enabled} | {maxPanels} | {isExistingAnchor}");
        }

        public void EnableSaveButton()
        {
            _saveButton.gameObject.SetActive(true);
            _saveButton.enabled = true;
        }

        public void EnableNavigation()
        {
            _nextButton.enabled = _index < _maxPanels - 1;
            _backButton.enabled = _index > 0;
        }
        
        public void DisableSaveButton()
        {
            _saveButton.gameObject.SetActive(false);
        }

        public void ShowOverrideView()
        {
            _currentState = State.OverrideExistingAnchors;
            SwitchView(_overrideExistingAnchorsPanel);
        }

        public void OnOverrideSelected()
        {
        Debug.Log("ConfigureWorldGraphPanelsState::OnOverrideSelected");
            OnOverrideAnchorsRequested?.Invoke();
        }

        public void OnReuseSelected()
        {
            Debug.Log("ConfigureWorldGraphPanelsState::OnReuseSelected");
            OnReuseAnchorsRequested?.Invoke();
        }

        public void OnBackClicked()
        {
            OnPreviousPanelRequest?.Invoke();
        }

        public void OnNextClicked()
        {
            OnNextPanelRequest?.Invoke();
        }

        public void OnSaveClicked()
        {
            _saveButton.enabled = false;
            OnSaveRequest?.Invoke();
        }

        public void Tick()
        {
            if (!Application.isEditor) return;

            switch (_currentState)
            {
                case State.OverrideExistingAnchors:
                    if (Input.GetKeyDown(KeyCode.Space) || Input.GetKeyDown(KeyCode.U))
                    {
                        OnReuseSelected();
                    }
                    else if (Input.GetKeyDown(KeyCode.O))
                    {
                        OnOverrideSelected();
                    }  
                    break;
                case State.EditRoomList:
                    if (_backButton.enabled && Input.GetKeyDown(KeyCode.LeftArrow))
                    {  
                        OnBackClicked();
                    }
                    else if (_nextButton.enabled && Input.GetKeyDown(KeyCode.RightArrow))
                    {
                        OnNextClicked();
                    }
                    else if (_saveButton.gameObject.activeSelf && _saveButton.enabled && (Input.GetKeyDown(KeyCode.S) || Input.GetKeyDown(KeyCode.Space)))
                    {
                        OnSaveClicked();
                    }
                    break;
            }
        }

        public void ShowSaveView()
        {
            _currentState = State.Saving;
            SwitchView(_savingAnchorsPanel);
        }

        public void ShowInitialisingDataView()
        {
            _currentState = State.InitialisingData;
            SwitchView(_initialisingDataPanel);
        }
    }
}