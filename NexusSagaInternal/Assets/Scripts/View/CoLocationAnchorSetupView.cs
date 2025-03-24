using System;
using TMPro;
using UnityEngine;

namespace View
{
    public class CoLocationAnchorSetupView : SiroUIView
    {
        public Action OnOverrideAnchors;
        public Action OnUseExistingAnchors;

        [SerializeField] private TMP_Text _info;
        [SerializeField] private CanvasGroup _confirmOverrideView;
        [SerializeField] private CanvasGroup _processingAnchorsView;
        private CanvasGroup _currentView;

        public void OnUseExistingClicked()
        {
            OnUseExistingAnchors?.Invoke();
        }

        public void OnCreateNewAnchorsClicked()
        {
            OnOverrideAnchors?.Invoke();
        }

        public void ShowConfirmResetAnchors(string message)
        {
            _info.text = message;
            SwitchToView(_confirmOverrideView);
            Show();
        }

        public void ShowProcessingAnchorsView()
        {
            SwitchToView(_processingAnchorsView);
            Show();
        }
        
        private void SwitchToView(CanvasGroup canvasGroup)
        {
            TurnOff(_confirmOverrideView);
            TurnOff(_processingAnchorsView);
            TurnOn(canvasGroup);
        }

        private void TurnOff(CanvasGroup canvasGroup)
        {
            canvasGroup.alpha = 0;
            canvasGroup.interactable = false;
            canvasGroup.blocksRaycasts = false;
        }
        
        private void TurnOn(CanvasGroup canvasGroup)
        {
            canvasGroup.alpha = 1;
            canvasGroup.interactable = true;
            canvasGroup.blocksRaycasts = true;
            _currentView = canvasGroup;
        }

        public void ShowCreatingAnchorView()
        {
            SwitchToView(_processingAnchorsView);
        }

        public void ShowOverrideView()
        {
            SwitchToView(_confirmOverrideView);
        }

        public void ShowLoadingAnchorsView()
        {
            SwitchToView(_processingAnchorsView);
        }

        public void Update()
        {
            if (!Application.isEditor) return;
            if (_currentView == _confirmOverrideView)
            {
                if (Input.GetKeyDown(KeyCode.U))
                {
                    OnUseExistingClicked();
                }
                else if (Input.GetKeyDown(KeyCode.Space) || Input.GetKeyDown(KeyCode.O))
                {
                    OnCreateNewAnchorsClicked();
                }
            }
        }
    }
}