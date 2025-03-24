using System;
using Multiplayer.Runtime.Data;
using TMPro;
using UnityEngine;

namespace View
{
    public class ServiceConnectionView : SiroUIView
    {
        public Action<string> OnAddressEntered;
        [SerializeField] private CanvasGroup _ipAddressForm;
        [SerializeField] private CanvasGroup _connectingState;
        [SerializeField] private CanvasGroup _errorConnectingState;
        [SerializeField] private CanvasGroup _autoDiscoveryState;
        [SerializeField] private TMP_InputField _ipAddressField;
        [SerializeField] private TMP_Text _errorLabel;
        private string _defaultIp;

        public void Initialise(string savedIP)
        {
            _defaultIp = savedIP;
            ResetForm();
        }

        private void ResetForm()
        {
            _ipAddressField.SetTextWithoutNotify(_defaultIp);
            _errorLabel.gameObject.SetActive(false);
        }

        public void OnAddressConfirm()
        {
            var address = _ipAddressField.text;
            if (!Application.isEditor)
            {
                if (string.IsNullOrWhiteSpace(address))
                {
                    ShowErrorMessage();
                    return;
                }

                var components = address.Split('.');
                if (components.Length != 4)
                {
                    ShowErrorMessage();
                    return;
                }

                foreach (var component in components)
                {
                    if (int.TryParse(component, out var number)) continue;
                    ShowErrorMessage();
                    return;
                }
            }
            
            ShowState(_connectingState);
            HideState(_ipAddressForm);
            HideState(_errorConnectingState);
            HideState(_autoDiscoveryState);
            OnAddressEntered?.Invoke(_ipAddressField.text);
        }

        private void ShowForm()
        {
            _ipAddressField.enabled = true;
            HideState(_connectingState);
            ShowState(_ipAddressForm);
            HideState(_errorConnectingState);
            HideState(_autoDiscoveryState);
        }

        private void ShowErrorMessage()
        {
            _ipAddressField.enabled = true;
            _ipAddressField.text = string.Empty;
            _errorLabel.text = "Not a valid ip address";
            _errorLabel.gameObject.SetActive(true);
        }

        public void ShowIPAddressForm()
        {
            ResetForm();
            ShowForm();
        }
        
        public void ShowConnectionErrorState()
        {
            HideState(_connectingState);
            HideState(_ipAddressForm);
            ShowState(_errorConnectingState);
            HideState(_autoDiscoveryState);
        }

        public void ShowConnectingToServiceState()
        {
            ShowState(_connectingState);
            HideState(_ipAddressForm);
            HideState(_errorConnectingState);
            HideState(_autoDiscoveryState);
        }
        

        public void ShowRetrievingServiceDetailsState()
        {
            ShowState(_autoDiscoveryState);
            HideState(_ipAddressForm);
            HideState(_errorConnectingState);
            HideState(_connectingState);
        }

        private void ShowState(CanvasGroup group)
        {
            group.alpha = 1;
            group.interactable = true;
            group.blocksRaycasts = true;
        }
        
        private void HideState(CanvasGroup group)
        {
            group.alpha = 0;
            group.interactable = false;
            group.blocksRaycasts = false;   
        }

        private void Update()
        {
            if (!Application.isEditor) return;
            if (Input.GetKeyDown(KeyCode.Space))
            {
                OnAddressConfirm();
            }
        }
    }
}