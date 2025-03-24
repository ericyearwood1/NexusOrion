using System;
using System.Collections;
using TMPro;
using UnityEngine;
using UnityEngine.XR;

namespace ARGlasses.Interaction
{
    public class ARGlassesEyeTrackingDebug : MonoBehaviour
    {
        [SerializeField] private float _warmUpSeconds = 5;
        [SerializeField, ReadOnly] private ARGlassesEyeTrackingInitialization _initialization;
        private TMP_Text _errorText;

        private void Awake()
        {
            this.Scene(ref _initialization);
        }

        IEnumerator Start()
        {
            yield return new WaitForSeconds(_warmUpSeconds);
            _initialization.WhenErrorMessageChanged += HandleErrorMessage;
            HandleErrorMessage(_initialization.ErrorMessage);
        }

        private void HandleErrorMessage(string message)
        {
            var errorActive = !string.IsNullOrEmpty(message);
            if (errorActive)
            {
                if (!_errorText) _errorText = this.CreateHeadlockedErrorText();
                _errorText.SetText(message);
            }
            else
            {
                if (_errorText) Destroy(_errorText.gameObject);
            }
        }
    }
}
