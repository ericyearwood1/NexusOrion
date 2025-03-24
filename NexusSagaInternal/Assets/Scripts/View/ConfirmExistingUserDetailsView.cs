using System;
using Multiplayer.Runtime.Config;
using Multiplayer.Runtime.Data;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace View
{
    public class ConfirmExistingUserDetailsView : SiroUIView
    {
        public Action OnConfirm;
        public Action OnEdit;
        [SerializeField] private TMP_Text _username;
        [SerializeField] private TMP_Text _userType;
        [SerializeField] private Image _userColor;

        public void Initialise(User user, ColorConfig config)
        {
            _username.text = user.DisplayName;
            _userType.text = $"{user.UserType}";
            _userColor.color = config.Options[user.Color].Colour;
        }

        private void Update()
        {
            if (!Application.isEditor) return;
            if (Input.GetKeyDown(KeyCode.Space) || Input.GetKeyDown(KeyCode.C))
            {
                OnConfirm?.Invoke();
            }
            else if (Input.GetKeyDown(KeyCode.E))
            {
               OnEdit?.Invoke();
            }
        }

        public void OnSubmitClicked()
        {
            OnConfirm?.Invoke();
        }

        public void OnEditClicked()
        {
            OnEdit?.Invoke();
        }

    }
}