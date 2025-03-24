using System;
using System.Collections.Generic;
using Multiplayer.Runtime.Config;
using Multiplayer.Runtime.Data;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using Random = UnityEngine.Random;

namespace View
{
    public class ConfigureUserDetailsView : SiroUIView
    {
        public Action OnUserDetailsConfirmed;
        [SerializeField] private TMP_InputField _usernameTextfield;
        [SerializeField] private TMP_Dropdown _roomSelectionDropDown;
        [SerializeField] private Toggle _hostToggle;
        [SerializeField] private Toggle _guestToggle;
        [SerializeField] private Toggle _observerToggle;
        [SerializeField] private SiroToggle[] _colorToggles;
        private User _user;
        private string _username;
        private int _colourIndex;

        public void Initialise(User user, ColorConfig colors, List<TMP_Dropdown.OptionData> roomList)
        {
            _user = user;
            InitialiseColours(user, colors);
            InitialiseInputFields(user, roomList);
        }

        private void InitialiseColours(User user, ColorConfig colors)
        {
            _colourIndex = user.Color == -1 ? 0 : user.Color;
            
            for (var i = 0; i < _colorToggles.Length; ++i)
            {
                var color = colors.Options[i];
                var toggle = _colorToggles[i];
                toggle.Background.color = color.Colour;
                toggle.SetIsOnWithoutNotify(i == _colourIndex);
            }
        }

        private void InitialiseInputFields(User user, List<TMP_Dropdown.OptionData> roomList)
        {
            _roomSelectionDropDown.options = roomList;
            _hostToggle.isOn = user.UserType == UserType.Host;
            _guestToggle.isOn = user.UserType == UserType.Guest;
            _observerToggle.isOn = user.UserType == UserType.Observer;
            _username = _user.DisplayName; 
            _usernameTextfield.text = _username;
        }

        public void OnSubmit()
        {
            Submit();
        }

        private void Update()
        {
            if (!Application.isEditor) return;
            if (_user == null) return;
            if (Input.GetKeyDown(KeyCode.Space) || Input.GetKeyDown(KeyCode.H))
            {
                _hostToggle.isOn = true;
                _guestToggle.isOn = false;
                _observerToggle.isOn = false;
                _user.UserType = UserType.Host;
                Submit();
            }
            else if (Input.GetKeyDown(KeyCode.G))
            {
                _hostToggle.isOn = false;
                _guestToggle.isOn = true;
                _observerToggle.isOn = false;
                _user.UserType = UserType.Guest;
                Submit();
            }
            else if (Input.GetKeyDown(KeyCode.R))
            {
                _usernameTextfield.text = $"User{Random.Range(0, 99999)}";
            }
            else if (Input.GetKeyDown(KeyCode.C))
            {
                _colourIndex = Random.Range(0, _colorToggles.Length);
                _colorToggles[_colourIndex].isOn = true;
                Debug.Log($"Input.GetKeyDown(KeyCode.C) {_colourIndex}");
            }
        }

        private void Submit()
        {
            _username = _usernameTextfield.text;
            SetUserType();
            SetUserColour();
            SavePreferences();
            
            OnUserDetailsConfirmed?.Invoke();
        }

        private void SetUserColour()
        {
            for (var i = 0; i < _colorToggles.Length; ++i)
            {
                var toggle = _colorToggles[i];
                Debug.Log($"SetUserColour {i} | {toggle.isOn}");
                if (!toggle.isOn) continue;
                _colourIndex = i;
                break;
            }
        }

        private void SetUserType()
        {
            if (_guestToggle.isOn)
            {
                _user.UserType = UserType.Guest;
            }
            else if (_hostToggle.isOn)
            {
                _user.UserType = UserType.Host;
            }
            else if (_observerToggle.isOn)
            {
                _user.UserType = UserType.Observer;
            }
        }

        private void SavePreferences()
        {
            Debug.Log($"Save Preferences {_username} | {_colourIndex} | {_roomSelectionDropDown.value}");
            _user.DisplayName = _username;
            _user.Color = _colourIndex;
            _user.Room = _roomSelectionDropDown.options[_roomSelectionDropDown.value].text;
            PlayerPrefs.SetString(Consts.PlayerPref_Username, _username);
            PlayerPrefs.SetInt(Consts.PlayerPref_Color, _colourIndex);
            PlayerPrefs.SetInt(Consts.PlayerPref_UserType, (int)_user.UserType);
        }
    }
}