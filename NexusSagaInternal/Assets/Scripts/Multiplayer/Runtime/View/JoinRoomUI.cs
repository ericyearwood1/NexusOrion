using System;
using System.Collections.Generic;
using Multiplayer.Runtime.Data;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class JoinRoomUI : MonoBehaviour
{
    public Action<UserType, string, int, string> OnJoinRoomRequest;
    public Action<string, int> OnUserDetailsUpdate;
    
    [SerializeField] private TMP_InputField _displayNameField;
    [SerializeField] private TMP_Dropdown _roomListDropdown;
    [SerializeField] private TMP_Dropdown _colorSelection;
    [SerializeField] private Button _guestJoinButton;
    [SerializeField] private Button _hostJoinButton;
    [SerializeField] private Button _updateUserButton;
    private string[] _roomList;

    private void Awake()
    {
        Disable();
    }
    
    public void Initialise(ColourOption[] colorOptions, string[] roomList, string displayName, int color)
    {
        _displayNameField.text = displayName;
        _roomList = roomList;
        if (_roomList == null || _roomList.Length == 0)
        {
            Debug.LogError("Invalid room list");
            return;
        }

        var options = new List<TMP_Dropdown.OptionData>(_roomList.Length);
        foreach (var roomId in _roomList)
        {
            options.Add(new TMP_Dropdown.OptionData(roomId));
        }
        
        _roomListDropdown.options = options;
        
        var optionList = new List<TMP_Dropdown.OptionData>();
        foreach (var colorOption in colorOptions)
        {
            optionList.Add(new TMP_Dropdown.OptionData(colorOption.Sprite));
        }
        _colorSelection.options = optionList;
        _colorSelection.value = color;
        _updateUserButton.gameObject.SetActive(false);
        ShowJoinRoomState();
        Enable();
    }

    private void Enable()
    {
        _roomListDropdown.enabled = true;
        if (_roomList == null || _roomList.Length == 0) return;
        _guestJoinButton.enabled = true;
        _hostJoinButton.enabled = true;
    }

    private void Disable()
    {
        _roomListDropdown.enabled = false;
        _guestJoinButton.enabled = false;
        _hostJoinButton.enabled = false;
    }

    public void OnUpdateUserDetails()
    {
        OnUserDetailsUpdate?.Invoke(_displayNameField.text, _colorSelection.value);
    }
    
    public void OnJoinAsGuest()
    {
        JoinRoom(UserType.Guest);
    }
    
    public void OnJoinAsHost()
    {
        JoinRoom(UserType.Host);
    }

    private void JoinRoom(UserType userType)
    {
        Disable(); 
        OnJoinRoomRequest?.Invoke(userType, _displayNameField.text, _colorSelection.value, _roomList[_roomListDropdown.value]);
    }

    public void ShowUserDetailsState()
    {
        _guestJoinButton.gameObject.SetActive(false);
        _hostJoinButton.gameObject.SetActive(false);
        _updateUserButton.gameObject.SetActive(true);
        _roomListDropdown.gameObject.SetActive(false);
    }
    
    private void ShowJoinRoomState()
    {
        _guestJoinButton.gameObject.SetActive(true);
        _hostJoinButton.gameObject.SetActive(true);
        _updateUserButton.gameObject.SetActive(false);
        _roomListDropdown.gameObject.SetActive(true);
    }

    public void Reset()
    {
        _roomListDropdown.options = new List<TMP_Dropdown.OptionData>();
        Disable();
    }
}
