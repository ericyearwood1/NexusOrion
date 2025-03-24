using System;
using Robot.Runtime.Data.WorldGraph;
using TMPro;
using UnityEngine;

public class WorldGraphPanelListView : MonoBehaviour
{
    public Action<WorldGraphPanelData> OnEditRequest;
    
    [SerializeField] private TMP_Text _roomName;
    private WorldGraphPanelData _data;

    public void Initialise(WorldGraphPanelData data)
    {
        _data = data;
        Refresh();
    }

    public void Refresh()
    {
        _roomName.text = _data.RoomName;
    }

    public void OnEditClicked()
    {
        OnEditRequest?.Invoke(_data);
    }
}
