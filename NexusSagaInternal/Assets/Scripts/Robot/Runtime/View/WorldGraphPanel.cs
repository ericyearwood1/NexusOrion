using System.Collections.Generic;
using Robot.Runtime.Utils;
using Robot.Runtime.View;
using TMPro;
using UnityEngine;
using HighlightState = Robot.Runtime.Data.HighlightState;

public class WorldGraphPanel : SimpleAnimatable
{
    [SerializeField] private CanvasGroup _canvasGroup;
    [SerializeField] private TMP_Text _roomLabel;
    [SerializeField] private Transform _objectListContainer;
    [SerializeField] private Transform _furnitureListContainer;
    private List<WorldGraphIcon> _activeObjects = new();
    private List<WorldGraphIcon> _activeFurniture = new();
    private HighlightPool<WorldGraphIcon> _iconPool;
    private string _roomName;

    public List<WorldGraphIcon> ActiveObjects => _activeObjects;
    public List<WorldGraphIcon> ActiveFurniture => _activeFurniture;

    public void Initialise(string roomName, HighlightPool<WorldGraphIcon> iconPool, Color color)
    {
        Debug.Log($"WorldGraph::Initialise: {roomName}");
        _roomName = roomName;
        if (!string.IsNullOrWhiteSpace(roomName))
        {
            _roomLabel.text = roomName.Replace("_", " ").ToTitleCase();
        }
        _roomLabel.color = color;
        _canvasGroup.alpha = 0;
        _iconPool = iconPool;
    }

    public void AddObject(WorldGraphIcon iconView)
    {
        Debug.Log($"WorldGraph::AddObject: {_roomName} | {iconView.Node.Id}");
        AddIconToContainer(_activeObjects, _objectListContainer, iconView);
    }

    public void AddFurniture(WorldGraphIcon iconView)
    {
        Debug.Log($"WorldGraph::AddFurniture: {_roomName} | {iconView.Node.Id}");
        AddIconToContainer(_activeFurniture, _furnitureListContainer, iconView);
        var isPanelVisible = _furnitureListContainer.gameObject.activeInHierarchy && _canvasGroup.alpha != 0;
        // if (!isPanelVisible || _state == HighlightState.Hidden || _state == HighlightState.None)
        // {
        //     iconView.ShowImmediate();
        // }
        // else
        // {
        //     iconView.Show();
        // }
        iconView.ShowImmediate();
    }
    
    private void AddIconToContainer(List<WorldGraphIcon> list, Transform container, WorldGraphIcon iconView)
    {
        Debug.Log($"WorldGraph::AddIconToContainer {_roomName} | {iconView.Node.Id} | {_state} | {container.gameObject.activeInHierarchy}");
        var iconTransform = iconView.transform;
        iconTransform.SetParent(container, false);
        iconTransform.localPosition = Vector3.zero;
        iconTransform.localScale = Vector3.one;
        iconTransform.localRotation = Quaternion.identity;
        list.Add(iconView);
    }

    public WorldGraphIcon RemoveFurniture(string itemId)
    {
        Debug.Log($"WorldGraph::RemoveFurniture {itemId}");
        foreach (var icon in _activeFurniture)
        {
            if (icon.Node.Id == itemId) return icon;
        }

        return null;
    }
    
    public WorldGraphIcon RemoveObject(string itemId)
    {
        Debug.Log($"WorldGraph::RemoveObject {itemId}");
        foreach (var icon in _activeObjects)
        {
            if (icon == null)
            {
                Debug.LogError($"Attempting to remove null icon {icon}");
            }
            Debug.Log($"WorldGraphTest::RemoveObject {icon.Node} | {icon.Node?.Id}");
            if (icon.Node == null || icon.Node.Id == itemId)
            {
                icon.IsDisposed = true;
                icon.HideImmediate();
                return icon;
            }
        }

        return null;
    }

    protected override void UpdateDisplay(float progress)
    {
        _canvasGroup.alpha = progress;
    }

    public void Tick()
    {
        for (var i = _activeObjects.Count - 1; i >= 0; --i)
        {
            var icon = _activeObjects[i];
            if (icon.IsDisposed && (icon.State == HighlightState.Hidden || icon.State == HighlightState.None))
            {
                _iconPool.Return(icon);
                _activeObjects.RemoveAt(i);
            }
        }
    }

    public void Reset()
    {
        _canvasGroup.alpha = 0;
        _activeObjects.Clear();
        _activeFurniture.Clear();
        _objectListContainer.DetachChildren();
        _furnitureListContainer.DetachChildren();
        
    }
}