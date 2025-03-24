using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class SelectionMenuController : MonoBehaviour {
    [System.Serializable]
    public class IntUnityEvent : UnityEvent<int> { }

    public Transform SelectorRoot;
    public bool AllowMultipleSelections = false;
    public bool DispatchOnStart = false;
    public SelectionMenuItem InitialSelection = null;

    public IntUnityEvent OnSelectionChanged;

    private void Start() {
        if (DispatchOnStart) {
            if (InitialSelection != null) {
                InitialSelection.Select();
            }
        }
    }

    public void Select(SelectionMenuItem item) {
        item.Select();
    }

    public void Select(int item) {
        if (SelectorRoot != null) {
            SelectorRoot.GetChild(item).GetComponent<SelectionMenuItem>().Select();
        }
    }

    public void OnSelectableChanged(SelectionMenuItem item) {
        // All Selection methods end up in this function, so only call this callback here.
        OnSelectionChanged?.Invoke(item.transform.GetSiblingIndex());

        // do not deselect if multiple selections are allowed
        if (AllowMultipleSelections) return;

        // otherwise only allow one selection at a time
        if (SelectorRoot != null) {
            foreach (Transform t in SelectorRoot) {
                var selectable = t.GetComponent<SelectionMenuItem>();
                if (selectable && selectable != item) {
                    selectable.Deselect();
                }
            }
        }
    }
}
