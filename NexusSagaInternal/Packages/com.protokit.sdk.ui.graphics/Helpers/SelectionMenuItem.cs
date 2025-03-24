using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class SelectionMenuItem : MonoBehaviour {
    public UnityEvent OnSelected;
    public UnityEvent OnDeselected;

    public void Select() {
        var t = transform.parent;
        while (t != null) {
            var selector = t.GetComponent<SelectionMenuController>();
            if (selector != null) {
                selector.OnSelectableChanged(this);
                break;
            }
            t = t.parent;
        }
        OnSelected?.Invoke();
    }

    public void Deselect() {
        OnDeselected?.Invoke();
    }
}
