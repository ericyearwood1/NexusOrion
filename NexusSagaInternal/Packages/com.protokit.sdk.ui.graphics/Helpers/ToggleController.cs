using UnityEngine;
using UnityEngine.Events;

public class ToggleController : MonoBehaviour {

    private bool cachedIsOn;
    [SerializeField] private bool isOn;
    public bool IsOn {
        get {
            return isOn;
        }
        set {
            Set(value);
        }
    }

    public UnityEvent ToggleChanged = null;
    public UnityEvent ToggleOn = null;
    public UnityEvent ToggleOff = null;
    public bool DispatchOnStart = false;

    private void Start() {
        if (DispatchOnStart) {
            if (IsOn) ToggleOn?.Invoke(); else ToggleOff?.Invoke();
        }
    }

    public void Set(bool on) {
        if (isOn != on) ToggleChanged?.Invoke();
        isOn = on;
        if (IsOn) ToggleOn?.Invoke(); else ToggleOff?.Invoke();
    }

    public void SetOn() {
        if (!isOn) ToggleChanged?.Invoke();
        isOn = true;
        ToggleOn?.Invoke();
    }

    public void SetOff() {
        if (isOn) ToggleChanged?.Invoke();
        isOn = false;
        ToggleOff?.Invoke();
    }

    public void Toggle() {
        isOn = !isOn;
        ToggleChanged?.Invoke();
        if (IsOn) ToggleOn?.Invoke(); else ToggleOff?.Invoke();
    }

#if UNITY_EDITOR
    private void OnValidate() {
        if (cachedIsOn != isOn) {
            ToggleChanged?.Invoke();
            if (IsOn) ToggleOn?.Invoke(); else ToggleOff?.Invoke();
            cachedIsOn = isOn;
        }
    }
#endif
}
