using System;
using UnityEngine;

using CTRL;
using CTRL.Schemes;

public class UIEventsLogger : ITK.DependBehavior
{
  [SerializeField]
  [ITK.Depend(Flags = ITK.DependFlags.Scene)]
  protected UIEvents uiEvents;

  [SerializeField]
  [ITK.Depend(Flags = ITK.DependFlags.Scene | ITK.DependFlags.Optional)]
  protected ContinuousUIEvents continuousUIEvents;

  void OnEnable()
  {
    uiEvents.OnButtonState.AddListener(OnButtonState);
    uiEvents.OnButtonAction.AddListener(OnButtonAction);
    uiEvents.OnDPadAction.AddListener(OnDPadAction);
    uiEvents.OnVertical1DAction.AddListener(OnVertical1DAction);
    uiEvents.OnShortcutAction.AddListener(OnShortcutAction);
    if(continuousUIEvents) continuousUIEvents.OnContinuousAction.AddListener(OnContinuousAction);
  }

  void OnDisable()
  {
    uiEvents.OnButtonState.RemoveListener(OnButtonState);
    uiEvents.OnButtonAction.RemoveListener(OnButtonAction);
    uiEvents.OnDPadAction.RemoveListener(OnDPadAction);
    uiEvents.OnVertical1DAction.RemoveListener(OnVertical1DAction);
    uiEvents.OnShortcutAction.RemoveListener(OnShortcutAction);
    if(continuousUIEvents) continuousUIEvents.OnContinuousAction.RemoveListener(OnContinuousAction);
  }

  void OnButtonState(ButtonState buttonState)
  {
    Debug.Log($"ButtonState: {buttonState.button}, {buttonState.timing}");
  }

  void OnButtonAction(ButtonAction buttonAction)
  {
    Debug.Log($"ButtonAction: {buttonAction.button}, {buttonAction.action}, {buttonAction.timing}");
  }

  void OnDPadAction(DPadAction dPadAction)
  {
    Debug.Log($"DPadAction: {dPadAction.action}");
  }

  void OnVertical1DAction(Vertical1DAction vertical1DAction)
  {
    Debug.Log($"Vertical1DAction: {vertical1DAction.action}");
  }

  void OnShortcutAction(ShortcutAction shortcutAction)
  {
    Debug.Log($"ShortcutAction: {shortcutAction.action}");
  }

  void OnContinuousAction(ContinuousAction continuousAction)
  {
    Debug.Log($"ContinuousAction: {continuousAction.action}, {string.Join(", ", continuousAction.axes)}");
  }
}
