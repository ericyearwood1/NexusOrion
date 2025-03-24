using System;
using UnityEngine;

using CTRL;
using CTRL.Schemes;

public class EMGEventsLogger : ITK.DependBehavior
{
  [SerializeField]
  [ITK.Depend(Flags = ITK.DependFlags.Scene)]
  protected EMGEvents emgEvents;

  void OnEnable()
  {
    emgEvents.onEMGActionEvent.AddListener(OnEMGActionEvent);
  }

  void OnDisable()
  {
    emgEvents.onEMGActionEvent.RemoveListener(OnEMGActionEvent);
  }

  void OnEMGActionEvent(EMGAction emgAction)
  {
    Debug.Log($"EMGAction: {emgAction.action}, {emgAction.timing}");
  }
}
