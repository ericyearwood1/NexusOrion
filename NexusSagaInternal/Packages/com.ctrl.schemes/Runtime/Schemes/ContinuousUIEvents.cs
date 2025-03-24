using System;
using System.Data;
using CTRL.Data;
using ITK;
using UnityEngine;
using UnityEngine.Events;

namespace CTRL.Schemes
{
  [Serializable]
  public struct ContinuousAction
  {
    public enum Action
    {
      Undefined,
      Vertical1D,
      Force1D,
      Joystick
    }
    public Action action;
    public string actionName;
    public float[] axes;
  }

  public class ContinuousUIEvents : DependBehavior
  {
    [SerializeField]
    [Depend(Flags = DependFlags.Scene)]
    private ContinuousUIEventsStream _continuousUIEventsStream;
    [SerializeField]
    private bool showWarnings = false;

    [System.Serializable]
    public class ContinuousActionEvent : UnityEvent<ContinuousAction> { }

    public ContinuousActionEvent OnVertical1DAction = new ContinuousActionEvent();
    public ContinuousActionEvent OnForce1DAction = new ContinuousActionEvent();
    public ContinuousActionEvent OnJoystickAction = new ContinuousActionEvent();
    public ContinuousActionEvent OnContinuousAction = new ContinuousActionEvent();


    private void OnEnable()
    {
      _continuousUIEventsStream.OnStreamSample.AddListener(OnUIEventSample);
    }

    private void OnDisable()
    {
      _continuousUIEventsStream.OnStreamSample.RemoveListener(OnUIEventSample);
    }

    private void OnUIEventSample(Sample<ContinuousUIEventPayload> sample)
    {
      ContinuousAction continuousAction = ContinuousActionForSample(sample.data);
      switch (continuousAction.action)
      {
        case ContinuousAction.Action.Vertical1D:
          OnVertical1DAction.Invoke(continuousAction);
          break;
        case ContinuousAction.Action.Joystick:
          OnJoystickAction.Invoke(continuousAction);
          break;
        case ContinuousAction.Action.Force1D:
          OnForce1DAction.Invoke(continuousAction);
          break;
      }
      OnContinuousAction.Invoke(continuousAction);
    }

    private ContinuousAction ContinuousActionForSample(ContinuousUIEventPayload sampleData)
    {
      return new ContinuousAction
      {
        actionName = sampleData.name,
        action = ContinuousActionActionFromString(sampleData.name),
        axes = sampleData.axes
      };
    }

    private ContinuousAction.Action ContinuousActionActionFromString(string continuousAction)
    {
      switch (continuousAction)
      {
        case "vertical_1d":
          return ContinuousAction.Action.Vertical1D;
        case "joystick":
          return ContinuousAction.Action.Joystick;
        case "force_1d":
          return ContinuousAction.Action.Force1D;
        default:
          LogWarning($"Unknown ContinuousAction action {continuousAction}");
          return ContinuousAction.Action.Undefined;
      }
    }

    private void LogWarning(string warningMessage)
    {
      if (showWarnings)
      {
        Debug.LogWarning(warningMessage);
      }
    }
  }
}
