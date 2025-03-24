using System;
using System.Data;
using CTRL.Data;
using ITK;
using UnityEngine;
using UnityEngine.Events;

namespace CTRL.Schemes
{
  [Serializable]
  public struct EMGAction
  {
    public enum Action
    {
      Undefined,
      IndexPinch,
      MiddlePinch,
      ThumbUp,
      ThumbDown,
      ThumbLeft,
      ThumbRight,
      ThumbTap,
      Flick,
      Snap,
      Fist,
    }
    public Action action;
    public string actionName;
    public Timing timing;
  }

  public class EMGEvents : DependBehavior
  {
    [SerializeField]
    [Depend(Flags = DependFlags.Scene)]
    private EMGEventsStream _emgEventStream;
    [SerializeField]
    private bool showWarnings = false;

    [System.Serializable]
    public class EMGActionEvent : UnityEvent<EMGAction> { }
    [System.Serializable]
    public class EMGEventEvent : UnityEvent<EMGEventPayload> { }

    public EMGActionEvent onEMGActionEvent = new EMGActionEvent();
    public EMGEventEvent OnEMGEvent = new EMGEventEvent();

    private void OnEnable()
    {
      _emgEventStream.OnStreamSample.AddListener(OnStreamSample);
    }

    private void OnDisable()
    {
      _emgEventStream.OnStreamSample.RemoveListener(OnStreamSample);
    }

    private void OnStreamSample(Sample<EMGEventPayload> sample)
    {
      switch (sample.data.name)
      {
        case "index_pinch":
        case "middle_pinch":
        case "thumb_up":
        case "thumb_down":
        case "thumb_left":
        case "thumb_right":
        case "thumb_tap":
        case "index_flick":
        case "snap":
        case "fist":
          onEMGActionEvent.Invoke(EMGActionForSample(sample.data));
          break;
        default:
          LogWarning($"Unknown sample name {sample.data.name}");
          break;
      }

      OnEMGEvent.Invoke(sample.data);
    }

    private EMGAction EMGActionForSample(EMGEventPayload sampleData)
    {
      return new EMGAction
      {
        action = EMGActionActionFromString(sampleData.name),
        actionName = sampleData.name,
        timing = TimingFromString(sampleData.timing)
      };
    }

    private EMGAction.Action EMGActionActionFromString(string action)
    {
      switch (action)
      {
        case "index_pinch":
          return EMGAction.Action.IndexPinch;
        case "middle_pinch":
          return EMGAction.Action.MiddlePinch;
        case "thumb_up":
          return EMGAction.Action.ThumbUp;
        case "thumb_down":
          return EMGAction.Action.ThumbDown;
        case "thumb_left":
          return EMGAction.Action.ThumbLeft;
        case "thumb_right":
          return EMGAction.Action.ThumbRight;
        case "thumb_tap":
          return EMGAction.Action.ThumbTap;
        case "index_flick":
          return EMGAction.Action.Flick;
        case "snap":
          return EMGAction.Action.Snap;
        case "fist":
          return EMGAction.Action.Fist;
        default:
          LogWarning($"Unknown EMGAction action {action}");
          return EMGAction.Action.Undefined;
      }
    }

    private Timing TimingFromString(string timing)
    {
      switch (timing)
      {
        case "start":
          return Timing.Start;
        case "end":
          return Timing.End;
        case "instant":
          return Timing.Instant;
        default:
          throw new EvaluateException($"Unknown UIEvent timing {timing}");
      };
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
