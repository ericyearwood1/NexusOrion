using System;
using System.Data;
using CTRL.Data;
using ITK;
using UnityEngine;
using UnityEngine.Events;

namespace CTRL.Schemes
{
  public enum Button
  {
    Undefined,
    Primary,
    Secondary,
    Tertiary
  }

  public enum Timing
  {
    Start,
    End,
    Instant,
    Click
  }

  [Serializable]
  public struct ButtonState
  {
    public Button button;
    public string buttonName;
    public Timing timing;
  }

  [Serializable]
  public struct ButtonAction
  {
    public enum Action
    {
      Undefined,
      Tap,
      Hold,
      DoubleTap
    }

    public Button button;
    public string buttonName;
    public Action action;
    public string actionName;
    public Timing timing;
  }

  [Serializable]
  public struct DPadAction
  {
    public enum Action
    {
      Undefined,
      Up,
      Down,
      Left,
      Right
    }

    public Action action;
    public string actionName;
  }

  [Serializable]
  public struct Vertical1DAction
  {
    public enum Action
    {
      Undefined,
      Up,
      Down,
    }

    public Action action;
    public string actionName;
  }

  [Serializable]
  public struct ShortcutAction
  {
    public enum Action
    {
      Undefined,
      Snap,
      IndexFlick,
      Squeeze,
      Knock,
      Shortcut05,
      Shortcut06,
      Shortcut07,
      Shortcut08,
      Shortcut09,
      Shortcut10
    }
    public Action action;
    public string actionName;
  }

  public class UIEvents : DependBehavior
  {
    [SerializeField]
    [Depend(Flags = DependFlags.Scene)]
    private UIEventsStream _uiEventsStream;

    [SerializeField]
    [Depend(Flags = DependFlags.Scene | DependFlags.Optional)]
    private CeresEventStream _ceresEventStream;

    [SerializeField]
    private bool showWarnings = false;

    [System.Serializable]
    public class ButtonActionEvent : UnityEvent<ButtonAction> { }
    [System.Serializable]
    public class ButtonStateEvent : UnityEvent<ButtonState> { }
    [System.Serializable]
    public class DPadActionEvent : UnityEvent<DPadAction> { }
    [System.Serializable]
    public class Vertical1DActionEvent : UnityEvent<Vertical1DAction> { }
    [System.Serializable]
    public class ShortcutActionEvent : UnityEvent<ShortcutAction> { }
    [System.Serializable]
    public class UIEventEvent : UnityEvent<UIEventPayload> { }

    public class CeresEventEvent : UnityEvent<CeresEventPayload> { }


    public ButtonActionEvent OnButtonAction = new ButtonActionEvent();
    public ButtonStateEvent OnButtonState = new ButtonStateEvent();
    public DPadActionEvent OnDPadAction = new DPadActionEvent();
    public Vertical1DActionEvent OnVertical1DAction = new Vertical1DActionEvent();
    public ShortcutActionEvent OnShortcutAction = new ShortcutActionEvent();
    public UIEventEvent OnUIEvent = new UIEventEvent();
    public CeresEventEvent OnCeresEvent = new CeresEventEvent();


    private void OnEnable()
    {
      _uiEventsStream.OnStreamSample.AddListener(OnUIEventSample);
      if(_ceresEventStream) _ceresEventStream.OnStreamSample.AddListener(OnCeresEventSample);
    }

    private void OnDisable()
    {
      _uiEventsStream.OnStreamSample.RemoveListener(OnUIEventSample);
      if(_ceresEventStream) _ceresEventStream.OnStreamSample.RemoveListener(OnCeresEventSample);
    }

    private void OnUIEventSample(Sample<UIEventPayload> sample)
    {
      switch (sample.data.type)
      {
        case "button":
          OnButtonState.Invoke(ButtonStateForSample(sample.data));
          break;
        case "dpad":
          OnDPadAction.Invoke(DPadActionForSample(sample.data));
          break;
        case "vertical_1d":
          OnVertical1DAction.Invoke(Vertical1DActionForSample(sample.data));
          break;
        case "shortcut":
          OnShortcutAction.Invoke(ShortcutActionForSample(sample.data));
          break;
        case "button_tap":
        case "button_tap_double":
        case "button_hold":
          OnButtonAction.Invoke(ButtonActionForSample(sample.data));
          break;
        default:
          LogWarning($"Unknown sample name {sample.data.type}");
          break;
      }

      OnUIEvent.Invoke(sample.data);
    }

    private void OnCeresEventSample(Sample<CeresEventPayload> sample)
    {
      switch (sample.data.finger)
      {
        case "index":
          OnButtonState.Invoke(CeresButtonStateForSample(sample.data.finger, sample.data.action));
          break;
        case "middle":
          OnButtonState.Invoke(CeresButtonStateForSample(sample.data.finger, sample.data.action));
          break;
        case "thumb":
          if (sample.data.action == "click")
          {
            OnButtonState.Invoke(CeresButtonStateForSample("tertiary", "instant"));
          }
          else
          {
            OnDPadAction.Invoke(CeresDPadActionForSample(sample.data));
          }
          break;
        default:
          LogWarning($"Unknown sample name {sample.data.finger}");
          break;
      }

      OnCeresEvent.Invoke(sample.data);
    }

    private ButtonAction ButtonActionForSample(UIEventPayload sampleData)
    {
      return new ButtonAction
      {
        actionName = sampleData.type,
        buttonName = sampleData.name,
        action = ButtonActionActionFromString(sampleData.type),
        button = ButtonFromString(sampleData.name),
        timing = TimingFromString(sampleData.timing)
      };
    }

    private ButtonState ButtonStateForSample(UIEventPayload sampleData)
    {
      return new ButtonState
      {
        buttonName = sampleData.name,
        button = ButtonFromString(sampleData.name),
        timing = TimingFromString(sampleData.timing)
      };
    }

    private ButtonState CeresButtonStateForSample(string finger, string action)
    {
      return new ButtonState
      {
        buttonName = finger,
        button = ButtonFromString(finger),
        timing = TimingFromString(action)
      };
    }

    private DPadAction DPadActionForSample(UIEventPayload sampleData)
    {
      return new DPadAction
      {
        actionName = sampleData.name,
        action = DPadActionFromString(sampleData.name)
      };
    }

    private DPadAction CeresDPadActionForSample(CeresEventPayload sampleData)
    {
      return new DPadAction
      {
        actionName = sampleData.action,
        action = DPadActionFromString(sampleData.action)
      };
    }

    private Vertical1DAction Vertical1DActionForSample(UIEventPayload sampleData)
    {
      return new Vertical1DAction
      {
        actionName = sampleData.name,
        action = Vertical1DActionFromString(sampleData.name)
      };
    }

    private ShortcutAction ShortcutActionForSample(UIEventPayload sampleData)
    {
      return new ShortcutAction
      {
        actionName = sampleData.name,
        action = ShortcutActionActionFromString(sampleData.name)
      };
    }

    private DPadAction.Action DPadActionFromString(string action)
    {
      switch (action)
      {
        case "up":
          return DPadAction.Action.Up;
        case "down":
          return DPadAction.Action.Down;
        case "left":
          return DPadAction.Action.Left;
        case "right":
          return DPadAction.Action.Right;
        default:
          LogWarning($"Unknown DPadAction Action name {action}");
          return DPadAction.Action.Undefined;
      }
    }

    private Vertical1DAction.Action Vertical1DActionFromString(string action)
    {
      switch (action)
      {
        case "up":
          return Vertical1DAction.Action.Up;
        case "down":
          return Vertical1DAction.Action.Down;
        default:
          LogWarning($"Unknown Vertical1D Action name {action}");
          return Vertical1DAction.Action.Undefined;
      }
    }


    private Button ButtonFromString(string button)
    {
      switch (button)
      {
        case "primary":
          return Button.Primary;
        case "secondary":
          return Button.Secondary;
        case "tertiary":
          return Button.Tertiary;
        case "index":
          return Button.Primary;
        case "middle":
          return Button.Secondary;
        default:
          LogWarning($"Unknown Button {button}");
          return Button.Undefined;
      }
    }

    private ButtonAction.Action ButtonActionActionFromString(string action)
    {
      switch (action)
      {
        case "button_tap":
          return ButtonAction.Action.Tap;
        case "button_hold":
          return ButtonAction.Action.Hold;
        case "button_tap_double":
          return ButtonAction.Action.DoubleTap;
        default:
          LogWarning($"Unknown ButtonAction action {action}");
          return ButtonAction.Action.Undefined;
      }
    }

    private ShortcutAction.Action ShortcutActionActionFromString(string shortcut)
    {
      switch (shortcut)
      {
        case "snap":
          return ShortcutAction.Action.Snap;
        case "index_flick":
          return ShortcutAction.Action.IndexFlick;
        case "squeeze":
          return ShortcutAction.Action.Squeeze;
        case "knock":
          return ShortcutAction.Action.Knock;
        case "shortcut05":
          return ShortcutAction.Action.Shortcut05;
        case "shortcut06":
          return ShortcutAction.Action.Shortcut06;
        case "shortcut07":
          return ShortcutAction.Action.Shortcut07;
        case "shortcut08":
          return ShortcutAction.Action.Shortcut08;
        case "shortcut09":
          return ShortcutAction.Action.Shortcut09;
        case "shortcut10":
          return ShortcutAction.Action.Shortcut10;
        default:
          LogWarning($"Unknown ShortcutAction action {shortcut}");
          return ShortcutAction.Action.Undefined;
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
        case "press":
          return Timing.Start;
        case "release":
          return Timing.End;
        case "click":
          return Timing.Click;
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
