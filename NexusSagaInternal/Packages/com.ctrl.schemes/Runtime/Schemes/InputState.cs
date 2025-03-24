using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using CTRL.Data;

namespace CTRL.Schemes
{
  public class InputState : ITK.DependBehavior
  {
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected UIEvents uiEvents;
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected ContinuousUIEvents continuousUIEvents;

    public abstract class DiscreteData
    {
      public abstract void Reset();
    }

    public class ButtonData : DiscreteData
    {
      public bool IsDown { get; set; }
      public bool DidDown { get; set; }
      public bool DidUp { get; set; }
      public bool DidTap { get; set; }
      public bool DidDoubleTap { get; set; }
      public bool DidStartHold { get; set; }
      public bool DidEndHold { get; set; }
      public override void Reset()
      {
        DidDown = false;
        DidUp = false;
        DidTap = false;
        DidDoubleTap = false;
        DidStartHold = false;
        DidEndHold = false;
      }
    }

    public class InstantaneousData : DiscreteData
    {
      public bool DidFire { get; set; }
      public override void Reset()
      {
        DidFire = false;
      }
    }

    public class ContinuousData
    {
      public float[] axes;
    }

    public readonly ButtonData Primary = new ButtonData();
    public readonly ButtonData Secondary = new ButtonData();
    public readonly ButtonData Tertiary = new ButtonData();
    public readonly InstantaneousData DPadUp = new InstantaneousData();
    public readonly InstantaneousData DPadDown = new InstantaneousData();
    public readonly InstantaneousData DPadLeft = new InstantaneousData();
    public readonly InstantaneousData DPadRight = new InstantaneousData();
    public readonly InstantaneousData Vertical1DUp = new InstantaneousData();
    public readonly InstantaneousData Vertical1DDown = new InstantaneousData();
    public readonly InstantaneousData Snap = new InstantaneousData();
    public readonly InstantaneousData IndexFlick = new InstantaneousData();
    public readonly InstantaneousData Squeeze = new InstantaneousData();
    public readonly InstantaneousData Knock = new InstantaneousData();
    public readonly InstantaneousData Shortcut05 = new InstantaneousData();
    public readonly InstantaneousData Shortcut06 = new InstantaneousData();
    public readonly InstantaneousData Shortcut07 = new InstantaneousData();
    public readonly InstantaneousData Shortcut08 = new InstantaneousData();
    public readonly InstantaneousData Shortcut09 = new InstantaneousData();
    public readonly InstantaneousData Shortcut10 = new InstantaneousData();
    public readonly ContinuousData Vertical1D = new ContinuousData();
    public readonly ContinuousData Joystick = new ContinuousData();

    // Management
    protected Coroutine endOfFrame;
    protected List<DiscreteData> discreteInputs;

    // Create list of discrete inputs
    protected void Awake()
    {
      base.Awake();

      discreteInputs = new List<DiscreteData>
      {
        Primary,
        Secondary,
        Tertiary,
        DPadUp,
        DPadDown,
        DPadLeft,
        DPadRight,
        Vertical1DUp,
        Vertical1DDown,
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
      };
    }

    // Register listeners for UIEvents
    protected void OnEnable()
    {
      uiEvents.OnButtonState.AddListener(OnButtonState);
      uiEvents.OnButtonAction.AddListener(OnButtonAction);
      uiEvents.OnDPadAction.AddListener(OnDPadAction);
      uiEvents.OnVertical1DAction.AddListener(OnVertical1DAction);
      uiEvents.OnShortcutAction.AddListener(OnShortcutAction);
      continuousUIEvents.OnContinuousAction.AddListener(OnContinuousAction);
      endOfFrame = StartCoroutine(EndOfFrameRoutine());
    }

    protected void OnDisable()
    {
      uiEvents.OnButtonState.RemoveListener(OnButtonState);
      uiEvents.OnButtonAction.RemoveListener(OnButtonAction);
      uiEvents.OnDPadAction.RemoveListener(OnDPadAction);
      uiEvents.OnVertical1DAction.RemoveListener(OnVertical1DAction);
      uiEvents.OnShortcutAction.RemoveListener(OnShortcutAction);
      continuousUIEvents.OnContinuousAction.RemoveListener(OnContinuousAction);
      if (endOfFrame != null)
      {
        StopCoroutine(endOfFrame);
      }
    }

    // End of frame routine
    protected IEnumerator EndOfFrameRoutine()
    {
      while (true)
      {
        yield return new WaitForEndOfFrame();

        foreach (var discreteInput in discreteInputs)
        {
          discreteInput.Reset();
        }
      }
    }

    // Process events from UIEvents
    protected void OnButtonState(ButtonState buttonState)
    {
      var buttonData = GetButtonDataFromButton(buttonState.button);
      if (buttonState.timing == Timing.Start)
      {
        buttonData.DidDown = true;
        buttonData.IsDown = true;
      }
      else if (buttonState.timing == Timing.End)
      {
        buttonData.DidUp = true;
        buttonData.IsDown = false;
      }
    }

    protected void OnButtonAction(ButtonAction buttonAction)
    {
      var buttonData = GetButtonDataFromButton(buttonAction.button);
      if (buttonAction.action == ButtonAction.Action.Tap)
      {
        buttonData.DidTap = true;
      }
      else if (buttonAction.action == ButtonAction.Action.DoubleTap)
      {
        buttonData.DidDoubleTap = true;
      }
      else if (buttonAction.action == ButtonAction.Action.Hold)
      {
        if (buttonAction.timing == Timing.Start)
        {
          buttonData.DidStartHold = true;
        }
        else if (buttonAction.timing == Timing.End)
        {
          buttonData.DidEndHold = true;
        }
      }
    }

    protected void OnDPadAction(DPadAction dPadAction)
    {
      switch (dPadAction.action)
      {
        case DPadAction.Action.Up:
          DPadUp.DidFire = true;
          break;
        case DPadAction.Action.Down:
          DPadDown.DidFire = true;
          break;
        case DPadAction.Action.Left:
          DPadLeft.DidFire = true;
          break;
        case DPadAction.Action.Right:
          DPadRight.DidFire = true;
          break;
      }
    }

    protected void OnVertical1DAction(Vertical1DAction vertical1DAction)
    {
      switch (vertical1DAction.action)
      {
        case Vertical1DAction.Action.Up:
          DPadUp.DidFire = true;
          break;
        case Vertical1DAction.Action.Down:
          DPadDown.DidFire = true;
          break;
      }
    }

    protected void OnShortcutAction(ShortcutAction shortcutAction)
    {
      switch (shortcutAction.action)
      {
        case ShortcutAction.Action.Snap:
          Snap.DidFire = true;
          break;
        case ShortcutAction.Action.IndexFlick:
          IndexFlick.DidFire = true;
          break;
        case ShortcutAction.Action.Squeeze:
          Squeeze.DidFire = true;
          break;
        case ShortcutAction.Action.Knock:
          Knock.DidFire = true;
          break;
        case ShortcutAction.Action.Shortcut05:
          Shortcut05.DidFire = true;
          break;
        case ShortcutAction.Action.Shortcut06:
          Shortcut06.DidFire = true;
          break;
        case ShortcutAction.Action.Shortcut07:
          Shortcut07.DidFire = true;
          break;
        case ShortcutAction.Action.Shortcut08:
          Shortcut08.DidFire = true;
          break;
        case ShortcutAction.Action.Shortcut09:
          Shortcut09.DidFire = true;
          break;
        case ShortcutAction.Action.Shortcut10:
          Shortcut10.DidFire = true;
          break;
      }
    }

    protected void OnContinuousAction(ContinuousAction continuousAction)
    {
      switch (continuousAction.action)
      {
        case ContinuousAction.Action.Vertical1D:
          Vertical1D.axes = continuousAction.axes;
          return;
        case ContinuousAction.Action.Joystick:
          Joystick.axes = continuousAction.axes;
          return;
      }
    }

    // Helper functions
    private ButtonData GetButtonDataFromButton(Button button)
    {
      switch (button)
      {
        case Button.Primary:
          return Primary;
        case Button.Secondary:
          return Secondary;
        case Button.Tertiary:
          return Tertiary;
      }

      return null;
    }
  }
}
