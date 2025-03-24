using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CTRL.Schemes
{
  public class InputStateLogger : ITK.DependBehavior
  {
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected InputState inputState;

    public class ButtonLogger
    {
      public string buttonName;
      public InputState.ButtonData buttonData;

      public ButtonLogger(string buttonName, InputState.ButtonData buttonData)
      {
        this.buttonName = buttonName;
        this.buttonData = buttonData;
      }

      public void LogEvents(InputState inputState)
      {
        if (buttonData.DidDown)
        {
          Debug.Log($"{buttonName} Down");
        }
        if (buttonData.DidUp)
        {
          Debug.Log($"{buttonName} Up");
        }
        if (buttonData.DidTap)
        {
          Debug.Log($"{buttonName} Tapped");
        }
        if (buttonData.DidDoubleTap)
        {
          Debug.Log($"{buttonName} Double Tapped");
        }
        if (buttonData.DidStartHold)
        {
          Debug.Log($"{buttonName} Hold Start");
        }
        if (buttonData.DidEndHold)
        {
          Debug.Log($"{buttonName} Hold End");
        }
      }
    }

    ButtonLogger primaryLogger;
    ButtonLogger secondaryLogger;
    ButtonLogger tertiaryLogger;

    protected override void Start()
    {
      base.Start();
      primaryLogger = new ButtonLogger("Primary", inputState.Primary);
      secondaryLogger = new ButtonLogger("Secondary", inputState.Secondary);
      tertiaryLogger = new ButtonLogger("Tertiary", inputState.Tertiary);
    }

    void Update()
    {
      primaryLogger.LogEvents(inputState);
      secondaryLogger.LogEvents(inputState);
      tertiaryLogger.LogEvents(inputState);
      if (inputState.DPadUp.DidFire)
      {
        Debug.Log("DPad Up");
      }
      if (inputState.DPadDown.DidFire)
      {
        Debug.Log("DPad Down");
      }
      if (inputState.DPadLeft.DidFire)
      {
        Debug.Log("DPad Left");
      }
      if (inputState.DPadRight.DidFire)
      {
        Debug.Log("DPad Right");
      }
      if (inputState.Vertical1DUp.DidFire)
      {
        Debug.Log("Vertical1D Up");
      }
      if (inputState.Vertical1DDown.DidFire)
      {
        Debug.Log("Vertical1D Down");
      }
      if (inputState.Snap.DidFire)
      {
        Debug.Log("Snap");
      }
      if (inputState.IndexFlick.DidFire)
      {
        Debug.Log("Index Flick");
      }
      if (inputState.Squeeze.DidFire)
      {
        Debug.Log("Squeeze");
      }
      if (inputState.Knock.DidFire)
      {
        Debug.Log("Knock");
      }
      if (inputState.Shortcut05.DidFire)
      {
        Debug.Log("Shortcut05");
      }
      if (inputState.Shortcut06.DidFire)
      {
        Debug.Log("Shortcut06");
      }
      if (inputState.Shortcut07.DidFire)
      {
        Debug.Log("Shortcut07");
      }
      if (inputState.Shortcut08.DidFire)
      {
        Debug.Log("Shortcut08");
      }
      if (inputState.Shortcut09.DidFire)
      {
        Debug.Log("Shortcut09");
      }
      if (inputState.Shortcut10.DidFire)
      {
        Debug.Log("Shortcut10");
      }
    }
  }
}
