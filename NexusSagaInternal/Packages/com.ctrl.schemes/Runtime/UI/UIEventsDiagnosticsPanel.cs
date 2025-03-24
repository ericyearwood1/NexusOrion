using UnityEngine;
using UnityEngine.UI;
using CTRL;
using CTRL.Schemes;

namespace CTRL.UI
{
  public class UIEventsDiagnosticsPanel : ITK.DependBehavior
  {
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected UIEvents uiEvents;
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected ContinuousUIEvents continuousUIEvents;
    [SerializeField]
    protected Text loggerText;
    [SerializeField]
    protected SpriteRenderer primaryButton;
    [SerializeField]
    protected SpriteRenderer secondaryButton;

    void OnEnable()
    {
      uiEvents.OnButtonState.AddListener(OnButtonState);
      uiEvents.OnButtonAction.AddListener(OnButtonAction);
      uiEvents.OnDPadAction.AddListener(OnDPadAction);
      uiEvents.OnShortcutAction.AddListener(OnShortcutAction);
      continuousUIEvents.OnContinuousAction.AddListener(OnContinuousAction);
    }

    void OnDisable()
    {
      uiEvents.OnButtonState.RemoveListener(OnButtonState);
      uiEvents.OnButtonAction.RemoveListener(OnButtonAction);
      uiEvents.OnDPadAction.RemoveListener(OnDPadAction);
      uiEvents.OnShortcutAction.RemoveListener(OnShortcutAction);
      continuousUIEvents.OnContinuousAction.RemoveListener(OnContinuousAction);
    }

    void OnButtonState(ButtonState buttonState)
    {
      loggerText.text = $"ButtonState:\n{buttonState.button}, {buttonState.timing}";

      if (buttonState.button == CTRL.Schemes.Button.Primary)
      {
        if (buttonState.timing == Timing.Start)
          primaryButton.maskInteraction = SpriteMaskInteraction.None;
        else
          primaryButton.maskInteraction = SpriteMaskInteraction.VisibleOutsideMask;
      }
      else if (buttonState.button == CTRL.Schemes.Button.Secondary)
      {
        if (buttonState.timing == Timing.Start)
          secondaryButton.maskInteraction = SpriteMaskInteraction.None;
        else
          secondaryButton.maskInteraction = SpriteMaskInteraction.VisibleOutsideMask;
      }
    }

    void OnButtonAction(ButtonAction buttonAction)
    {
      loggerText.text = $"ButtonAction:\n{buttonAction.button}, {buttonAction.action}, {buttonAction.timing}";
    }

    void OnDPadAction(DPadAction dPadAction)
    {
      loggerText.text = $"DPadAction:\n{dPadAction.action}";
    }

    void OnShortcutAction(ShortcutAction shortcutAction)
    {
      loggerText.text = $"ShortcutAction:\n{shortcutAction.action}";
    }

    void OnContinuousAction(ContinuousAction continuousAction)
    {
      loggerText.text = $"ContinuousAction: {continuousAction.action}, {string.Join(", ", continuousAction.axes)}";
    }
  }
}
