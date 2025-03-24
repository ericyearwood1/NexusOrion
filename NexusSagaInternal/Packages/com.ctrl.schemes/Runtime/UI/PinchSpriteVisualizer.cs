using UnityEngine;
using CTRL.Schemes;

namespace CTRL.UI
{
  [RequireComponent(typeof(Pinch))]
  public class PinchSpriteVisualizer : ITK.DependBehavior
  {
    public Finger finger;

    public SpriteRenderer sprite;

    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected Pinch pinch;
    protected void Update()
    {
      if (sprite != null)
      {

        if (pinch.GetFinger(finger).IsPinched)
        {
          sprite.maskInteraction = SpriteMaskInteraction.None;
        }
        else
        {
          sprite.maskInteraction = SpriteMaskInteraction.VisibleOutsideMask;
        }
      }

    }
  }
}
