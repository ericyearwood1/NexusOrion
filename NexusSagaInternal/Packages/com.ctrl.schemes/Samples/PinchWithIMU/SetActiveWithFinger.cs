using UnityEngine;

namespace CTRL.Schemes.Samples
{
  public class SetActiveWithFinger : ITK.DependBehavior
  {
    public Finger Finger;
    public GameObject Target;

    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected Pinch pinch;

    protected void OnEnable()
    {
      pinch.GetFingerEvents(Finger).OnStateChange.AddListener(Target.SetActive);
    }

    protected void OnDisable()
    {
      pinch.GetFingerEvents(Finger).OnStateChange.RemoveListener(Target.SetActive);
    }
  }
}
