using UnityEngine;

using System;
using CTRL.Schemes;

public class PinchLogger : ITK.DependBehavior
{
  [SerializeField]
  [ITK.Depend(Flags = ITK.DependFlags.Scene)]
  protected Pinch pinch;

  [SerializeField]
  protected bool printIsPinched = false;

  protected virtual void Update()
  {

    foreach (Finger finger in Enum.GetValues(typeof(Finger)))
    {
      var fingerData = pinch.GetFinger(finger);

      if (printIsPinched)
      {
        Debug.Log($"{finger} {fingerData.IsPinched}");
      }

      if (fingerData.DidPinch)
      {
        Debug.Log($"{finger} DidPinch");
      }
      if (fingerData.DidRelease)
      {
        Debug.Log($"{finger} DidRelease");
      }
      if (fingerData.DidTap)
      {
        Debug.Log($"{finger} DidTap");
      }
      if (fingerData.DidDoubleTap)
      {
        Debug.Log($"{finger} DidDoubleTap");
      }
    }
  }
}
