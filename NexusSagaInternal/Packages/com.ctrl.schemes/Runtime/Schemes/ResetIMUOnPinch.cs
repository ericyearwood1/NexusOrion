using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace CTRL.Schemes
{
  public class ResetIMUOnPinch : ITK.DependBehavior
  {
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected Pinch pinch;

    protected List<RelativeIMU> imus;

    [SerializeField]
    protected Finger finger = Finger.Index;
    public Finger Finger { get => finger; }

    protected virtual void OnEnable()
    {
      imus = FindObjectsOfType<RelativeIMU>().ToList();
      pinch.GetFingerEvents(finger).OnPinch.AddListener(OnPinch);
    }

    protected virtual void OnDisable()
    {
      imus = new List<RelativeIMU>();
      pinch.GetFingerEvents(finger).OnPinch.RemoveListener(OnPinch);
    }

    public void OnPinch()
    {
      foreach(var imu in imus)
      {
        imu.ResetReference();
      }
    }
  }
}
