using UnityEngine;
using UnityEngine.Events;

using CTRL.Data;

namespace CTRL.Schemes
{
  public enum Finger
  {
    Index = 0,
    Middle = 1,
  }

  public interface IFingerData
  {
    /// Current state of finger
    bool IsPinched { get; }

    /// Per-frame event getters
    bool DidPinch { get; }
    bool DidRelease { get; }
    bool DidTap { get; }
    bool DidDoubleTap { get; }
  }

  [System.Serializable]
  public class BoolEmitter : UnityEvent<bool> { }

  public interface IFingerEvents
  {
    UnityEvent OnPinch { get; }
    UnityEvent OnRelease { get; }
    UnityEvent OnTap { get; }
    UnityEvent OnDoubleTap { get; }
    UnityEvent<bool> OnStateChange { get; }
  }

  public interface IPinchProvider
  {
    IFingerData GetFinger(int fingerIdx);
    IFingerData GetFinger(Finger finger);
    IFingerEvents GetFingerEvents(int fingerIdx);
    IFingerEvents GetFingerEvents(Finger finger);
  }

  public class Pinch : ITK.DependBehavior, IPinchProvider
  {
    public bool IsConnected
    {
      get => eventStream != null && eventStream.State == StreamState.Connected;
    }

    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected UIEventsStream eventStream;

    protected MutableFingerData indexData = new MutableFingerData();
    protected MutableFingerData middleData = new MutableFingerData();

    [SerializeField]
    protected FingerEvents indexEvents = new FingerEvents();

    [SerializeField]
    protected FingerEvents middleEvents = new FingerEvents();

    protected Coroutine endOfFrame = null;

    protected virtual void OnEnable()
    {
      eventStream.OnStreamSample.AddListener(OnEventSample);
      endOfFrame = StartCoroutine(PerFrameCoroutine());
    }

    protected virtual void OnDisable()
    {
      eventStream.OnStreamSample.RemoveListener(OnEventSample);

      if (endOfFrame != null)
      {
        StopCoroutine(endOfFrame);
      }
    }

    protected void OnEventSample(Sample<UIEventPayload> sample)
    {
      IFingerEvents events;
      MutableFingerData data;

      //if (sample.data.payload.finger == "index")
      if (sample.data.name == "primary")
      {
        data = indexData;
        events = indexEvents;
      }
      //else if (sample.data.payload.finger == "middle")
      else if (sample.data.name == "secondary")
      {
        data = middleData;
        events = middleEvents;
      }
      else
      {
        return;
      }

      if (sample.data.type == "button" && sample.data.timing == "start")
      {
        data.IsPinched = true;
        data.DidPinch = true;
        events.OnStateChange.Invoke(true);
        events.OnPinch.Invoke();
      }
      else if (sample.data.type == "button" && sample.data.timing == "end")
      {
        data.IsPinched = false;
        data.DidRelease = true;
        events.OnStateChange.Invoke(false);
        events.OnRelease.Invoke();
      }
      else if (sample.data.type == "button_tap")
      {
        data.DidTap = true;
        events.OnTap.Invoke();
      }
      else if (sample.data.type == "button_double_tap")
      {
        data.DidDoubleTap = true;
        events.OnDoubleTap.Invoke();
      }
    }

    public IFingerData GetFinger(int fingerIdx)
    {
      switch (fingerIdx)
      {
        case 0:
          return indexData;
        case 1:
          return middleData;
        default:
          throw new System.Exception($"No such finger id: ${fingerIdx}");
      }
    }

    public IFingerData GetFinger(Finger finger)
    {
      return GetFinger((int)finger);
    }

    public IFingerEvents GetFingerEvents(int fingerIdx)
    {
      switch (fingerIdx)
      {
        case 0:
          return indexEvents;
        case 1:
          return middleEvents;
        default:
          throw new System.Exception($"No such finger id: ${fingerIdx}");
      }
    }

    public IFingerEvents GetFingerEvents(Finger finger)
    {
      return GetFingerEvents((int)finger);
    }

    System.Collections.IEnumerator PerFrameCoroutine()
    {
      while (true)
      {
        yield return new WaitForEndOfFrame();

        indexData.NextFrame();
        middleData.NextFrame();
      }
    }

    protected class MutableFingerData : IFingerData
    {
      public bool IsPinched { get; set; } = false;
      public bool DidPinch { get; set; } = false;
      public bool DidRelease { get; set; } = false;
      public bool DidTap { get; set; } = false;
      public bool DidDoubleTap { get; set; } = false;

      public void NextFrame()
      {
        this.DidPinch = false;
        this.DidRelease = false;
        this.DidTap = false;
        this.DidDoubleTap = false;
      }
    }

    [System.Serializable]
    protected class FingerEvents : IFingerEvents
    {
      [SerializeField]
      protected UnityEvent onPinch = new UnityEvent();
      public UnityEvent OnPinch { get => onPinch; }

      [SerializeField]
      protected UnityEvent onRelease = new UnityEvent();
      public UnityEvent OnRelease { get => onRelease; }

      [SerializeField]
      protected UnityEvent onTap = new UnityEvent();
      public UnityEvent OnTap { get => onTap; }

      [SerializeField]
      protected UnityEvent onDoubleTap = new UnityEvent();
      public UnityEvent OnDoubleTap { get => onDoubleTap; }

      [SerializeField]
      protected BoolEmitter onStateChange = new BoolEmitter();
      public UnityEvent<bool> OnStateChange { get => onStateChange; }
    }
  }
}
