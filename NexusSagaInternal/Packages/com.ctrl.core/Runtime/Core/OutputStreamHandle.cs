using UnityEngine;
using UnityEngine.Events;

using System.Collections.Generic;
using Newtonsoft.Json.Linq;

namespace CTRL
{
  [System.Serializable]
  public class StreamBatchEmitter<T> : UnityEvent<StreamBatch<T>> { }

  [System.Serializable]
  public class SampleEmitter<T> : UnityEvent<Sample<T>> { }

  /**
    This component is the main point of usage of the SDK.
    This is an abstract class, so users can't actually instantiate it.
    Instead, users should add the derived components from this class,
    which represent actual streams.
   */
  public abstract class OutputStreamHandle<T> : StreamHandle<T>
  {
    /// Invoked with each batch
    [SerializeField]
    protected StreamBatchEmitter<T> onStreamBatch = new StreamBatchEmitter<T>();
    public StreamBatchEmitter<T> OnStreamBatch { get => onStreamBatch; }

    /// Invoked with every sample from each batch
    [SerializeField]
    protected SampleEmitter<T> onStreamSample = new SampleEmitter<T>();
    public SampleEmitter<T> OnStreamSample { get => onStreamSample; }

    /// Invoked with just the last sample from each batch
    [SerializeField]
    protected SampleEmitter<T> onStreamLatestSample = new SampleEmitter<T>();
    public SampleEmitter<T> OnStreamLatestSample { get => onStreamLatestSample; }

    /// List of samples arriving between Update()'s
    protected List<Sample<T>> sampleBuffer = new List<Sample<T>>();
    protected Coroutine endOfFrame = null;

    /// Samples since the last call to Update
    public IEnumerable<Sample<T>> FrameSamples { get => sampleBuffer; }

    /// Can query the latest sample
    public Sample<T>? Latest { get; protected set; }

    // TODO: expose the JSON message for debugging
    public void ProcessStreamBatch(StreamBatch<T> streamBatch)
    {
      // Invoke the event with the entire batch
      OnStreamBatch.Invoke(streamBatch);

      // Store the samples in the back buffer
      sampleBuffer.AddRange(streamBatch.samples);

      // Invoke the event with each sample in order
      foreach (var sample in streamBatch.samples)
      {
        OnStreamSample.Invoke(sample);
      }

      // Invoke the latest sample event if there's at least one sample
      if (streamBatch.samples.Count > 0)
      {
        var latest = streamBatch.samples[streamBatch.samples.Count - 1];
        OnStreamLatestSample.Invoke(latest);
        Latest = latest;
      }
    }

    protected new void OnEnable()
    {
      base.OnEnable();

      endOfFrame = StartCoroutine(PerFrameCoroutine());
    }

    protected new void OnDisable()
    {
      base.OnDisable();

      if (endOfFrame != null)
      {
        StopCoroutine(endOfFrame);
      }
    }

    System.Collections.IEnumerator PerFrameCoroutine()
    {
      while (true)
      {
        yield return new WaitForEndOfFrame();
        sampleBuffer.Clear();
      }
    }
  }
}
