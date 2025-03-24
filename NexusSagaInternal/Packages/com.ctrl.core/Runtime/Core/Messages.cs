using System.Collections.Generic;

namespace CTRL
{
  /// <summary>
  /// A batch of samples emitted by CTRL-R.
  /// The SDK automatically merges the batches per stream between frames.
  /// This means that when you call OutputStreamHandle.GetData() within Update(), you will get
  /// a batch of all the samples received between the previous frame and the current frame.
  /// </summary>
#pragma warning disable IDE1006
  [System.Serializable]
  public class StreamBatch<T>
  {
    public List<Sample<T>> samples;
    public long batch_num;

    public Sample<T> Latest()
    {
      return samples[samples.Count - 1];
    }
  }
#pragma warning restore IDE1006

  /// <summary>
  /// A single sample of data, and the estimated timestamp the event that
  /// caused this sample occurred at. For instance, if someone pinches at time=0, and the
  /// application receives this message at time=1, then Sample.timestamp_s=0 (+- some margin of error).
  /// </summary>
#pragma warning disable IDE1006
  [System.Serializable]
  public struct Sample<T>
  {
    public T data;
    public double timestamp_s;
  }
#pragma warning restore IDE1006
}
