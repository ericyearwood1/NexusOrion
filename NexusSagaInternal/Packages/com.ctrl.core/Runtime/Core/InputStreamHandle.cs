using UnityEngine;
using System.Collections.Generic;

namespace CTRL
{
  public abstract class InputStreamHandle<T> : StreamHandle<T>
  {
    public static double Timestamp()
    {
      var ms = new System.DateTimeOffset(System.DateTime.Now).ToUnixTimeMilliseconds();
      return (double)ms / 1000.0;
    }

    public void SendSample(T sampleData)
    {
      SendSample(sampleData, Timestamp());
    }

    public void SendSample(T sampleData, double timestampSeconds)
    {
      var sample = new Sample<T>
      {
        data = sampleData,
        timestamp_s = timestampSeconds,
      };

      this.SendBatch(new List<Sample<T>> { sample });
    }

    public void SendBatch(List<Sample<T>> samples)
    {
      Client.SendStreamBatch(StreamName, StreamId, samples);
    }
  }
}
