using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace CTRL.Math.Filters
{

  [Serializable]
  public class MovingAverageFilter : IFilter<float>
  {
    [SerializeField]
    public int WindowSize = 10;

    protected List<float> samples = new List<float>();

    public void Reset()
    {
      samples.Clear();
    }

    protected float SumFn(float sum, float val)
    {
      return sum + (val / samples.Count);
    }

    public float Evaluate(float sample)
    {
      samples.Add(sample);

      // Ensure `samples` contains only the `windowSize` most recent samples
      samples.RemoveRange(0, Mathf.Max(0, this.samples.Count - this.WindowSize));

      // Compute sample mean
      var mean = samples.Aggregate(0f, SumFn);

      return mean;
    }
  }

}
