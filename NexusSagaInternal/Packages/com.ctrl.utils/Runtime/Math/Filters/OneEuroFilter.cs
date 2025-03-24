using System;
using UnityEngine;

namespace CTRL.Math.Filters
{

  [Serializable]
  public class OneEuroFilter
  {
    //[Range(0f, 100.0f)]
    [SerializeField]
    public float Mincutoff;

    //[Range(0f, 10.0f)]
    [SerializeField]
    public float Dcutoff;

    //[Range(0f, 1.0f)]
    [SerializeField]
    public float Beta;

    [Serializable]
    public enum FilterMode
    {
      Full,
      Half,
    }

    [SerializeField]
    public FilterMode Mode = FilterMode.Full;

    protected bool _firstTime = true;
    protected ExponentialFilter _xfilter = new ExponentialFilter();
    protected ExponentialFilter _dxfilter = new ExponentialFilter();

    public void Reset()
    {
      _firstTime = true;
      _xfilter.Reset();
      _dxfilter.Reset();
    }

    public float Evaluate(float x, float rate)
    {
      var dx = this._firstTime ? 0 : ((x - _xfilter.Hatxprev) * rate);
      _firstTime = false;

      var edx = EstimateDerivative(dx, rate);
      var cutoff = Mincutoff + Beta * edx;
      return _xfilter.Evaluate(x, Alpha(rate, cutoff));
    }

    protected float Alpha(float rate, float cutoff)
    {
      if (rate == 0 || cutoff == 0)
      {
        return 0f;
      }
      var tau = 1 / (2 * Mathf.PI * cutoff);
      var te = 1 / rate;
      var a = te / (te + tau);
      return a;
    }

    protected float EstimateDerivative(float dx, float rate)
    {
      var edx = _dxfilter.Evaluate(dx, Alpha(rate, Dcutoff));

      switch (Mode)
      {
        case FilterMode.Full:
          return Mathf.Abs(edx);

        case FilterMode.Half:
          return Mathf.Max(0, edx); // RELU

        default:
          Debug.LogErrorFormat("Unhandled filter mode: {0}", Mode);
          Debug.Assert(false);
          return 0;
      }
    }
  }

  [Serializable]
  public class FixedOneEuroFilter : OneEuroFilter, IFilter<float>
  {
    [SerializeField]
    public float Rate = 1f / 60f;

    public float Evaluate(float x)
    {
      return base.Evaluate(x, Rate);
    }
  }
}
