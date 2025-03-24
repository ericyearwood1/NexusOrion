using System;
using UnityEngine;

namespace CTRL.Math.Filters
{
  [Serializable]
  public class ExponentialFilter
  {
    protected bool _firstTime = true;

    protected float _hatxprev = 0;
    public float Hatxprev { get => _hatxprev; }

    public void Reset()
    {
      this._firstTime = true;
      this._hatxprev = 0;
    }

    public float Evaluate(float x, float alpha)
    {
      if (this._firstTime)
      {
        this._firstTime = false;
        this._hatxprev = x;
      }
      var hatx = alpha * x + (1 - alpha) * this._hatxprev;
      this._hatxprev = hatx;
      return hatx;
    }
  }

  [Serializable]
  public class FixedExponentialFilter : ExponentialFilter, IFilter<float>
  {
    [SerializeField]
    public float Alpha = 1f / 60f;

    public float Evaluate(float x)
    {
      return base.Evaluate(x, Alpha);
    }
  }
}
