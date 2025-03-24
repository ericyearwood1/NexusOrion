using System;

namespace CTRL.Math.Filters
{
  public interface IFilter<T, U>
  {
    void Reset();
    U Evaluate(T val);
  }

  public interface IFilter<T> : IFilter<T, T> { }
}
