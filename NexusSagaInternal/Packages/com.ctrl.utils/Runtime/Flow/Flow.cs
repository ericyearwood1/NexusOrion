using System;
using UnityEngine.Events;

namespace CTRL.Flow
{
  public interface IEmittable<T>
  {
    void Invoke(T val);
  }

  public interface ISubscribable<T>
  {
    void AddListener(UnityAction<T> fn);
    void RemoveListener(UnityAction<T> fn);
  }

  public class Pipe<T> : UnityEvent<T>, IEmittable<T>, ISubscribable<T>
  {
    protected T latest = default;
    public T Latest { get => latest; }

    public new void Invoke(T val)
    {
      latest = val;
      base.Invoke(val);
    }

    public Pipe<T> Tap(UnityAction<T> fn)
    {
      AddListener(fn);
      return this;
    }

    public Pipeline<T, U> Chain<U>(Func<T, U> fn)
    {
      return new Pipeline<T, T>(this, this).Chain(fn);
    }
  }

  public class Pipeline<T, U> : IEmittable<T>, ISubscribable<U>
  {
    protected Pipe<T> head;
    public Pipe<T> Head { get => head; }

    protected Pipe<U> tail;
    public Pipe<U> Tail { get => tail; }

    internal Pipeline(Pipe<T> head, Pipe<U> tail)
    {
      this.head = head;
      this.tail = tail;
    }

    public Pipeline<T, U> Tap(UnityAction<U> fn)
    {
      tail.AddListener(fn);
      return this;
    }

    public Pipeline<T, V> Chain<V>(Func<U, V> fn)
    {
      var newTail = new Pipe<V>();
      this.tail.AddListener((v) =>
      {
        var res = fn(v);
        if (v != null)
        {
          newTail.Invoke(res);
        }
      });

      return new Pipeline<T, V>(this.head, newTail);
    }

    public void AddListener(UnityAction<U> fn)
    {
      tail.AddListener(fn);
    }

    public void RemoveListener(UnityAction<U> fn)
    {
      tail.RemoveListener(fn);
    }

    public void Invoke(T val)
    {
      head.Invoke(val);
    }
  }

  public static class UnityEventExtensions
  {
    public static void AddListener<T, U>(this UnityEvent<T> ev, Pipeline<T, U> pipeline)
    {
      ev.AddListener(pipeline.Invoke);
    }

    public static void RemoveListener<T, U>(this UnityEvent<T> ev, Pipeline<T, U> pipeline)
    {
      ev.RemoveListener(pipeline.Invoke);
    }

    public static void AddListener<T, U>(this UnityEvent<T> ev, Pipe<T> pipe)
    {
      ev.AddListener(pipe.Invoke);
    }

    public static void RemoveListener<T, U>(this UnityEvent<T> ev, Pipe<T> pipe)
    {
      ev.RemoveListener(pipe.Invoke);
    }
  }

  public static class FlowUtil
  {
    public static Pipeline<T, T> Start<T>()
    {
      var pipe = new Pipe<T>();
      return new Pipeline<T, T>(pipe, pipe);
    }

    public static UnityAction Proxy<T>(ISubscribable<T> src, IEmittable<T> dst)
    {
      void fn(T val) => dst.Invoke(val);
      src.AddListener(fn);
      return () => { src.RemoveListener(fn); };
    }

    public static UnityAction Proxy<T>(ISubscribable<T> src, UnityEvent<T> dst)
    {
      void fn(T val) => dst.Invoke(val);
      src.AddListener(fn);
      return () => { src.RemoveListener(fn); };
    }

    public static UnityAction Proxy<T>(UnityEvent<T> src, IEmittable<T> dst)
    {
      void fn(T val) => dst.Invoke(val);
      src.AddListener(fn);
      return () => { src.RemoveListener(fn); };
    }

    public static UnityAction Proxy<T>(UnityEvent<T> src, UnityEvent<T> dst)
    {
      void fn(T val) => dst.Invoke(val);
      src.AddListener(fn);
      return () => { src.RemoveListener(fn); };
    }
  }
}
