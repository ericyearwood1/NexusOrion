using UnityEngine;
using UnityEngine.Events;

namespace CTRL
{
  /**
    This component is the main point of usage of the SDK.
    This is an abstract class, so users can't actually instantiate it.
    Instead, users should add the derived components from this class,
    which represent actual streams.

    This component provides implementations of methods to reduce duplication
    of logic between the stream specific components.
    */
  public abstract class StreamHandle<T> : ITK.DependBehavior
  {
    /// Set in inspector to override the default API stream requested
    [SerializeField]
    protected string streamName = "";
    public string StreamName => streamName == "" ? defaultStreamName : streamName;

    /// <summary>
    /// Derives from StreamName, mirroring the behavior of the JS SDK
    /// </summary>
    public string StreamId { get => StreamName.ToUpper(); }

    /// The majority of the time this will automatically fill itself.
    /// Public so it is available in to be overridden when connecting to
    /// multiple CTRL-R's.
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected CTRLClient client;
    public CTRLClient Client { get => client; }

    /// Overridden in stream specific components with the default API stream name.
    /// End users won't need to change this.
#pragma warning disable IDE1006
    protected virtual string defaultStreamName => StreamName;
#pragma warning restore IDE1006

    // Internal stream reference
    internal Stream<T> stream = null;

    public StreamState State
    {
      get
      {
        if (stream == null)
        {
          return StreamState.Disconnected;
        }

        return stream.State;
      }
    }

    [SerializeField]
    protected UnityEvent onConnect = new UnityEvent();
    public UnityEvent OnConnect { get => onConnect; }

    [SerializeField]
    protected UnityEvent onDisconnect = new UnityEvent();
    public UnityEvent OnDisconnect { get => onDisconnect; }

    protected virtual void OnEnable()
    {
      stream = client.RegisterStreamHandle(this);
    }

    protected virtual void OnDisable()
    {
      client.ReleaseStreamHandle(this);
      stream = null;
    }
  }
}
