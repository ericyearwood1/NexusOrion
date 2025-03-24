using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace CTRL
{
  public enum StreamState
  {
    Disconnected,
    Requested,
    Connected,
    Disconnecting,
  }

  public interface IStream
  {
    // Config
    string StreamId { get; }
    string StreamName { get; }

    // State to be updated
    StreamState State { get; set; }
    int HandleCount { get; }

    // Lifecycle
    void DidConnect();
    void ProcessBatch(JToken msg);
    void ProcessBatch<T>(StreamBatch<T> msg);
    void DidDisconnect();
  }

  public class Stream<T> : IStream
  {
    /// The stream_id assigned to this stream for communication with CTRL-R. Must be unique per session.
    public string StreamId { get; private set; }

    /// The name of the stream requested. Corresponds to the API to request.
    public string StreamName { get; private set; }

    /// State of the request of this stream
    public StreamState State { get; set; } = StreamState.Disconnected;

    /// Can be polled for closing
    public int HandleCount { get => handles.Count; }

    // Internal store
    protected HashSet<StreamHandle<T>> handles = new HashSet<StreamHandle<T>>();

    public Stream(string streamName, string streamId)
    {
      this.StreamName = streamName;
      this.StreamId = streamId;
    }

    public void ProcessBatch(JToken msg)
    {
      var batchMsg = msg[StreamName];
      if (batchMsg == null)
      {
        // TODO: this should be a validation error of some kind
        return;
      }

      ProcessBatch(batchMsg.ToObject<StreamBatch<T>>());
    }

    public void ProcessBatch<T>(StreamBatch<T> streamBatch)
    {
      foreach (var handle in handles)
      {
        if (handle is OutputStreamHandle<T> oh)
        {
          oh.ProcessStreamBatch(streamBatch);
        }
      }
    }

    public void AddHandle(StreamHandle<T> handle)
    {
      handles.Add(handle);
    }

    public void RemoveHandle(StreamHandle<T> handle)
    {
      handles.Remove(handle);
    }

    public void DidConnect()
    {
      foreach (var handle in handles)
      {
        handle.OnConnect.Invoke();
      }
    }

    public void DidDisconnect()
    {
      foreach (var handle in handles)
      {
        handle.OnDisconnect.Invoke();
      }
    }
  }
}
