using UnityEngine;
using UnityEngine.Events;

using System;
using System.Collections.Generic;
using System.Threading.Tasks;

using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using NativeWebSocket;

using CTRL.Utils;
using CTRL.Utils.Logging;
using CTRL.Data;

namespace CTRL
{
  public enum ConnectionState
  {
    Disconnected,
    Connected,
  }

  [System.Serializable]
  public class ErrorEvent : UnityEvent<string> { }

  [System.Serializable]
  public class MessageEvent : UnityEvent<string> { }

  /// <summary>
  /// This component handles connecting to CTRL-R, requesting streams
  /// and dispatching received messages to streams.
  /// A GameObject with CTRLR will be automatically added to the scene
  /// if an OutputStreamHandle is added to the scene but no CTRLR is
  /// found in the scene.
  /// </summary>
  [DefaultExecutionOrder(-85)]
  public class CTRLClient : ITK.DependBehavior
  {
    public static readonly string API_VERSION = "0.12";

    [Header("Connection")]

    /// Host name or IP Address of CTRL-R
    public string Host = "127.0.0.1";

    /// Port of CTRL-R
    public int Port = 9999;

    /// Application id for CTRL-R. Automatically filled if empty with `Application.productName`
    [Tooltip("If empty, \"unity-app-Application.productName\" will be used")]
    public string AppId = "";

    // we use internal field to avoid persisting AppId unless user explicitly specified it
    private string _appId = "";

    /// Seconds between reconnect attempts
    [SerializeField]
    protected float reconnectBackoffSec = 0.5f;

    /// Number of messages to keep in the queue; stream batches beyond this will be discarded.
    [SerializeField]
    protected uint messageQueueSize = 256;

    [System.Flags]
    public enum LogCategory : int
    {
      None = 0,
      Connection = 1 << 0,
      ReconnectAttempt = 1 << 1,
      APIError = 1 << 2,
      SentMessages = 1 << 3,
      ReceivedMessages = 1 << 4,
      Everything = ~0
    }

    [Header("Logging")]

    [SerializeField]
    private CTRLLogger logger = new CTRLLogger("CTRLClient");
    public CTRLLogger Logger => logger;

    [SerializeField]
    public LogCategory logCategory = LogCategory.Connection | LogCategory.APIError;

    /// Show the connection state
    public ConnectionState State { get; protected set; } = ConnectionState.Disconnected;

    [Header("Events")]

    /// Lifecycle events
    [SerializeField]
    protected UnityEvent onConnect = new UnityEvent();
    public UnityEvent OnConnect { get => onConnect; }

    [SerializeField]
    protected UnityEvent onDisconnect = new UnityEvent();
    public UnityEvent OnDisconnect { get => onDisconnect; }

    [SerializeField]
    protected ErrorEvent onError = new ErrorEvent();
    public ErrorEvent OnError { get => onError; }
    [SerializeField]
    protected UnityEvent<string> onMessage = new MessageEvent();
    public UnityEvent<string> OnMessage = new MessageEvent();

    public List<Func<Task<bool>>> ConfigCallbacks = new List<Func<Task<bool>>>();

    // Networking
    protected WebSocket ctrlWebsocket;
    protected float lastReconnectTime = 0;
    protected bool isConnecting = false;

    // Counter for communication with CTRL-R
    protected static uint requestId = 1;
    protected Dictionary<uint, TaskCompletionSource<JToken>> requests = new Dictionary<uint, TaskCompletionSource<JToken>>();

    // The thread-safe message queue used to move messages from the
    // websocket thread to the main thread for Unity.
    // Not using ConcurrentQueue<T> as that seems to have bad performance in webgl
    protected SliceableFIFO<object> messageQueue = new SliceableFIFO<object>(256);

    // Mapping from stream_id to Stream<T> (generic parameter erased with IBatchProcessor)
    protected Dictionary<string, IStream> streams = new Dictionary<string, IStream>();
    public IEnumerable<IStream> Streams { get => streams.Values; }

    #region Stream Handle Management

    // Construct and register a Stream with this backend, and return it. If another Stream with
    // the same streamId already exists, return that instead.
    internal Stream<T> RegisterStreamHandle<T>(StreamHandle<T> handle)
    {
      if (!streams.TryGetValue(handle.StreamId, out IStream stream))
      {
        stream = new Stream<T>(handle.StreamName, handle.StreamId);
        streams.Add(handle.StreamId, stream);
      }

      if (handle.StreamName != stream.StreamName)
      {
        throw new System.Exception($"Attempted to use different stream name {handle.StreamName} for already requested id {handle.StreamId}");
      }

      var typedStream = stream as Stream<T>;
      typedStream.AddHandle(handle);
      return typedStream;
    }

    internal void ReleaseStreamHandle<T>(StreamHandle<T> handle)
    {
      if (!streams.TryGetValue(handle.StreamId, out IStream stream))
      {
        throw new System.Exception($"Attempted to un-requested release stream for id {handle.StreamId}!");
      }

      (stream as Stream<T>).RemoveHandle(handle);
    }

    #endregion

    #region Logging

    protected void Log(LogType logType, LogCategory category, string msg)
    {
      var tag = Enum.GetName(typeof(LogCategory), category);
      logger.logEnabled = ((category & logCategory) != LogCategory.None);
      logger.Log(logType, tag, msg);
    }

    protected void Log(LogCategory category, string msg)
    {
      Log(LogType.Log, category, msg);
    }

    protected void LogError(LogCategory category, string msg)
    {
      Log(LogType.Error, category, msg);
      OnError.Invoke(msg);
    }

    protected void LogWarning(LogCategory category, string msg)
    {
      Log(LogType.Warning, category, msg);
    }

    protected void LogException(Exception e)
    {
      logger.logEnabled = true;
      logger.LogException(e);
    }

    #endregion

    #region Websocket Callbacks

    protected void RegisterWebsocket(WebSocket ws)
    {
      ws.OnOpen += OnWSOpen;
      ws.OnClose += OnWSClose;
      ws.OnError += OnWSError;
      ws.OnMessage += OnWSMessage;
    }

    protected void DeregisterWebsocket(WebSocket ws)
    {
      if (ws == null)
      {
        return;
      }

      ws.OnOpen -= OnWSOpen;
      ws.OnClose -= OnWSClose;
      ws.OnError -= OnWSError;
      ws.OnMessage -= OnWSMessage;
    }

    protected void OnWSOpen()
    {
      DidConnect();
    }

    protected void OnWSClose(WebSocketCloseCode closeCode)
    {
      CleanupWebsocket();
      DidDisconnect();
    }

    protected void OnWSError(string errMsg)
    {
      LogCategory category = LogCategory.ReconnectAttempt;
      if (State == ConnectionState.Connected)
      {
        category = LogCategory.Connection;
      }

      LogError(category, string.Format("Websocket error: {0}:{1} {2}", Host, Port, errMsg));
    }

    protected void OnWSMessage(WebsocketMessage message)
    {
      if (message.IsJSON())
      {
        EnqueueMessage(message.ToString());
      }
      else
      {
        EnqueueMessage(message);
      }
    }

    #endregion

    #region Connection

    protected void CleanupWebsocket()
    {
      // Clean up resources
      var ws = ctrlWebsocket;
      ctrlWebsocket = null;

      if (ws != null)
      {
        DeregisterWebsocket(ws);
        ws.CancelConnection();
        Task.Run(() => ws.Close());
      }
    }

    protected async Task RunConfigCallbacks()
    {
      // Make a copy in case list changes during async execution.
      var callbacks = new List<Func<Task<bool>>>(ConfigCallbacks);

      try
      {
        foreach (var callback in callbacks)
        {
          if (await callback())
          {
            // When the callback returns true, short-circuit any remaining callbacks.
            break;
          }
        }
      }
      catch (Exception e)
      {
        LogException(e);
      }
    }

    protected async Task ConnectInternal()
    {
      await RunConfigCallbacks();

      var url = $"ws://{Host}:{Port}";
      Log(LogCategory.ReconnectAttempt, string.Format("Attempting websocket connection to {0}", url));

      // Deregister old callbacks
      if (ctrlWebsocket != null)
      {
        CleanupWebsocket();
      }

      // Create new websocket instance and register callbacks
      try
      {
        ctrlWebsocket = WebSocketFactory.CreateInstance(url);
      }
      catch (UriFormatException)
      {
        LogWarning(LogCategory.Connection, $"Invalid host or port in URL '{url}'");
        return;
      }

      RegisterWebsocket(ctrlWebsocket);

      try
      {
        await ctrlWebsocket.Connect();
      }
      catch (WebSocketException e)
      {
        LogError(LogCategory.Connection, $"Error connecting: {e}");
        CleanupWebsocket();
      }
    }

    protected async void Connect()
    {
      isConnecting = true;

      try
      {
        await ConnectInternal();
      }
      finally
      {
        isConnecting = false;
      }
    }

    protected void DidConnect()
    {
      State = ConnectionState.Connected;
      OnConnect.Invoke();
    }

    internal void Disconnect()
    {
      CleanupWebsocket();
      DidDisconnect();
    }

    protected void DidDisconnect()
    {
      // Clear any outstanding requests
      foreach (var pair in requests)
      {
        if (!pair.Value.Task.IsCompleted)
          pair.Value.SetException(new System.Exception("Disconnected before completion"));
      }
      requests.Clear();

      State = ConnectionState.Disconnected;

      OnDisconnect.Invoke();

      foreach (var stream in streams.Values)
      {
        stream.State = StreamState.Disconnected;
        stream.DidDisconnect();
      }
    }

    #endregion

    #region Process API Messages

    protected void ProcessStreamBatchMessage(JToken msg)
    {
      var streamId = msg["stream_id"];

      if (!streams.TryGetValue((string)streamId, out IStream stream))
      {
        LogError(LogCategory.APIError, $"Received message for streamId {streamId} but no stream with that id was registered");
        return;
      }
      stream.ProcessBatch(msg);
    }

    private double ToDouble(byte[] data, ref int offset)
    {
        double value = BitConverter.ToDouble(data, offset);
        offset += sizeof(double);

        return value;
    }

    private float ToFloat(byte[] data, ref int offset)
    {
        float value = BitConverter.ToSingle(data, offset);
        offset += sizeof(float);

        return value;
    }

    private int ToInt(byte[] data, ref int offset)
    {
        int value = BitConverter.ToInt32(data, offset);
        offset += sizeof(int);

        return value;
    }

    private int batchNumber = 0;
    protected void ProcessStreamBatchMessage(WebsocketMessage message)
    {
        // TODO just assume that this is the binary IMU stream for now
        if (!streams.TryGetValue("RAW_IMU", out var stream) && !streams.TryGetValue("RAW_IMU_HALF", out stream)) return;

        StreamBatch<IMU> batch = new StreamBatch<IMU>();
        batch.batch_num = batchNumber++;
        batch.samples = new List<Sample<IMU>>();

        int offset = 1;

        int count = ToInt(message.Data, ref offset);
        for (int i = 0; i < count; i++)
        {
            Sample<IMU> imu = new Sample<IMU>();
            imu.timestamp_s = ToDouble(message.Data, ref offset);

            imu.data.calibrated = true;

            imu.data.orientation.w = ToFloat(message.Data, ref offset);
            imu.data.orientation.x = ToFloat(message.Data, ref offset);
            imu.data.orientation.y = ToFloat(message.Data, ref offset);
            imu.data.orientation.z = ToFloat(message.Data, ref offset);
            batch.samples.Add(imu);
        }

        stream.ProcessBatch(batch);
    }

    protected void ProcessAPIEventMessage(JToken msg)
    {
      if (msg["start_stream_event"] != null)
      {
        ProcessStartStreamMessage(msg["start_stream_event"]);
      }
      else if (msg["end_stream_event"] != null)
      {
        ProcessEndStreamMessage(msg["end_stream_event"]);
      }
      else if (msg["error_event"] != null)
      {
        ProcessErrorMessage(msg["error_event"]);
      }
      else
      {
        if (msg["request_id"] != null)
        {
          // Handle tasks
          var requestId = (uint)msg["request_id"];
          requests.TryGetValue(requestId, out TaskCompletionSource<JToken> source);
          if (source != null)
          {
            source.SetResult(msg);
          }
          else
          {
            LogWarning(LogCategory.APIError, $"Received response for request_id {requestId} but have no record of this request");
          }
        }
      }
    }

    protected void ProcessStartStreamMessage(JToken msg)
    {
      var streamId = msg["stream_id"];

      if (!streams.TryGetValue((string)streamId, out IStream stream))
      {
        LogError(LogCategory.APIError, $"Received message for streamId {streamId} but no stream with that id was registered");
        return;
      }

      stream.State = StreamState.Connected;
      stream.DidConnect();
    }

    protected void ProcessEndStreamMessage(JToken msg)
    {
      var streamId = msg["stream_id"];

      if (!streams.TryGetValue((string)streamId, out IStream stream))
      {
        LogError(LogCategory.APIError, $"Received message for streamId {streamId} but no stream with that id was registered");
        return;
      }

      stream.State = StreamState.Disconnected;
      stream.DidDisconnect();
    }

    protected void ProcessErrorMessage(JToken msg)
    {
      var errStr = string.Format(
        "Received error: {0} [{1}] (request_id:{2})",
        msg["error_code"], msg["desc"], msg["request_id"] ?? "unknown"
      );
      LogError(LogCategory.APIError, errStr);

      if (msg["request_id"] != null)
      {
        var requestId = (uint)msg["request_id"];
        requests.TryGetValue(requestId, out TaskCompletionSource<JToken> source);
        if (source != null)
        {
          source.SetException(new System.Exception(errStr));
        }
        else
        {
          LogWarning(LogCategory.APIError, $"Received error for request_id {requestId} but have no record of this request");
        }
      }
    }

    protected void ProcessAPIRequestMessage(JToken _)
    {
      throw new System.Exception("Received API Request");
    }

    protected void ProcessLatencyStreamMessage(JToken msg)
    {
      if (msg["timestamp"] != null)
      {
        Send(new JObject {
          { "latency_stream", msg }
        });
      }
    }

    protected void ProcessMessage(JToken msg)
    {
      if (msg["stream_batch"] != null)
      {
        ProcessStreamBatchMessage(msg["stream_batch"]);
      }
      else if (msg["api_event"] != null)
      {
        ProcessAPIEventMessage(msg["api_event"]);
      }
      else if (msg["api_request"] != null)
      {
        ProcessAPIRequestMessage(msg["api_request"]);
      }
      else if (msg["latency_stream"] != null)
      {
        ProcessLatencyStreamMessage(msg);
      }
    }

    private bool _isFull;
    private void EnqueueMessage(WebsocketMessage message)
    {
      if (messageQueue.Count < messageQueue.Capacity)
      {
        _isFull = false;
        messageQueue.Enqueue(message);
        return;
      }

      if (_isFull) return;

      LogWarning(
        LogCategory.Connection,
        "Message Queue is full, dropping old messages. This can be due to " +
        "having insufficent message queue size for the number of streams, " +
        "or due to low FPS. Increase message queue size in this component " +
        "to ensure all batches get delivered."
      );
      _isFull = true;
    }

    protected void EnqueueMessage(string strmsg)
    {
      JToken msg;

      try
      {
        msg = JObject.Parse(strmsg);
      }
      catch (System.Exception e)
      {
        LogError(LogCategory.APIError, string.Format("JSON parsing when parsing {0}", strmsg));
        throw e;
      }

      if (messageQueue.Count < messageQueue.Capacity)
      {
        _isFull = false;
        messageQueue.Enqueue(msg);
        return;
      }

      // If we're at capacity, dequeue a message to drop
      object dequeue = messageQueue.Dequeue();
      if (dequeue is JToken jToken)
      {
        // only drop stream batches
        // heuristic reason: Unity apps often run slow the first few frames
        // In this case we don't want to drop api events (stream creation)
        if (jToken["stream_batch"] == null)
        {
          messageQueue.Enqueue(jToken);
        }
        else
        {
          messageQueue.Enqueue(msg);
        }
      }

      if (_isFull) return;

      LogWarning(
        LogCategory.Connection,
        "Message Queue is full, dropping old messages. This can be due to " +
        "having insufficent message queue size for the number of streams, " +
        "or due to low FPS. Increase message queue size in this component " +
        "to ensure all batches get delivered."
      );
      _isFull = true;
    }

    #endregion

    #region Unity Lifecycle

    protected void ManageStreams()
    {
      var toClose = new List<string>();
      foreach (var stream in streams.Values)
      {
        if (stream.HandleCount == 0)
        {
          toClose.Add(stream.StreamId);
        }
        else if (stream.State != StreamState.Requested && stream.State != StreamState.Connected)
        {
          if (State == ConnectionState.Connected)
          {
            stream.State = StreamState.Requested;
            SendStartStreamRequest(stream.StreamName, stream.StreamId);
          }
        }
      }

      foreach (var streamId in toClose)
      {
        streams.Remove(streamId);

        if (State == ConnectionState.Connected)
        {
          SendEndStreamRequest(streamId);
        }
      }
    }

    protected virtual void Update()
    {
      // Reconnect every fixed interval
      if (State != ConnectionState.Connected && !isConnecting)
      {
        float now = Time.time;
        if (lastReconnectTime < 0 || now - lastReconnectTime > reconnectBackoffSec)
        {
          lastReconnectTime = now;
          Connect();
        }
      }

      // Read messages from the web socket
      // For WebGL, this all happens via the jslib
#if !UNITY_WEBGL || UNITY_EDITOR
      if (ctrlWebsocket != null)
      {
        ctrlWebsocket.DispatchMessageQueue();
      }
#endif

      // Handle internal message queue
      var msgCount = messageQueue.Count;
      for (int i = 0; i < msgCount; i++)
      {
        var msg = messageQueue.Dequeue();
        if (msg is JToken)
        {
          var m = (JToken) msg;
          ProcessMessage(m);
          OnMessage.Invoke(m.ToString());
        }
        else if (msg is WebsocketMessage)
        {
          ProcessStreamBatchMessage((WebsocketMessage)msg);
        }
      }

      // Manage stream connection and disconnection
      ManageStreams();
    }

    protected virtual void OnValidate()
    {

      if (messageQueue == null || messageQueue.Capacity != messageQueueSize)
      {
        messageQueue = new SliceableFIFO<object>(messageQueueSize);
      }

      // Connecting to "localhost" was failing, but substituting an IP seemed to work
      if (Host == "localhost")
      {
        Host = "127.0.0.1";
      }
    }

    protected virtual void OnEnable()
    {
      _appId = AppId != "" ? AppId : "unity-app-" + Application.productName;
      lastReconnectTime = -1;
    }

    protected virtual void OnDisable()
    {
      Disconnect();
    }

    #endregion

    #region Send API Messages

    public void SendStartStreamRequest(string streamName, string streamId)
    {
      SendRequest(new JObject {
        { "start_stream_request", new JObject {
          { "stream_id",  streamId },
          { "app_id", _appId },
          { streamName, new JObject() }
        }}
      });
    }

    public void SendEndStreamRequest(string streamId)
    {
      SendRequest(new JObject {
        { "end_stream_request", new JObject {
          { "stream_id",  streamId },
        }}
      });
    }

    public Task<JToken> SendSetGraphRequest(string graph)
    {
      return SendRequest(new JObject {
        { "set_pipeline", new JObject {
          { "graph",  graph },
        }}
      });
    }

    public void SendParameterChangeRequest(string transformID, string parameterName, string parameterValue)
    {
      SendRequest(new JObject {
        {"change_parameter_request", new JObject {
          {"transforms", new JObject {
            {transformID, new JObject {
              {"parameters", new JObject {
                {parameterName, parameterValue}
              }}
            }}
          }}
        }}
      });
    }

    public void SendStartLatencyStream()
    {
      Send(new JObject {
        { "start_latency_stream", "<dummy>" }
      });
    }

    public Task<JToken> SendRequest(JObject req)
    {
      var reqId = requestId++;
      req["request_id"] = reqId;

      var wrapper = new JObject {
        { "api_version", API_VERSION },
        { "api_request", req }
      };

      Send(JObject.FromObject(wrapper));

      var source = new TaskCompletionSource<JToken>();
      requests.Add(reqId, source);
      return source.Task;
    }

    public void SendStreamBatch<T>(string streamName, string streamId, List<Sample<T>> samples)
    {
      Send(new JObject {
        { "api_version", API_VERSION },
        { "stream_batch", new JObject {
          { "stream_id",  streamId },
          { streamName, new JObject {
            {"samples", JArray.FromObject(samples) }
          }}
        }}
      });
    }

    /// Manually send a message to CTRL-R
    public void Send(string msg)
    {
      if (ctrlWebsocket == null || ctrlWebsocket.State != WebSocketState.Open)
      {
        throw new System.Exception("CTRL-R is not connected");
      }

      Log(LogCategory.SentMessages, $"Sending: {msg}");
      ctrlWebsocket.Send(msg.ToByteArray());
    }

    public void Send(JToken msg)
    {
      try
      {
        var msgstr = msg.ToString(Formatting.None);
        Send(msgstr);
      }
      catch (JsonException e)
      {
        LogError(LogCategory.APIError, $"JSONException {e} sending {msg}");
      }
    }

    #endregion
  }

}
