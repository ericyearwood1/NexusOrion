using System;
using System.Collections.Generic;
using System.IO;
using System.Net.WebSockets;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

using AOT;
using System.Runtime.InteropServices;
using UnityEngine;
using System.Collections;

public class MainThreadUtil : MonoBehaviour
{
  public static MainThreadUtil Instance { get; private set; }
  public static SynchronizationContext synchronizationContext { get; private set; }

  [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
  public static void Setup()
  {
    Instance = new GameObject("MainThreadUtil")
        .AddComponent<MainThreadUtil>();
    synchronizationContext = SynchronizationContext.Current;
  }

  public static void Run(IEnumerator waitForUpdate)
  {
    synchronizationContext.Post(_ => Instance.StartCoroutine(
                waitForUpdate), null);
  }

  void Awake()
  {
    gameObject.hideFlags = HideFlags.HideAndDontSave;
    DontDestroyOnLoad(gameObject);
  }
}

public class WaitForUpdate : CustomYieldInstruction
{
  public override bool keepWaiting
  {
    get { return false; }
  }

  public MainThreadAwaiter GetAwaiter()
  {
    var awaiter = new MainThreadAwaiter();
    MainThreadUtil.Run(CoroutineWrapper(this, awaiter));
    return awaiter;
  }

  public class MainThreadAwaiter : INotifyCompletion
  {
    Action continuation;

    public bool IsCompleted { get; set; }

    public void GetResult() { }

    public void Complete()
    {
      IsCompleted = true;
      continuation?.Invoke();
    }

    void INotifyCompletion.OnCompleted(Action continuation)
    {
      this.continuation = continuation;
    }
  }

  public static IEnumerator CoroutineWrapper(IEnumerator theWorker, MainThreadAwaiter awaiter)
  {
    yield return theWorker;
    awaiter.Complete();
  }
}

namespace NativeWebSocket
{
  public delegate void WebSocketOpenEventHandler();
  public delegate void WebSocketMessageEventHandler(WebsocketMessage data);
  public delegate void WebSocketErrorEventHandler(string errorMsg);
  public delegate void WebSocketCloseEventHandler(WebSocketCloseCode closeCode);

  public enum WebSocketCloseCode
  {
    /* Do NOT use NotSet - it's only purpose is to indicate that the close code cannot be parsed. */
    NotSet = 0,
    Normal = 1000,
    Away = 1001,
    ProtocolError = 1002,
    UnsupportedData = 1003,
    Undefined = 1004,
    NoStatus = 1005,
    Abnormal = 1006,
    InvalidData = 1007,
    PolicyViolation = 1008,
    TooBig = 1009,
    MandatoryExtension = 1010,
    ServerError = 1011,
    TlsHandshakeFailure = 1015
  }

  public enum WebSocketState
  {
    Connecting,
    Open,
    Closing,
    Closed
  }

  public interface IWebSocket
  {
    event WebSocketOpenEventHandler OnOpen;
    event WebSocketMessageEventHandler OnMessage;
    event WebSocketErrorEventHandler OnError;
    event WebSocketCloseEventHandler OnClose;

    WebSocketState State { get; }
  }

  public struct WebsocketMessage
  {
    public byte[] Data { get; set; }

    public int Length { get; set; }

    public bool IsJSON()
    {
        return Data[0] == 0x7b;
    }

    public override string ToString()
    {
        return Encoding.UTF8.GetString(Data, 0, Length);
    }
  }

  public static class WebSocketHelpers
  {
    public static WebSocketCloseCode ParseCloseCodeEnum(int closeCode)
    {

      if (WebSocketCloseCode.IsDefined(typeof(WebSocketCloseCode), closeCode))
      {
        return (WebSocketCloseCode)closeCode;
      }
      else
      {
        return WebSocketCloseCode.Undefined;
      }

    }

    public static WebSocketException GetErrorMessageFromCode(int errorCode, Exception inner)
    {
      switch (errorCode)
      {
        case -1:
          return new WebSocketUnexpectedException("WebSocket instance not found.", inner);
        case -2:
          return new WebSocketInvalidStateException("WebSocket is already connected or in connecting state.", inner);
        case -3:
          return new WebSocketInvalidStateException("WebSocket is not connected.", inner);
        case -4:
          return new WebSocketInvalidStateException("WebSocket is already closing.", inner);
        case -5:
          return new WebSocketInvalidStateException("WebSocket is already closed.", inner);
        case -6:
          return new WebSocketInvalidStateException("WebSocket is not in open state.", inner);
        case -7:
          return new WebSocketInvalidArgumentException("Cannot close WebSocket. An invalid code was specified or reason is too long.", inner);
        default:
          return new WebSocketUnexpectedException("Unknown error.", inner);
      }
    }
  }

  public class WebSocketException : Exception
  {
    public WebSocketException() { }
    public WebSocketException(string message) : base(message) { }
    public WebSocketException(string message, Exception inner) : base(message, inner) { }
  }

  public class WebSocketUnexpectedException : WebSocketException
  {
    public WebSocketUnexpectedException() { }
    public WebSocketUnexpectedException(string message) : base(message) { }
    public WebSocketUnexpectedException(string message, Exception inner) : base(message, inner) { }
  }

  public class WebSocketInvalidArgumentException : WebSocketException
  {
    public WebSocketInvalidArgumentException() { }
    public WebSocketInvalidArgumentException(string message) : base(message) { }
    public WebSocketInvalidArgumentException(string message, Exception inner) : base(message, inner) { }
  }

  public class WebSocketInvalidStateException : WebSocketException
  {
    public WebSocketInvalidStateException() { }
    public WebSocketInvalidStateException(string message) : base(message) { }
    public WebSocketInvalidStateException(string message, Exception inner) : base(message, inner) { }
  }

  public class WaitForBackgroundThread
  {
    public ConfiguredTaskAwaitable.ConfiguredTaskAwaiter GetAwaiter()
    {
      return Task.Run(() => { }).ConfigureAwait(false).GetAwaiter();
    }
  }

#if UNITY_WEBGL && !UNITY_EDITOR

  /// <summary>
  /// WebSocket class bound to JSLIB.
  /// </summary>
  public class WebSocket : IWebSocket {

    /* WebSocket JSLIB functions */
    // IMPORTANT: NATIVE prepended to functions names to avoid conflicts with other
    // websocket jslibs that may exist in projects using com.ctrl.core
    [DllImport ("__Internal")]
    public static extern int NativeWebSocketConnect (int instanceId);

    [DllImport ("__Internal")]
    public static extern int NativeWebSocketClose (int instanceId, int code, string reason);

    [DllImport ("__Internal")]
    public static extern int NativeWebSocketSend (int instanceId, byte[] dataPtr, int dataLength);

    [DllImport ("__Internal")]
    public static extern int NativeWebSocketSendText (int instanceId, string message);

    [DllImport ("__Internal")]
    public static extern int NativeWebSocketGetState (int instanceId);

    public void DispatchMessageQueue()
    {
      Debug.LogError("How to implement DispatchMessageQueue?");
    }

    protected int instanceId;

    public event WebSocketOpenEventHandler OnOpen;
    public event WebSocketMessageEventHandler OnMessage;
    public event WebSocketErrorEventHandler OnError;
    public event WebSocketCloseEventHandler OnClose;

    public WebSocket (string url, Dictionary<string, string> headers = null) {
      if (!WebSocketFactory.isInitialized) {
        WebSocketFactory.Initialize ();
      }

      int instanceId = WebSocketFactory.NativeWebSocketAllocate (url);
      WebSocketFactory.instances.Add (instanceId, this);

      this.instanceId = instanceId;
    }

    public WebSocket (string url, string subprotocol, Dictionary<string, string> headers = null) {
      if (!WebSocketFactory.isInitialized) {
        WebSocketFactory.Initialize ();
      }

      int instanceId = WebSocketFactory.NativeWebSocketAllocate (url);
      WebSocketFactory.instances.Add (instanceId, this);

      WebSocketFactory.NativeWebSocketAddSubProtocol(instanceId, subprotocol);

      this.instanceId = instanceId;
    }

    public WebSocket (string url, List<string> subprotocols, Dictionary<string, string> headers = null) {
      if (!WebSocketFactory.isInitialized) {
        WebSocketFactory.Initialize ();
      }

      int instanceId = WebSocketFactory.NativeWebSocketAllocate (url);
      WebSocketFactory.instances.Add (instanceId, this);

      foreach (string subprotocol in subprotocols) {
        WebSocketFactory.NativeWebSocketAddSubProtocol(instanceId, subprotocol);
      }

      this.instanceId = instanceId;
    }

    ~WebSocket () {
      WebSocketFactory.HandleInstanceDestroy (this.instanceId);
    }

    public int GetInstanceId () {
      return this.instanceId;
    }

    public Task Connect () {
      int ret = NativeWebSocketConnect (this.instanceId);

      if (ret < 0)
        throw WebSocketHelpers.GetErrorMessageFromCode (ret, null);

      return Task.CompletedTask;
    }

	public void CancelConnection () {
		if (State == WebSocketState.Open)
			Close (WebSocketCloseCode.Abnormal);
	}

    public Task Close (WebSocketCloseCode code = WebSocketCloseCode.Normal, string reason = null) {
      int ret = NativeWebSocketClose (this.instanceId, (int) code, reason);

      if (ret < 0)
        throw WebSocketHelpers.GetErrorMessageFromCode (ret, null);

      return Task.CompletedTask;
    }

    public Task Send (byte[] data) {
      int ret = NativeWebSocketSend (this.instanceId, data, data.Length);

      if (ret < 0)
        throw WebSocketHelpers.GetErrorMessageFromCode (ret, null);

      return Task.CompletedTask;
    }

    public Task SendText (string message) {
      int ret = NativeWebSocketSendText (this.instanceId, message);

      if (ret < 0)
        throw WebSocketHelpers.GetErrorMessageFromCode (ret, null);

      return Task.CompletedTask;
    }

    public WebSocketState State {
      get {
        int state = NativeWebSocketGetState (this.instanceId);

        if (state < 0)
          throw WebSocketHelpers.GetErrorMessageFromCode (state, null);

        switch (state) {
          case 0:
            return WebSocketState.Connecting;

          case 1:
            return WebSocketState.Open;

          case 2:
            return WebSocketState.Closing;

          case 3:
            return WebSocketState.Closed;

          default:
            return WebSocketState.Closed;
        }
      }
    }

    public void DelegateOnOpenEvent () {
      this.OnOpen?.Invoke ();
    }

    public void DelegateOnMessageEvent (byte[] data) {
      this.OnMessage?.Invoke (new WebsocketMessage(){Data = data, Length = data.Length});
    }

    public void DelegateOnErrorEvent (string errorMsg) {
      this.OnError?.Invoke (errorMsg);
    }

    public void DelegateOnCloseEvent (int closeCode) {
      this.OnClose?.Invoke (WebSocketHelpers.ParseCloseCodeEnum (closeCode));
    }

  }

#else

  public class WebSocket : IWebSocket
  {
    public event WebSocketOpenEventHandler OnOpen;
    public event WebSocketMessageEventHandler OnMessage;
    public event WebSocketErrorEventHandler OnError;
    public event WebSocketCloseEventHandler OnClose;

    private Uri uri;
    private Dictionary<string, string> headers;
    private List<string> subprotocols;
    private ClientWebSocket m_Socket = new ClientWebSocket();

    private CancellationTokenSource m_TokenSource;
    private CancellationToken m_CancellationToken;

    private readonly object OutgoingMessageLock = new object();

    private bool isSending = false;
    private List<ArraySegment<byte>> sendBytesQueue = new List<ArraySegment<byte>>();
    private List<ArraySegment<byte>> sendTextQueue = new List<ArraySegment<byte>>();
            
    private WebsocketMessage[] m_MessageList = new WebsocketMessage[60];

    private ArraySegment<byte> m_RxBuffer = new(new byte[8192]);

    private int readIndex = 0;
    private int writeIndex = 0;

    public WebSocket(string url, Dictionary<string, string> headers = null)
            : this(url, new List<string>(), headers) { }

    public WebSocket(string url, string subprotocol, Dictionary<string, string> headers = null)
            : this(url, new List<string>() { subprotocol }, headers) { }

    public WebSocket(string url, List<string> subprotocols, Dictionary<string, string> headers = null)
    {
      uri = new Uri(url);

      if (headers == null)
      {
        this.headers = new Dictionary<string, string>();
      }
      else
      {
        this.headers = headers;
      }

      this.subprotocols = subprotocols;

      for (int i = 0; i < m_MessageList.Length; i++)
      {
        m_MessageList[i] = new WebsocketMessage();
        m_MessageList[i].Data = new byte[8192];
        m_MessageList[i].Length = 0;
      }

      string protocol = uri.Scheme;
      if (!protocol.Equals("ws") && !protocol.Equals("wss"))
        throw new ArgumentException("Unsupported protocol: " + protocol);
    }

    public void CancelConnection()
    {
      m_TokenSource?.Cancel();
    }

    public async Task Connect()
    {
      try
      {
        m_TokenSource = new CancellationTokenSource();
        m_CancellationToken = m_TokenSource.Token;

        m_Socket = new ClientWebSocket();

        foreach (var header in headers)
        {
          m_Socket.Options.SetRequestHeader(header.Key, header.Value);
        }

        foreach (string subprotocol in subprotocols)
        {
          m_Socket.Options.AddSubProtocol(subprotocol);
        }

        await m_Socket.ConnectAsync(uri, m_CancellationToken);
        OnOpen?.Invoke();

        await Receive();
      }
      catch (Exception ex)
      {
        OnError?.Invoke(ex.Message);
        OnClose?.Invoke(WebSocketCloseCode.Abnormal);
      }
      finally
      {
        if (m_Socket != null)
        {
          m_TokenSource.Cancel();
          m_Socket.Dispose();
        }
      }
    }

    public WebSocketState State
    {
      get
      {
        switch (m_Socket.State)
        {
          case System.Net.WebSockets.WebSocketState.Connecting:
            return WebSocketState.Connecting;

          case System.Net.WebSockets.WebSocketState.Open:
            return WebSocketState.Open;

          case System.Net.WebSockets.WebSocketState.CloseSent:
          case System.Net.WebSockets.WebSocketState.CloseReceived:
            return WebSocketState.Closing;

          case System.Net.WebSockets.WebSocketState.Closed:
            return WebSocketState.Closed;

          default:
            return WebSocketState.Closed;
        }
      }
    }

    public Task Send(byte[] bytes)
    {
      // return m_Socket.SendAsync(buffer, WebSocketMessageType.Binary, true, CancellationToken.None);
      return SendMessage(sendBytesQueue, WebSocketMessageType.Binary, new ArraySegment<byte>(bytes));
    }

    public Task SendText(string message)
    {
      var encoded = Encoding.UTF8.GetBytes(message);

      // m_Socket.SendAsync(buffer, WebSocketMessageType.Text, true, CancellationToken.None);
      return SendMessage(sendTextQueue, WebSocketMessageType.Text, new ArraySegment<byte>(encoded, 0, encoded.Length));
    }

    private async Task SendMessage(List<ArraySegment<byte>> queue, WebSocketMessageType messageType, ArraySegment<byte> buffer)
    {
      // Return control to the calling method immediately.
      // await Task.Yield ();

      // Make sure we have data.
      if (buffer.Count == 0)
      {
        return;
      }

      // The state of the connection is contained in the context Items dictionary.
      bool sending;

      lock (OutgoingMessageLock)
      {
        sending = isSending;

        // If not, we are now.
        if (!isSending)
        {
          isSending = true;
        }
      }

      if (!sending)
      {
        // Lock with a timeout, just in case.
        if (!Monitor.TryEnter(m_Socket, 1000))
        {
          // If we couldn't obtain exclusive access to the socket in one second, something is wrong.
          await m_Socket.CloseAsync(WebSocketCloseStatus.InternalServerError, string.Empty, m_CancellationToken);
          return;
        }

        try
        {
          // Send the message synchronously.
          var t = m_Socket.SendAsync(buffer, messageType, true, m_CancellationToken);
          t.Wait(m_CancellationToken);
        }
        finally
        {
          Monitor.Exit(m_Socket);
        }

        // Note that we've finished sending.
        lock (OutgoingMessageLock)
        {
          isSending = false;
        }

        // Handle any queued messages.
        await HandleQueue(queue, messageType);
      }
      else
      {
        // Add the message to the queue.
        lock (OutgoingMessageLock)
        {
          queue.Add(buffer);
        }
      }
    }

    private async Task HandleQueue(List<ArraySegment<byte>> queue, WebSocketMessageType messageType)
    {
      var buffer = new ArraySegment<byte>();
      lock (OutgoingMessageLock)
      {
        // Check for an item in the queue.
        if (queue.Count > 0)
        {
          // Pull it off the top.
          buffer = queue[0];
          queue.RemoveAt(0);
        }
      }

      // Send that message.
      if (buffer.Count > 0)
      {
        await SendMessage(queue, messageType, buffer);
      }
    }

    // simple dispatcher for queued messages.
    public void DispatchMessageQueue()
    {
      if (readIndex == writeIndex)
      {
        return;
      }

      int end = writeIndex;
      while (readIndex < end)
      {
        OnMessage?.Invoke(m_MessageList[readIndex++ % m_MessageList.Length]);
      }
    }

    public async Task Receive()
    {
      WebSocketCloseCode closeCode = WebSocketCloseCode.Abnormal;
      await new WaitForBackgroundThread();

      try
      {
        while (m_Socket.State == System.Net.WebSockets.WebSocketState.Open)
        {
          WebSocketReceiveResult result = null;

          int index = writeIndex % m_MessageList.Length;

          using (MemoryStream stream = new MemoryStream(m_MessageList[index].Data)) {
            do
            {
              result = await m_Socket.ReceiveAsync(m_RxBuffer, m_CancellationToken);

              stream.Write(m_RxBuffer.Array, m_RxBuffer.Offset, result.Count);
            }
            while (!result.EndOfMessage);

            m_MessageList[index].Length = (int) stream.Position;
          }

          Interlocked.Increment(ref writeIndex);
          if (result.MessageType == WebSocketMessageType.Close)
          {
            await Close();
            closeCode = WebSocketHelpers.ParseCloseCodeEnum((int)result.CloseStatus);
            break;
          }
        }
      }
      catch (Exception)
      {
        m_TokenSource.Cancel();
      }
      finally
      {
        await new WaitForUpdate();
        OnClose?.Invoke(closeCode);
      }
    }

    public async Task Close()
    {
      if (State == WebSocketState.Open)
      {
        await m_Socket.CloseAsync(WebSocketCloseStatus.NormalClosure, string.Empty, m_CancellationToken);
      }
    }
  }
#endif

  ///
  /// Factory
  ///

  /// <summary>
  /// Class providing static access methods to work with JSLIB WebSocket or WebSocketSharp interface
  /// </summary>
  public static class WebSocketFactory
  {

#if UNITY_WEBGL && !UNITY_EDITOR
    /* Map of websocket instances */
    public static Dictionary<Int32, WebSocket> instances = new Dictionary<Int32, WebSocket> ();

    /* Delegates */
    public delegate void OnOpenCallback (int instanceId);
    public delegate void OnMessageCallback (int instanceId, System.IntPtr msgPtr, int msgSize);
    public delegate void OnErrorCallback (int instanceId, System.IntPtr errorPtr);
    public delegate void OnCloseCallback (int instanceId, int closeCode);

    /* WebSocket JSLIB callback setters and other functions */
    [DllImport ("__Internal")]
    public static extern int NativeWebSocketAllocate (string url);

    [DllImport ("__Internal")]
    public static extern int NativeWebSocketAddSubProtocol (int instanceId, string subprotocol);

    [DllImport ("__Internal")]
    public static extern void NativeWebSocketFree (int instanceId);

    [DllImport ("__Internal")]
    public static extern void NativeWebSocketSetOnOpen (OnOpenCallback callback);

    [DllImport ("__Internal")]
    public static extern void NativeWebSocketSetOnMessage (OnMessageCallback callback);

    [DllImport ("__Internal")]
    public static extern void NativeWebSocketSetOnError (OnErrorCallback callback);

    [DllImport ("__Internal")]
    public static extern void NativeWebSocketSetOnClose (OnCloseCallback callback);

    /* If callbacks was initialized and set */
    public static bool isInitialized = false;

    /*
     * Initialize WebSocket callbacks to JSLIB
     */
    public static void Initialize () {

      NativeWebSocketSetOnOpen (DelegateOnOpenEvent);
      NativeWebSocketSetOnMessage (DelegateOnMessageEvent);
      NativeWebSocketSetOnError (DelegateOnErrorEvent);
      NativeWebSocketSetOnClose (DelegateOnCloseEvent);

      isInitialized = true;

    }

    /// <summary>
    /// Called when instance is destroyed (by destructor)
    /// Method removes instance from map and free it in JSLIB implementation
    /// </summary>
    /// <param name="instanceId">Instance identifier.</param>
    public static void HandleInstanceDestroy (int instanceId) {

      instances.Remove (instanceId);
      NativeWebSocketFree (instanceId);

    }

    [MonoPInvokeCallback (typeof (OnOpenCallback))]
    public static void DelegateOnOpenEvent (int instanceId) {

      WebSocket instanceRef;

      if (instances.TryGetValue (instanceId, out instanceRef)) {
        instanceRef.DelegateOnOpenEvent ();
      }

    }

    [MonoPInvokeCallback (typeof (OnMessageCallback))]
    public static void DelegateOnMessageEvent (int instanceId, System.IntPtr msgPtr, int msgSize) {

      WebSocket instanceRef;

      if (instances.TryGetValue (instanceId, out instanceRef)) {
        byte[] msg = new byte[msgSize];
        Marshal.Copy (msgPtr, msg, 0, msgSize);

        instanceRef.DelegateOnMessageEvent (msg);
      }

    }

    [MonoPInvokeCallback (typeof (OnErrorCallback))]
    public static void DelegateOnErrorEvent (int instanceId, System.IntPtr errorPtr) {

      WebSocket instanceRef;

      if (instances.TryGetValue (instanceId, out instanceRef)) {

        string errorMsg = Marshal.PtrToStringAuto (errorPtr);
        instanceRef.DelegateOnErrorEvent (errorMsg);

      }

    }

    [MonoPInvokeCallback (typeof (OnCloseCallback))]
    public static void DelegateOnCloseEvent (int instanceId, int closeCode) {

      WebSocket instanceRef;

      if (instances.TryGetValue (instanceId, out instanceRef)) {
        instanceRef.DelegateOnCloseEvent (closeCode);
      }

    }
#endif

    /// <summary>
    /// Create WebSocket client instance
    /// </summary>
    /// <returns>The WebSocket instance.</returns>
    /// <param name="url">WebSocket valid URL.</param>
    public static WebSocket CreateInstance(string url)
    {
      return new WebSocket(url);
    }

  }

}
