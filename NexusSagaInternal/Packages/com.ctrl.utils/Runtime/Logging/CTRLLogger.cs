using System;
using System.Collections.Generic;

using UnityEngine;

namespace CTRL.Utils.Logging
{
  /// <summary>
  /// An implementation of `UnityEditor.ILogger` that logs to both the Unity
  /// developer console and Windows Device Portal (WDP).
  /// </summary>
  /// <remarks>
  /// There are two main log handlers created by `CTRLLogger`:
  ///
  /// 1. A wrapper around the default Unity console logger. The `filterLogType`
  /// property controls filtering for this log handler.
  ///
  /// 2. A `WDPLogHandler` that allows streaming realtime logs in WDP. This log
  /// handler is only active in UWP/HL2 builds. Each `CTRLLogger` instance
  /// generates a unique GUID based on the input `tag` value. This GUID is used
  /// to enable streaming for that particular logger in WDP.
  ///
  /// To enable realtime logging in Windows Device Portal:
  ///
  /// 1. Find the `CTRLLogger` instance in the Unity editor inspector view,
  /// then expand and copy the "WDP GUID" value.
  ///
  /// 2. Open a web browser and navigate to <a
  /// href="http://&lt;hololens-host&gt;/#Logging">http://&lt;hololens-host&gt;/#Logging</a>.
  ///
  /// 3. Paste the copied GUID value into the "Custom providers" field, then
  /// click "Enable".
  ///
  /// 4. Watch for log messages in "Events" section.
  /// </remarks>
  [Serializable]
  public class CTRLLogger : ILogger, ISerializationCallbackReceiver
  {
    // ILogger properties

    // Note: this property only applies to Unity developer console logging, not
    // WDP logging.
    [SerializeField]
    private LogType _filterLogType;
    public LogType filterLogType
    {
      get => _filterLogType;
      set => _filterLogType = value;
    }

    // Note: this property only applies to Unity developer console logging, not
    // WDP logging.
    [SerializeField]
    private bool _logEnabled;
    public bool logEnabled
    {
      get => _logEnabled;
      set => _logEnabled = value;
    }

    private ILogHandler _logHandler;
    public ILogHandler logHandler
    {
      get => _logHandler;
      set => _logHandler = value;
    }

    // CTRLLogger properties

    [SerializeField]
    private string _tag;
    public string tag
    {
      get => _tag;
      set { _tag = value; OnTagChange(); }
    }

    [SerializeField]
    private string _guid;
    public string guid => _guid;

    // CTRLLogger methods

    public CTRLLogger(string tag, LogType filterLogType = LogType.Log)
    {
      _filterLogType = filterLogType;
      _logEnabled = true;
      _tag = tag;
      OnTagChange();
    }

    // ILogger methods

    public void Log(LogType logType, object message)
    {
      Log(logType, message, null);
    }

    public void Log(LogType logType, object message, UnityEngine.Object context)
    {
      LogFormat(logType, context, EscapeMessage(message));
    }

    public void Log(LogType logType, string tag, object message)
    {
      Log(logType, tag, message, null);
    }

    public void Log(LogType logType, string tag, object message, UnityEngine.Object context)
    {
      LogFormat(logType, context, TaggedLogHandler.ApplyTag(tag, EscapeMessage(message)));
    }

    public void Log(object message)
    {
      Log(LogType.Log, message);
    }

    public void Log(string tag, object message)
    {
      Log(LogType.Log, tag, message);
    }

    public void Log(string tag, object message, UnityEngine.Object context)
    {
      Log(LogType.Log, tag, message, context);
    }

    public void LogWarning(object message)
    {
      Log(LogType.Warning, message);
    }

    public void LogWarning(string tag, object message)
    {
      Log(LogType.Warning, tag, message);
    }

    public void LogWarning(string tag, object message, UnityEngine.Object context)
    {
      Log(LogType.Warning, tag, message, context);
    }

    public void LogError(object message)
    {
      Log(LogType.Error, message);
    }

    public void LogError(string tag, object message)
    {
      Log(LogType.Error, tag, message);
    }

    public void LogError(string tag, object message, UnityEngine.Object context)
    {
      Log(LogType.Error, tag, message, context);
    }

    public void LogException(Exception exception)
    {
      LogException(exception, null);
    }

    public void LogException(Exception exception, UnityEngine.Object context)
    {
      logHandler.LogException(exception, context);
    }

    public void LogFormat(LogType logType, string format, params object[] args)
    {
      LogFormat(logType, null, format, args);
    }

    public void LogFormat(LogType logType, UnityEngine.Object context, string format, params object[] args)
    {
      logHandler.LogFormat(logType, context, format, args);
    }

    public bool IsLogTypeAllowed(LogType logType)
    {
      return GetLogLevel(logType) <= GetLogLevel(filterLogType);
    }

    // ISerializationCallbackReceiver methods

    public void OnBeforeSerialize()
    {
    }

    public void OnAfterDeserialize()
    {
      // Keep `guid`/`logHandler` properties in sync with `tag` property after
      // updating any properties in the Unity inspector.
      OnTagChange();
    }

    // Private methods

    private int GetLogLevel(LogType logType)
    {
      switch (logType)
      {
        case LogType.Log: return 5;
        case LogType.Warning: return 4;
        case LogType.Assert: return 3;
        case LogType.Error: return 2;
        case LogType.Exception: return 1;
      }
      return 0;
    }

    private string EscapeMessage(object message)
    {
      return message.ToString().Replace("{", "{{").Replace("}", "}}");
    }

    private void OnTagChange()
    {
      _guid = WDPLogHandler.GenerateGuid(_tag).ToString();
      _logHandler = new TaggedLogHandler(
        new CompositeLogHandler(new List<ILogHandler>{
                new FilteredLogHandler(Debug.unityLogger.logHandler, this),
                new WDPLogHandler(_tag),
        }), _tag);
    }
  }
}
