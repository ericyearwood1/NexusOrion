using System;
using System.Security.Cryptography;
using System.Text;

using UnityEngine;

#if WINDOWS_UWP
using Windows.Foundation.Diagnostics;
#endif

namespace CTRL.Utils.Logging
{
  /// <summary>
  /// Wraps `Windows.Foundation.Diagnostics.LoggingChannel` to provide realtime
  /// logging in Windows Device Portal (WDP).
  /// </summary>
  /// <remarks>
  /// To enable realtime logging in Windows Device Portal:
  ///
  /// 1. Navigate to <a
  /// href="http://&lt;hololens-host&gt;/#Logging">http://&lt;hololens-host&gt;/#Logging</a>
  ///
  /// 2. Paste the value of the `Guid` property into "Custom providers" field,
  /// then click "Enable".
  ///
  /// 3. Watch for log messages in "Events" section.
  /// </remarks>
  public class WDPLogHandler : ILogHandler
  {
    private static readonly MD5 md5 = MD5.Create();

    public readonly Guid Guid;

    public static Guid GenerateGuid(string tag)
    {
      // To compute this value on the command line (using Python):
      // $ python -c "import sys, hashlib, uuid; print(uuid.UUID(bytes_le=hashlib.md5(str.encode(sys.argv[1])).digest()))" <tag>
      return new Guid(md5.ComputeHash(Encoding.Default.GetBytes(tag)));
    }

    public WDPLogHandler(string tag, Guid? guid = null)
    {
      this.Guid = guid != null ? (Guid)guid : GenerateGuid(tag);
      CreateLoggingChannel(tag, this.Guid);
    }

#if WINDOWS_UWP
    private LoggingChannel channel;

    private void CreateLoggingChannel(string tag, Guid guid)
    {
      channel = new LoggingChannel(tag, null, guid);
    }

    public void LogFormat(LogType logType, UnityEngine.Object context, string format, params object[] args)
    {
      channel.LogMessage(string.Format(format, args), GetLoggingLevel(logType));
    }

    public void LogException(Exception exception, UnityEngine.Object context)
    {
      channel.LogMessage(exception.ToString(), GetLoggingLevel(LogType.Exception));
    }

    private static LoggingLevel GetLoggingLevel(LogType logType)
    {
      switch (logType)
      {
        case LogType.Log:
          return LoggingLevel.Information;
        case LogType.Warning:
          return LoggingLevel.Warning;
        case LogType.Error:
          return LoggingLevel.Error;
        case LogType.Assert:
        case LogType.Exception:
          return LoggingLevel.Critical;
      }
      throw new Exception("unrecognized log type");
    }
#else
    private void CreateLoggingChannel(string tag, Guid guid)
    {
      // No-op on non-Windows platforms
    }

    public void LogFormat(LogType logType, UnityEngine.Object context, string format, params object[] args)
    {
      // No-op on non-Windows platforms
    }

    public void LogException(Exception exception, UnityEngine.Object context)
    {
      // No-op on non-Windows platforms
    }
#endif
  }
}
