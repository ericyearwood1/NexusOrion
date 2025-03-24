using System;

using UnityEngine;

namespace CTRL.Utils.Logging
{
  public class TaggedLogHandler : ILogHandler
  {
    private ILogHandler handler;
    private string tag;

    public TaggedLogHandler(ILogHandler handler, string tag)
    {
      this.handler = handler;
      this.tag = tag;
    }

    public void LogFormat(LogType logType, UnityEngine.Object context, string format, params object[] args)
    {
      handler.LogFormat(logType, context, ApplyTag(tag, format), args);
    }

    public void LogException(Exception exception, UnityEngine.Object context)
    {
      handler.LogException(exception, context);
    }

    public static string ApplyTag(string tag, string message)
    {
      return $"[{tag}] {message}";
    }
  }
}
