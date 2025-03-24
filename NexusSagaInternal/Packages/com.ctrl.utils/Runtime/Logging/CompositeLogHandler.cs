using System;
using System.Collections.Generic;

using UnityEngine;

namespace CTRL.Utils.Logging
{
  public class CompositeLogHandler : ILogHandler
  {
    private IEnumerable<ILogHandler> handlers;

    public CompositeLogHandler(IEnumerable<ILogHandler> handlers)
    {
      this.handlers = handlers;
    }

    public void LogFormat(LogType logType, UnityEngine.Object context, string format, params object[] args)
    {
      foreach (var handler in handlers)
      {
        handler.LogFormat(logType, context, format, args);
      }
    }

    public void LogException(Exception exception, UnityEngine.Object context)
    {
      foreach (var handler in handlers)
      {
        handler.LogException(exception, context);
      }
    }
  }
}
