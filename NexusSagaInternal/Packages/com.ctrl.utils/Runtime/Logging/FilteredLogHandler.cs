using System;

using UnityEngine;

namespace CTRL.Utils.Logging
{
  public class FilteredLogHandler : ILogHandler
  {
    private ILogHandler handler;
    private ILogger logger;

    public FilteredLogHandler(ILogHandler handler, ILogger logger)
    {
      this.handler = handler;
      this.logger = logger;
    }

    public void LogFormat(LogType logType, UnityEngine.Object context, string format, params object[] args)
    {
      if (!logger.logEnabled || !logger.IsLogTypeAllowed(logType))
      {
        return;
      }
      handler.LogFormat(logType, context, format, args);
    }

    public void LogException(Exception exception, UnityEngine.Object context)
    {
      if (!logger.logEnabled || !logger.IsLogTypeAllowed(LogType.Exception))
      {
        return;
      }
      handler.LogException(exception, context);
    }
  }
}
