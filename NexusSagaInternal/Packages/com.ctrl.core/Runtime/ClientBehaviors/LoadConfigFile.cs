using UnityEngine;

using System;
using System.Threading.Tasks;

using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

using CTRL.Utils;
using CTRL.Utils.Logging;

namespace CTRL.ClientBehaviors
{
  [ExecuteInEditMode]
  [DefaultExecutionOrder(ExecutionOrder.LoadConfigFile)]
  public class LoadConfigFile : ITK.DependBehavior
  {
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    public CTRLClient client;

    [SerializeField]
    private CTRLLogger logger = new CTRLLogger("LoadConfigFile", LogType.Error);
    public CTRLLogger Logger => logger;

    protected virtual void OnEnable()
    {
      client.ConfigCallbacks.Add(LoadOnTryConnect);
    }

    protected virtual void OnDisable()
    {
      client.ConfigCallbacks.Remove(LoadOnTryConnect);
    }

    public bool Load()
    {
      logger.Log("Begin loading ctrl.json.");
      JToken content = ConfigFileHelper.GetConfigFileJson(logger);
      if (content != null)
      {
        string host = ConfigFileHelper.ParseString(content, "ctrlr_host", logger);
        int? port = ConfigFileHelper.ParseInt(content, "ctrlr_port", logger);
        if (port != null)
        {
          client.Port = (int) port;
        }
        if (host != null)
        {
          client.Host = host;
          logger.Log("Found host: " + host);
          return true;
        }
      }
      return false;

    }

    private Task<bool> LoadOnTryConnect()
    {
      return Task.FromResult(Load());
    }
  }
}
