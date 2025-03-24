using System.Collections.Generic;
using UnityEngine;

using Newtonsoft.Json.Linq;

using CTRL.Data;

namespace CTRL.ClientBehaviors
{
  [System.Serializable]
  public struct TransformParameterDef
  {
    public string Transform;
    public string Parameter;

    [TextArea]
    public string Value;
  }

  public class AutoApplyParameter : ITK.DependBehavior
  {
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected CTRLClient client;

    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected ParameterMonitorStream stream;
    public ParameterMonitorStream Stream { get => stream; }

    [SerializeField]
    protected float backoffTime = 0.5f;

    [SerializeField]
    protected List<TransformParameterDef> parameters = new List<TransformParameterDef>();

    bool isSending = false;
    private float lastSendTime = -1f;

    protected void OnEnable()
    {
      stream.OnStreamLatestSample.AddListener(OnParameterSample);
    }

    protected void OnDisable()
    {
      stream.OnStreamLatestSample.RemoveListener(OnParameterSample);
    }

    protected void OnParameterSample(Sample<JObject> sample)
    {
      if (lastSendTime > 0 && Time.time - lastSendTime < backoffTime)
      {
        return;
      }

      bool hasChange = false;
      foreach (var param in parameters)
      {
        // Skip empty data
        if (param.Transform == "" || param.Parameter == "" || param.Value == "")
        {
          continue;
        }

        var remoteParam = sample.data[param.Transform]?[param.Parameter];
        if (remoteParam == null)
        {
          Debug.LogWarning($"Skipping update of parameter {param.Transform}:{param.Parameter}, which isn't known to CTRL-R.");
          continue;
        }

        var type = remoteParam.Value<string>("type");
        if (type == "object")
        {
          // Object types force us to parse the input and compare its nested value
          try
          {
            // This section is attempting to use parsing to determine if the object is actually valid.
            // Because Newtonsoft JSON is more permissive than CTRL-R, this can still let invalid JSON through.
            var currValObj = JObject.Parse(param.Value);
            if (currValObj.Type != JTokenType.Object)
            {
              Debug.LogWarning($"Skipping update to object parameter {param.Transform}:{param.Parameter} with a non-object type.");
              continue;
            }

            var currValStr = currValObj.ToString();
            var remoteValStr = remoteParam["value"].ToString();
            if (currValStr != remoteValStr)
            {
              hasChange = true;
              break;
            }
          }
          catch
          {
            Debug.LogWarning($"Parse error while updating {param.Transform}:{param.Parameter}");
            continue;
          }
        }
        else
        {
          // Non-object types are okay to just compare via string
          var currValue = remoteParam["value"].ToObject<string>();
          if (currValue != param.Value)
          {
            hasChange = true;
            break;
          }
        }
      }

      if (hasChange)
      {
        lastSendTime = Time.time;
        ApplyParameters();
      }
    }

    protected void ApplyParameters()
    {
      if (isSending)
      {
        return;
      }

      if (client.State != ConnectionState.Connected)
      {
        return;
      }

      isSending = true;

      var request = new JObject {
        {"change_parameter_request", new JObject {
          {"transforms", new JObject()}
        }}
      };

      var transformsObj = request["change_parameter_request"]["transforms"];
      foreach (var param in parameters)
      {
        if (transformsObj[param.Transform] == null)
        {
          transformsObj[param.Transform] = new JObject {
            {"parameters", new JObject {}}
          };
        }

        transformsObj[param.Transform]["parameters"][param.Parameter] = param.Value;
      }

      try
      {
        // TODO: can we await this somehow?
        client.SendRequest(request);
      }
      finally
      {
        isSending = false;
      }
    }
  }
}
