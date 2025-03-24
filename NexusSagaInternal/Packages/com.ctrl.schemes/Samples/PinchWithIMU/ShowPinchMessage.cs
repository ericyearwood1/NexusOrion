using UnityEngine;
using UnityEngine.UI;

using CTRL.Data;
using Newtonsoft.Json;

namespace CTRL.Schemes.Samples
{
  public class ShowPinchMessage : ITK.DependBehavior
  {
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected StatefulClickEventStream stream;

    [SerializeField]
    protected Text target;

    protected void Update()
    {
      if (stream.Latest.HasValue)
      {
        target.text = JsonConvert.SerializeObject(stream.Latest.Value, Formatting.Indented);
      }
    }
  }
}
