using System;
using Newtonsoft.Json.Linq;

namespace CTRL.Data
{
  [Serializable]
  public struct ContinuousUIEventPayload
  {
    public string name;
    public JToken payload;
    public float[] axes;

  }

  public class ContinuousUIEventsStream : OutputStreamHandle<ContinuousUIEventPayload>
  {
    protected override string defaultStreamName => "continuous_ui_events";
  }
}
