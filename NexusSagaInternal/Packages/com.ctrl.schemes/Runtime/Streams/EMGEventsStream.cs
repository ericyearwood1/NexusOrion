using System;
using Newtonsoft.Json.Linq;

namespace CTRL.Data
{
  [Serializable]
  public struct EMGEventPayload
  {
    public string name;
    public string type;
    public string timing;
    public JToken payload;
  }

  public class EMGEventsStream : OutputStreamHandle<EMGEventPayload>
  {
    protected override string defaultStreamName => "emg_events";
  }
}
