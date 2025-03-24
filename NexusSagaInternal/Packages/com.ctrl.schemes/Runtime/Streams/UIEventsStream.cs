using System;
using Newtonsoft.Json.Linq;

namespace CTRL.Data
{
  [Serializable]
  public struct UIEventPayload
  {
    public string name;
    public string timing;
    public JToken payload;
    public string type;
  }

  public class UIEventsStream : OutputStreamHandle<UIEventPayload>
  {
    protected override string defaultStreamName => "ui_events";
  }
}
