using System.Collections.Generic;

namespace CTRL.Data
{
  /// A discrete event detected by CTRL-R.
  /// Documentation: https://github.com/ctrl-labs/RFCs/blob/master/0031-new-event-api.md
#pragma warning disable IDE1006
  [System.Serializable]
  public struct CTRLEvent<T>
  {
    /// Human readable event name, e.g. 'pinch_index_press', 'thumb_swipe_north'
    public string name;
    /// One of: `start`, `end`, or `instant`
    public string timing;
    /// Name of control scheme, describes how payload should be parsed
    public string type;

    /// Payload
    public T payload;
  }
#pragma warning restore IDE1006
}
