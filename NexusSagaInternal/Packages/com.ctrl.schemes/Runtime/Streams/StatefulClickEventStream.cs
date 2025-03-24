using System.Collections.Generic;

namespace CTRL.Data
{
  /// A discrete event detected by CTRL-R, sent on tap or double taps.
#pragma warning disable IDE1006
  [System.Serializable]
  public struct StatefulClickPayload
  {
    /// One of: `press`, `release`, `tap`, `double-tap`
    public string action;
    /// One of: `index`, `middle`
    public string finger;
    /// Contains finger -> state mappings, where state is 'holding' or 'released'
    public Dictionary<string, string> state;
  }
#pragma warning restore IDE1006

  public class StatefulClickEventStream : OutputStreamHandle<CTRLEvent<StatefulClickPayload>>
  {
    protected override string defaultStreamName => "stateful_click_api";
  }
}
