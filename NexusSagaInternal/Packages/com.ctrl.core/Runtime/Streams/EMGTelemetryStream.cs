namespace CTRL.Data
{

  [System.Serializable]
  public struct EMGTelemetryData
  {
    public string message;
    public int timestamp;
    public string type;
    public string unixname;
    public string study_id;
  }

  /// A continuous stream of the EMG telemetry data from ctrl-r
  /// Outputs a performance snapshot every 10s and stateful clicks every 30s
  public class EMGTelemetryStream : OutputStreamHandle<EMGTelemetryData[]>
  {
    protected override string defaultStreamName => "eventhub_delegator.main";
  }
}
