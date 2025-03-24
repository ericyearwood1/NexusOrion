namespace CTRL.Data
{

  [System.Serializable]
  public struct PipelineHealthData
  {
    public float path_latency;
    public float imu_data_gap;
    public float emg_data_gap;
    public float ctrlr2sdk;
    public float produce2tcp;
    public string status;
  }

  /// Should receive ~10 messages per second
  public class PipelineHealthStream : OutputStreamHandle<PipelineHealthData>
  {
    protected override string defaultStreamName => "health_checker.main";
  }
}
