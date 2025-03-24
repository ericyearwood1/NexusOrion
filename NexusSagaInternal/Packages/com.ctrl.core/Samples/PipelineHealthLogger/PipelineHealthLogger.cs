using UnityEngine;

using CTRL.Data;
using CTRL.Utils.Logging;

namespace CTRL.Samples
{
  public class PipelineHealthLogger : ITK.DependBehavior
  {
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected PipelineHealthStream events;

    [SerializeField]
    private CTRLLogger logger = new CTRLLogger("PipelineHealthLogger");

    protected void OnEnable()
    {
      events.OnStreamBatch.AddListener(OnBatch);
    }

    protected void OnDisable()
    {
      events.OnStreamBatch.RemoveListener(OnBatch);
    }

    protected void OnBatch(StreamBatch<PipelineHealthData> batch)
    {
      foreach (var packet in batch.samples)
      {
        logger.Log(
          $"status: {packet.data.status}, " +
          $"path_latency: {packet.data.path_latency}, " +
          $"imu_data_gap: {packet.data.imu_data_gap}, " +
          $"emg_data_gap: {packet.data.emg_data_gap}, " +
          $"ctrlr2sdk: {packet.data.ctrlr2sdk}, " +
          $"produce2tcp: {packet.data.produce2tcp}, "
        );
      }
    }
  }
}
