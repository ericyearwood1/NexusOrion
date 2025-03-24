using UnityEngine;

using CTRL.Data;
using CTRL.Utils.Logging;

namespace CTRL.Samples
{
  public class EMGTelemetryLogger : ITK.DependBehavior
  {
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected EMGTelemetryStream events;

    [SerializeField]
    private CTRLLogger logger = new CTRLLogger("EMGTelemetryLogger", LogType.Log);

    protected void OnEnable()
    {
      events.OnStreamBatch.AddListener(OnBatch);
    }

    protected void OnDisable()
    {
      events.OnStreamBatch.RemoveListener(OnBatch);
    }

    protected void OnBatch(StreamBatch<EMGTelemetryData[]> batch)
    {
      foreach (var sample in batch.samples)
      {
        foreach (var packet in sample.data)
        {
          logger.Log($"type: {packet.type}, timestamp: {packet.timestamp}, message: {packet.message}, unixname: {packet.unixname}, study-id: {packet.study_id}");
        }
      }
    }
  }
}
