using UnityEngine;

using CTRL.Data;
using CTRL.Utils.Logging;

namespace CTRL.Samples
{
  public class EMGLogger : ITK.DependBehavior
  {
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected EMGStream events;

    [SerializeField]
    private CTRLLogger logger = new CTRLLogger("EMGLogger");
    public CTRLLogger Logger => logger;

    protected void OnEnable()
    {
      events.OnStreamBatch.AddListener(OnBatch);
    }

    protected void OnDisable()
    {
      events.OnStreamBatch.RemoveListener(OnBatch);
    }

    protected void OnBatch(StreamBatch<float[]> batch)
    {
      logger.Log($"Raw EMG: [{string.Join(", ", batch.Latest().data)}]");
    }
  }
}
