using UnityEngine;
using UnityEngine.Events;

namespace CTRL
{
  /**
    This component initialises the latency stream on CTRLClient.
    */
  public class StartLatencyStream : ITK.DependBehavior
  {
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected CTRLClient client;

    void OnEnable()
    {
      client.OnConnect.AddListener(StartStream);
    }

    void OnDisable()
    {
      client.OnConnect.RemoveListener(StartStream);
    }

    void StartStream()
    {
      client.SendStartLatencyStream();
    }
  }
}
