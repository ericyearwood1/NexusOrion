
using UnityEngine;

namespace CTRL.ClientBehaviors
{
  public class SetPipeline : ITK.DependBehavior
  {
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected CTRLClient client;

    [SerializeField]
    protected bool sendOnConnect = true;

    [SerializeField]
    protected TextAsset config;
    public TextAsset Config { get => config; }

    protected virtual void OnEnable()
    {
      client.OnConnect.AddListener(OnClientConnect);
    }

    protected virtual void OnDisable()
    {
      client.OnConnect.RemoveListener(OnClientConnect);
    }

    protected void OnClientConnect()
    {
      if (sendOnConnect)
      {
        SendConfig();
      }
    }

    public void SendConfig()
    {
      client.SendSetGraphRequest(config.text);
    }
  }
}
