using UnityEngine;
using UnityEngine.UI;

using System.Text;
using System.Linq;

namespace CTRL.DebugUI
{
  using CTRL;

  public class ClientDebugText : ITK.DependBehavior
  {
    [SerializeField]
    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    protected CTRLClient client;

    [SerializeField]
    protected Text textTarget;

    protected StringBuilder sb = new StringBuilder(1024);

    public bool ShowStreamList = false;

    readonly string RED = "#db4325";
    readonly string YELLOW = "#ffc107";
    readonly string GREEN = "#428f87";

    protected void OnValidate()
    {
      if (textTarget == null)
      {
        textTarget = GetComponentInChildren<Text>();
      }
    }

    protected void Update()
    {
      if (textTarget == null)
      {
        return;
      }

      sb.Clear();

      bool isConnected = client.State == ConnectionState.Connected;

      // Connection Status
      sb.Append("<size=22>");
      if (isConnected)
      {
        sb.Append($"<color={GREEN}></color>");
      }
      else
      {
        sb.Append("");
      }
      sb.Append(" CTRL-R");
      sb.Append("</size>");
      sb.AppendLine();
      sb.Append($"<size=11>{client.Host}:{client.Port}</size>");
      sb.AppendLine();

      // Signal Quality
      PliStream pliStream = client.GetComponent<PliStream>();
      if (pliStream && pliStream.Latest.HasValue)
      {
        sb.Append(getPliIndicator(pliStream));
        sb.AppendLine();
      }

      BatteryStream batteryStream = client.GetComponent<BatteryStream>();
      if (batteryStream && batteryStream.Latest.HasValue)
      {
        sb.Append(getBatteryIndicator(batteryStream));
        sb.AppendLine();
      }

      ContactLossStream contactLossStream = client.GetComponent<ContactLossStream>();
      if (contactLossStream && contactLossStream.Latest.HasValue)
      {
        sb.Append(getContactLossIndicator(contactLossStream));
        sb.AppendLine();
      }

      DeviceMonitorStream deviceMonitorStream = client.GetComponent<DeviceMonitorStream>();
      if (deviceMonitorStream && deviceMonitorStream.Latest.HasValue)
      {
        sb.Append(getDeviceMonitorIndicator(deviceMonitorStream));
        sb.AppendLine();
      }

      if (ShowStreamList && client.Streams.Count() > 0)
      {
        sb.AppendLine();
        sb.Append("<size=18>Streams</size>");
        foreach (var stream in client.Streams)
        {
          sb.AppendLine();
          sb.Append(stream.State == StreamState.Connected ? $"<color={GREEN}></color> " : " ");
          sb.Append(stream.StreamName);
        }
      }

      textTarget.text = sb.ToString();
    }

    private string getPliIndicator(PliStream pliStream)
    {
      int pliCount = pliStream.Latest.Value.data.count[0];
      float percent = Mathf.Min(pliCount * 100, 100);
      string color;

      if (percent > 66)
      {
        color = RED;
      }
      else if (percent > 33)
      {
        color = YELLOW;
      }
      else
      {
        color = GREEN;
      }
      return $"<color={color}></color> Interference";
    }

    private string getBatteryIndicator(BatteryStream batteryStream)
    {
      float percent = batteryStream.Latest.Value.data.percent;
      string color;

      // empty
      string icon = "";
      if (percent > 80)
      {
        // full
        icon = "";
        color = GREEN;
      }
      else if (percent > 60)
      {
        // three quarters
        icon = "";
        color = GREEN;
      }
      else if (percent > 40)
      {
        // half
        icon = "";
        color = GREEN;
      }
      else if (percent > 20)
      {
        // quarter
        icon = "";
        color = YELLOW;
      }
      else
      {
        color = RED;
      }
      return $"<color={color}>{icon}</color> Battery";
    }

    private string getContactLossIndicator(ContactLossStream contactLossStream)
    {
      ContactLossState state = contactLossStream.Latest.Value.data.state;
      string color = GREEN;
      if (state == ContactLossState.ContactLoss)
      {
        color = RED;
      }
      if (state == ContactLossState.Nominal)
      {
        color = GREEN;
      }


      return $"<color={color}></color> Contact Loss";
    }

    private string getDeviceMonitorIndicator(DeviceMonitorStream deviceMonitorStream)
    {
      float timeSinceData = deviceMonitorStream.Latest.Value.data.time_since_data;
      string color;
      if (timeSinceData > 3)
      {
        color = RED;
      }
      else
      {
        color = GREEN;
      }

      return $"<color={color}></color> Data Loss";
    }
  }
}
