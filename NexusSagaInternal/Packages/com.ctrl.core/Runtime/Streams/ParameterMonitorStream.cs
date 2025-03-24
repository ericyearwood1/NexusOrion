using Newtonsoft.Json.Linq;
namespace CTRL.Data
{
  public class ParameterMonitorStream : OutputStreamHandle<JObject>
  {
    protected override string defaultStreamName => "parameter_monitor";
  }
}
