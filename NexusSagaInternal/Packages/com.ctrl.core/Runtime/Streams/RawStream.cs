using Newtonsoft.Json.Linq;

namespace CTRL.Data
{
  /// Mainly useful for debugging or using streams without
  /// automatically deserialising them. Set the `streamName` in the
  /// inspector of this component before using it.
  public class RawStream : OutputStreamHandle<JToken> { }
}
