using CTRL;
using CTRL.Data;

namespace ARGlasses.Interaction
{
    public class RawImuStream_POCR_60 : OutputStreamHandle<IMU>
    {
        protected override string defaultStreamName => "RAW_IMU_HALF";
    }
}
