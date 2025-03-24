using CTRL;
using CTRL.Data;

namespace ARGlasses.Interaction
{
    public class RawImuStream_Nostrand : OutputStreamHandle<IMU>
    {
        protected override string defaultStreamName => "raw_imu";
    }
}
