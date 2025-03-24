using CTRL;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class RawAccelerationStream_Nostrand : OutputStreamHandle<float[]>
    {
        protected override string defaultStreamName => "IMU_ACCEL";
    }
}
