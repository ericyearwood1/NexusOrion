using CTRL;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class RawAccelerationStream_POCR_60 : OutputStreamHandle<Vector3>
    {
        // caps?
        protected override string defaultStreamName => "RAW_ACCEL_HALF";
    }
}
