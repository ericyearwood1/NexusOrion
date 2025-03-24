using CTRL;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class RawAccelerationStream_POCR_120 : OutputStreamHandle<Vector3>
    {
        protected override string defaultStreamName => "RAW_ACCEL";
    }
}
