// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using CTRL;
using CTRL.Data;

namespace ARGlasses.Interaction
{
    public class RawImuStream_POCR_120 : OutputStreamHandle<IMU>
    {
        protected override string defaultStreamName => "RAW_IMU";
    }
}
