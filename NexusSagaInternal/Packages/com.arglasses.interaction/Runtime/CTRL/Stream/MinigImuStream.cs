// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using CTRL;
using CTRL.Data;

namespace ARGlasses.Interaction
{
    public class MinigImuStream : OutputStreamHandle<IMU>
    {
        protected override string defaultStreamName => "minig.hand_imu";
    }
}
