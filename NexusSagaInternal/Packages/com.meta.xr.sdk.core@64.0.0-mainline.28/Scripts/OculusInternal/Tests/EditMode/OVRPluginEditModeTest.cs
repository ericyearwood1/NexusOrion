/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * Licensed under the Oculus SDK License Agreement (the "License");
 * you may not use the Oculus SDK except in compliance with the License,
 * which is provided at the time of installation or download, or which
 * otherwise accompanies this software in either electronic or hard copy form.
 *
 * You may obtain a copy of the License at
 *
 * https://developer.oculus.com/licenses/oculussdk/
 *
 * Unless required by applicable law or agreed to in writing, the Oculus SDK
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if OVRPLUGIN_TESTING

using System.Collections;
using Bool = OVRPlugin.Bool;
using Result = OVRPlugin.Result;

// Base class for any Unity test for OVRPlugin that needs to set OVRPlugin.initialized to true
// (generally recommended). Stubs out some things that break in this scenario.
public class OVRPluginEditModeTest
{
    protected virtual bool TelemetryMockedOut => true;

    protected const int DefaultTimeoutMs = 10000;

    public virtual IEnumerator UnitySetUp()
    {
        // May not be cleaned up if previous test timed out, failed during setup, etc.
        OVRPlugin.OVRP_1_1_0.mockObj = new FakeOVRPlugin_1();
        OVRPlugin.OVRP_1_3_0.mockObj = new FakeOVRPlugin_3();
        OVRPlugin.OVRP_1_66_0.mockObj = new FakeOVRPlugin_66();
        OVRPlugin.OVRP_1_70_0.mockObj = new FakeOVRPlugin_70();

        if (TelemetryMockedOut)
        {
            OVRTelemetry.Mock(value: false);
        }

        yield return null;
    }

    public virtual IEnumerator UnityTearDown()
    {
        // Point mockObjs back at original production version of code
        OVRPlugin.OVRP_1_1_0.mockObj = new OVRPlugin.OVRP_1_1_0_TEST();
        OVRPlugin.OVRP_1_3_0.mockObj = new OVRPlugin.OVRP_1_3_0_TEST();
        OVRPlugin.OVRP_1_66_0.mockObj = new OVRPlugin.OVRP_1_66_0_TEST();
        OVRPlugin.OVRP_1_70_0.mockObj = new OVRPlugin.OVRP_1_70_0_TEST();

        if (TelemetryMockedOut)
        {
            OVRTelemetry.Unmock();
        }

        yield return null;
    }

    public class FakeOVRPlugin_1 : OVRPlugin.OVRP_1_1_0_TEST
    {
        // Force OVRPlugin.initialized to true, even though it cannot actually initialize
        // successfully in the test environment. This is mainly required for OVRCameraRig.UpdateAnchors()
        // to update positions of the hand objects (it checks OVRManager.OVRManagerinitialized, and
        // OVRManager checks OVRPlugin.initialized before calling InitOVRManager()).
        public override Bool ovrp_GetInitialized()
        {
            return Bool.True;
        }
    }

    public class FakeOVRPlugin_3 : OVRPlugin.OVRP_1_3_0_TEST
    {
        // If this is called on an uninitialized OVRPlugin, Unity will crash, stub it out
        public override Bool ovrp_SetEyeOcclusionMeshEnabled(Bool value)
        {
            return Bool.True;
        }
    }

    public class FakeOVRPlugin_66 : OVRPlugin.OVRP_1_66_0_TEST
    {
        // Force this to succeed, otherwise the code will produce an error log and fail the test
        public override Result ovrp_GetInsightPassthroughInitializationState()
        {
            return Result.Success;
        }
    }

    public class FakeOVRPlugin_70 : OVRPlugin.OVRP_1_70_0_TEST
    {
        // This prevents a crash when tests are run twice and OVRPlugin is uninitialized
        public override Result ovrp_SetLogCallback2(OVRPlugin.LogCallback2DelegateType logCallback)
        {
            return Result.Success;
        }
    }
}

#endif
