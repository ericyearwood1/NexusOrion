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
public class TestContext
{
    private IOVRPluginPlayModeTestEventQueue EventQueue { get; set; }
    internal SceneManagerTests SceneManagerTestFixture;

    public void Mock()
    {
        OVRPlugin.OVRP_1_1_0.mockObj = new MockOVRPlugin_1();
        OVRPlugin.OVRP_1_3_0.mockObj = new FakeOVRPlugin_3();
        OVRPlugin.OVRP_1_9_0.mockObj = new MockOVRPlugin9();
        OVRPlugin.OVRP_1_66_0.mockObj = new FakeOVRPlugin_66();
        OVRPlugin.OVRP_1_70_0.mockObj = new FakeOVRPlugin_70();
        var fake551 = new FakeOVRPlugin_55_1();
        EventQueue = fake551;
        OVRPlugin.OVRP_1_55_1.mockObj = fake551;

        OVRTelemetry.Mock(value: false);

        SceneManagerTestFixture = new SceneManagerTests();
        SceneManagerTestFixture.Mock();
    }

    public void Unmock()
    {
        OVRPlugin.OVRP_1_1_0.mockObj = new OVRPlugin.OVRP_1_1_0_TEST();
        OVRPlugin.OVRP_1_3_0.mockObj = new OVRPlugin.OVRP_1_3_0_TEST();
        OVRPlugin.OVRP_1_9_0.mockObj = new OVRPlugin.OVRP_1_9_0_TEST();
        OVRPlugin.OVRP_1_66_0.mockObj = new OVRPlugin.OVRP_1_66_0_TEST();
        OVRPlugin.OVRP_1_70_0.mockObj = new OVRPlugin.OVRP_1_70_0_TEST();
        OVRPlugin.OVRP_1_55_1.mockObj = new OVRPlugin.OVRP_1_55_1_TEST();
        EventQueue = null;

        OVRTelemetry.Unmock();

        SceneManagerTestFixture.Unmock();
        OVRTask<bool>.Clear();
    }

    public class MockOVRPlugin9 : OVRPlugin.OVRP_1_9_0_TEST
    {
        public override OVRPlugin.Controller ovrp_GetActiveController()
        {
            return (OVRPlugin.Controller) (OVRInput.Controller.LTouch | OVRInput.Controller.RTouch);
        }

        public override OVRPlugin.Controller ovrp_GetConnectedControllers()
        {
            return (OVRPlugin.Controller) (OVRInput.Controller.LTouch | OVRInput.Controller.RTouch);
        }
    }

    public class MockOVRPlugin_1 : OVRPlugin.OVRP_1_1_0_TEST
    {
        public override OVRPlugin.Bool ovrp_GetInitialized()
        {
            return OVRPlugin.Bool.True;
        }

        public override OVRPlugin.Bool ovrp_GetNodePresent(OVRPlugin.Node nodeId)
        {
            return nodeId == OVRPlugin.Node.EyeCenter ? OVRPlugin.Bool.True : OVRPlugin.Bool.False;
        }

        public override OVRPlugin.Bool ovrp_GetAppMonoscopic()
        {
            return OVRPlugin.Bool.False;
        }
    }
}

#endif //OVRPLUGIN_TESTING

