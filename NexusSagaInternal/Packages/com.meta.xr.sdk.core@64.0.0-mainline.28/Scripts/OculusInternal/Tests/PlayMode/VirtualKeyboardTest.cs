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
#if OVRPLUGIN_TESTING_XR_INPUT_TEXT_ENTRY
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.RegularExpressions;
using NUnit.Framework;
using OVR.OpenVR;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.TestTools;
using UnityEngine.TestTools.Utils;
using UnityEngine.UI;
using Button = UnityEngine.UIElements.Button;
using Object = UnityEngine.Object;
using Result = OVRPlugin.Result;
using Quatf = OVRPlugin.Quatf;
using Vector3f = OVRPlugin.Vector3f;

[Category("OnCall:xrinput_text_entry")]
public class VirtualKeyboardTest : OVRPluginPlayModeTest
{
    private class FakeOVRPlugin0 : OVRPlugin.OVRP_1_0_0_TEST
    {
        private OVRPlugin.TrackingOrigin _originType = OVRPlugin.TrackingOrigin.EyeLevel;

        public override OVRPlugin.Bool ovrp_SetTrackingOriginType(OVRPlugin.TrackingOrigin originType)
        {
            _originType = originType;
            base.ovrp_SetTrackingOriginType(originType);
            return OVRPlugin.Bool.True;
        }

        public override OVRPlugin.TrackingOrigin ovrp_GetTrackingOriginType()
        {
            return _originType;
        }
    }

    private class FakeOVRPlugin38 : OVRPlugin.OVRP_1_38_0_TEST
    {
        public delegate Result GetNodePositionValid(OVRPlugin.Node nodeId, ref OVRPlugin.Bool nodePositionValid);
        public GetNodePositionValid OnGetNodePositionValid;
        public override Result ovrp_GetNodePositionValid(OVRPlugin.Node nodeId, ref OVRPlugin.Bool nodePositionValid)
        {
            return OnGetNodePositionValid?.Invoke(nodeId, ref nodePositionValid) ?? Result.Failure;
        }
    }
    private class FakeOVRPlugin64 : OVRPlugin.OVRP_1_64_0_TEST
    {
        public static OVRPlugin.VirtualKeyboardLocationType CurrentSpaceType;
        private readonly Func<OVRPlugin.Posef> locateCustomSpace_;

        public FakeOVRPlugin64(Func<OVRPlugin.Posef> locateCustomSpace)
        {
            locateCustomSpace_ = locateCustomSpace;
        }

        public override Result ovrp_LocateSpace(ref OVRPlugin.Posef location, ref ulong space,
            OVRPlugin.TrackingOrigin trackingOrigin)
        {
            switch (CurrentSpaceType)
            {
                case OVRPlugin.VirtualKeyboardLocationType.Direct:
                    location = VirtualKeyboardTest.NearDefaultPose;
                    break;
                case OVRPlugin.VirtualKeyboardLocationType.Far:
                    location = VirtualKeyboardTest.FarDefaultPose;
                    break;
                case OVRPlugin.VirtualKeyboardLocationType.Custom:
                    location = locateCustomSpace_.Invoke();
                    break;
            }

            return OVRPlugin.Result.Success;
        }
    }

    private class FakeOVRPlugin68 : OVRPlugin.OVRP_1_68_0_TEST
    {

        public Result GetRenderModelPathsDefaultReturnValue = Result.Success;
        public override Result ovrp_GetRenderModelPaths(uint index, IntPtr path)
        {
            if (index == 0)
            {
                var pathData = Encoding.ASCII.GetBytes("/model_meta/keyboard/virtual\0");
                Marshal.Copy(pathData, 0, path, pathData.Length);
                return GetRenderModelPathsDefaultReturnValue;
            }

            return OVRPlugin.Result.Failure;
        }

        public override Result ovrp_LoadRenderModel(ulong modelKey, uint bufferInputCapacity, ref uint bufferCountOutput,
            IntPtr buffer)
        {
            if (modelKey != 123)
            {
                throw new InvalidOperationException("Unexpected modelKey");
            }

            var glbPath = "Assets/Oculus/VR/Scripts/OculusInternal/Tests/PlayMode/Resources/virtual_keyboard.glb";

            var glbFileLength = (uint)new FileInfo(glbPath).Length;
            bufferCountOutput = glbFileLength;

            if (bufferInputCapacity == 0)
            {
                return OVRPlugin.Result.Success;
            }

            if (bufferInputCapacity != glbFileLength)
            {
                throw new InvalidOperationException("Unexpected bufferInputCapacity");
            }

            Marshal.Copy(File.ReadAllBytes(glbPath), 0, buffer, (int)glbFileLength);

            return OVRPlugin.Result.Success;
        }
    }

    private class FakeOVRPlugin74 : OVRPlugin.OVRP_1_74_0_TEST
    {
        public static Vector3f TestPosition = new Vector3f() { x = 0.2f, y = 0.3f, z = 0.4f };

        public static Quatf TestRotation = new Quatf()
            { x = 0.00439511752f, y = 0.00520915771f, z = 0.00608565286f, w = 0.999958336f };

        public uint TestShapeCount = 0;
        public int StopDestroyCount = 0;
        public int SuggestVirtualKeyboardLocationCallCount = 0;

        public OVRPlugin.VirtualKeyboardLocationInfo TestLocationInfo;
        public Result CreateVirtualKeyboardDefaultReturnValue;
        public string LastReceivedTextContext { get; private set; } = string.Empty;
        public List<OVRPlugin.VirtualKeyboardInputInfo> SendInputInfoList = new List<OVRPlugin.VirtualKeyboardInputInfo>();
        private int _currentFrameCount;

        public override Result ovrp_GetRenderModelProperties2(string path, OVRPlugin.RenderModelFlags flags,
            out OVRPlugin.RenderModelPropertiesInternal properties)
        {
            properties = default;
            if (path != "/model_meta/keyboard/virtual")
            {
                throw new InvalidOperationException("Unexpected path");
            }

            properties.ModelKey = 123;
            properties.ModelName = Encoding.ASCII.GetBytes("Virtual Keyboard\0");
            return OVRPlugin.Result.Success;
        }

        public override Result ovrp_CreateVirtualKeyboard(OVRPlugin.VirtualKeyboardCreateInfo createInfo)
        {
            return CreateVirtualKeyboardDefaultReturnValue;
        }

        public override Result ovrp_CreateVirtualKeyboardSpace(OVRPlugin.VirtualKeyboardSpaceCreateInfo createInfo,
            out ulong keyboardSpace)
        {
            keyboardSpace = (ulong)createInfo.locationType;
            keyboardSpace++;
            FakeOVRPlugin64.CurrentSpaceType = createInfo.locationType;
            return OVRPlugin.Result.Success;
        }

        public override Result ovrp_SuggestVirtualKeyboardLocation(OVRPlugin.VirtualKeyboardLocationInfo locationInfo)
        {
            TestLocationInfo = locationInfo;

            FakeOVRPlugin64.CurrentSpaceType = locationInfo.locationType;
            SuggestVirtualKeyboardLocationCallCount++;
            return OVRPlugin.Result.Success;
        }

        public override Result ovrp_ChangeVirtualKeyboardTextContext(string textContext)
        {
            LastReceivedTextContext = textContext;
            return OVRPlugin.Result.Success;
        }

        public override Result ovrp_GetVirtualKeyboardScale(out float scale)
        {
            if (TestLocationInfo.locationType != OVRPlugin.VirtualKeyboardLocationType.Custom)
            {
                scale = 1;
            }
            else
            {
                scale = TestLocationInfo.scale;
            }
            return OVRPlugin.Result.Success;
        }

        public override Result ovrp_DestroyVirtualKeyboard()
        {
            StopDestroyCount++;
            return OVRPlugin.Result.Success;
        }

        public override Result ovrp_SendVirtualKeyboardInput(OVRPlugin.VirtualKeyboardInputInfo inputInfo, ref OVRPlugin.Posef interactorRootPose)
        {
            // Reset if the frame advances
            if (Time.frameCount != _currentFrameCount)
            {
                _currentFrameCount = Time.frameCount;
                SendInputInfoList.Clear();
            }
            SendInputInfoList.Add(inputInfo);
            return Result.Success;
        }
    }

    private class FakeOVRPlugin83 : OVRPlugin.OVRP_1_83_0_TEST
    {
        public OVRPlugin.VirtualKeyboardModelVisibility Visibility;

        public override Result ovrp_GetVirtualKeyboardDirtyTextures(
            ref OVRPlugin.VirtualKeyboardTextureIdsInternal textureIds)
        {
            textureIds.TextureIdCountOutput = 1;
            if (textureIds.TextureIdCapacityInput == 0)
            {
                return Result.Success;
            }

            if (textureIds.TextureIdCapacityInput >= textureIds.TextureIdCountOutput)
            {
                Marshal.WriteInt64(textureIds.TextureIdsBuffer, 8901880085579532826);
                return Result.Success;
            }

            return Result.Failure;
        }

        public override Result ovrp_GetVirtualKeyboardTextureData(ulong textureId,
            ref OVRPlugin.VirtualKeyboardTextureData textureData)
        {
            if (textureId != 8901880085579532826)
            {
                return Result.Failure;
            }

            textureData.TextureWidth = 1536;
            textureData.TextureHeight = 1536;

            var texPath =
                "Assets/Oculus/VR/Scripts/OculusInternal/Tests/PlayMode/Resources/virtual_keyboard_glyph_texture_data.bytes";

            var texFileLength = (uint)new FileInfo(texPath).Length;

            textureData.BufferCountOutput = texFileLength;

            if (textureData.BufferCapacityInput == 0)
            {
                return Result.Success;
            }

            if (textureData.BufferCapacityInput >= textureData.BufferCountOutput)
            {
                Marshal.Copy(File.ReadAllBytes(texPath), 0, textureData.Buffer, (int)texFileLength);
                return Result.Success;
            }

            return Result.Failure;
        }

        public override Result ovrp_GetVirtualKeyboardModelAnimationStates(
            ref OVRPlugin.VirtualKeyboardModelAnimationStatesInternal animationStates)
        {
            var stateData = new[]
            {
                (0, 1.0f), // root bg
                (6, 0.0f), // hide layout
                (7, 0.0f), // hide layout
                (8, 0.4943257f), // next 8, top left gyyph vertice morph value
                (9, 0.5121053f),
                (10, 0.5056744f),
                (11, 0.5121053f),
                (12, 0.4943257f),
                (13, 0.4878947f),
                (14, 0.5056744f),
                (15, 0.4878947f),
                (16, 0.0f), // next 8, top left gyyph uv morph value
                (17, 1.0f),
                (18, 0.01953125f),
                (19, 1.0f),
                (20, 0.0f),
                (21, 0.9583333f),
                (22, 0.01953125f),
                (23, 0.9583333f)
            };

            animationStates.StateCountOutput = (uint)stateData.Length;
            if (animationStates.StateCapacityInput == 0)
            {
                return Result.Success;
            }

            if (animationStates.StateCapacityInput >= animationStates.StateCountOutput)
            {
                IntPtr ptr = animationStates.StatesBuffer;
                var state = new OVRPlugin.VirtualKeyboardModelAnimationState();
                for (int i = 0; i < stateData.Length; i++)
                {
                    state.AnimationIndex = stateData[i].Item1;
                    state.Fraction = stateData[i].Item2;
                    Marshal.StructureToPtr(state, ptr, false);
                    ptr += Marshal.SizeOf<OVRPlugin.VirtualKeyboardModelAnimationState>();
                }

                return Result.Success;
            }

            return Result.Failure;
        }

        public override Result ovrp_SetVirtualKeyboardModelVisibility(
            ref OVRPlugin.VirtualKeyboardModelVisibility visibility)
        {
            Visibility = visibility;
            return Result.Success;
        }
    }

    public class MockOVRPhysicsRaycaster : OVRPhysicsRaycaster
    {
        public Action<PointerEventData, List<RaycastResult>> OnRaycast;
        public int OnRaycastCount;
        public override void Raycast(PointerEventData eventData, List<RaycastResult> resultAppendList)
        {
            OnRaycastCount++;
            OnRaycast?.Invoke(eventData, resultAppendList);
        }
    }

    private const string testScenePath_ = "Oculus/VR/Scenes/VirtualKeyboard";

    private static readonly OVRPlugin.Posef NearDefaultPose = new OVRPlugin.Posef
    {
        Orientation = Quaternion.Euler(65, 0, 0).ToFlippedZQuatf(),
        Position = new Vector3f
        {
            x = 0,
            y = -0.4f,
            z = -0.4f // runtime has flipped z
        }
    };

    private static readonly OVRPlugin.Posef FarDefaultPose = new OVRPlugin.Posef
    {
        Orientation = Quatf.identity,
        Position = new Vector3f
        {
            x = 0,
            y = -0.5f,
            z = -1f // runtime has flipped z
        }
    };

    private FakeOVRPlugin83 fakeOVRPlugin_83_;
    private FakeOVRPlugin74 fakeOVRPlugin_74_;
    private FakeOVRPlugin68 fakeOVRPlugin_68_;
    private FakeOVRPlugin64 fakeOVRPlugin_64_;
    private FakeOVRPlugin_44 fakeOVRPlugin_44_;
    private FakeOVRPlugin38 fakeOVRPlugin_38_;
    private FakeOVRPlugin0 fakeOVRPlugin_0_;

    private OVRVirtualKeyboard ovrVirtualKeyboard;

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        fakeOVRPlugin_83_ = new FakeOVRPlugin83();
        OVRPlugin.OVRP_1_83_0.mockObj = fakeOVRPlugin_83_;

        fakeOVRPlugin_74_ = new FakeOVRPlugin74();
        OVRPlugin.OVRP_1_74_0.mockObj = fakeOVRPlugin_74_;

        fakeOVRPlugin_68_ = new FakeOVRPlugin68();
        OVRPlugin.OVRP_1_68_0.mockObj = fakeOVRPlugin_68_;

        fakeOVRPlugin_64_ = new FakeOVRPlugin64(() => fakeOVRPlugin_74_.TestLocationInfo.pose);
        OVRPlugin.OVRP_1_64_0.mockObj = fakeOVRPlugin_64_;

        // Hand Tracking
        fakeOVRPlugin_44_ = new FakeOVRPlugin_44();
        OVRPlugin.OVRP_1_44_0.mockObj = fakeOVRPlugin_44_;

        // Controller Tracking
        fakeOVRPlugin_38_ = new FakeOVRPlugin38();
        OVRPlugin.OVRP_1_38_0.mockObj = fakeOVRPlugin_38_;

        // TrackingSpace
        fakeOVRPlugin_0_ = new FakeOVRPlugin0();
        OVRPlugin.OVRP_1_0_0.mockObj = fakeOVRPlugin_0_;

        yield return LoadTestScene(testScenePath_);

        // Destroy the OVR Input Module to simplify test mocks
        GameObject.Destroy(GameObject.FindAnyObjectByType<OVRInputModule>());

        ovrVirtualKeyboard = GameObject.FindAnyObjectByType<OVRVirtualKeyboard>();

        // simulate initial keyboard shown event
        EventQueue.EnqueueEventVirtualKeyboardShown();
        yield return null;

        LogAssert.Expect(LogType.Log, new Regex(@"^Virtual Keyboard Duration(.+)$"));
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        yield return base.UnityTearDown();
        OVRPlugin.OVRP_1_83_0.mockObj = new OVRPlugin.OVRP_1_83_0_TEST();
        OVRPlugin.OVRP_1_74_0.mockObj = new OVRPlugin.OVRP_1_74_0_TEST();
        OVRPlugin.OVRP_1_68_0.mockObj = new OVRPlugin.OVRP_1_68_0_TEST();
        OVRPlugin.OVRP_1_64_0.mockObj = new OVRPlugin.OVRP_1_64_0_TEST();
        OVRPlugin.OVRP_1_44_0.mockObj = new OVRPlugin.OVRP_1_44_0_TEST();
        OVRPlugin.OVRP_1_38_0.mockObj = new OVRPlugin.OVRP_1_38_0_TEST();
        OVRPlugin.OVRP_1_0_0.mockObj = new OVRPlugin.OVRP_1_0_0_TEST();
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestKeyboardCreation()
    {
        Assert.That(ovrVirtualKeyboard.enabled, Is.True);
        yield return new OVRVirtualKeyboard.WaitUntilKeyboardVisible(ovrVirtualKeyboard);

        // Check collision initialized
        Assert.That(ovrVirtualKeyboard.Collider, Is.Not.Null);
        Assert.That(ovrVirtualKeyboard.Collider.gameObject.name, Is.EqualTo("collision"));

        // note: this could change if the gLTF test model is updated.
        var root = ovrVirtualKeyboard.transform.FindChildRecursive("root");
        Assert.That(root, Is.Not.Null);
        // check that root is visible
        Assert.That(root.localScale, Is.EqualTo(Vector3.one).Using(Vector3EqualityComparer.Instance));

        var defaultLayout = ovrVirtualKeyboard.transform.FindChildRecursive("default_layout");
        Assert.That(defaultLayout, Is.Not.Null);
        // check that default layout is visible
        Assert.That(defaultLayout.localScale, Is.EqualTo(Vector3.one).Using(Vector3EqualityComparer.Instance));
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestKeyboardInput()
    {
        Assert.That(ovrVirtualKeyboard.enabled, Is.True);
        yield return new OVRVirtualKeyboard.WaitUntilKeyboardVisible(ovrVirtualKeyboard);

        // Check collision initialized
        Assert.That(ovrVirtualKeyboard.Collider, Is.Not.Null);
        Assert.That(ovrVirtualKeyboard.Collider.gameObject.name, Is.EqualTo("collision"));

        var currentFrameCount = Time.frameCount;
        // Controllers will check for valid node positions
        var onGetNodeCount = 0;
        fakeOVRPlugin_38_.OnGetNodePositionValid += (OVRPlugin.Node id, ref OVRPlugin.Bool valid) =>
        {
            if (currentFrameCount != Time.frameCount)
            {
                currentFrameCount = Time.frameCount;
                onGetNodeCount = 0;
            }
            onGetNodeCount++;
            valid = OVRPlugin.Bool.True;
            return Result.Success;
        };

        // delay a frame to collect input
        yield return null;
        // wait until end of frame to measure results
        yield return null;
        // Input Loop
        Assert.That(fakeOVRPlugin_74_.SendInputInfoList.Count, Is.EqualTo(6));
        // Assert Controllers
        Assert.That(onGetNodeCount, Is.EqualTo(4)); // 2 calls for controllers 2 calls for hands
        Assert.That(fakeOVRPlugin_74_.SendInputInfoList[0].inputSource, Is.EqualTo(OVRPlugin.VirtualKeyboardInputSource.ControllerRayLeft));
        Assert.That(fakeOVRPlugin_74_.SendInputInfoList[1].inputSource, Is.EqualTo(OVRPlugin.VirtualKeyboardInputSource.ControllerDirectLeft));
        Assert.That(fakeOVRPlugin_74_.SendInputInfoList[2].inputSource, Is.EqualTo(OVRPlugin.VirtualKeyboardInputSource.ControllerRayRight));
        Assert.That(fakeOVRPlugin_74_.SendInputInfoList[3].inputSource, Is.EqualTo(OVRPlugin.VirtualKeyboardInputSource.ControllerDirectRight));
        // Assert Hands
        Assert.That(fakeOVRPlugin_74_.SendInputInfoList[4].inputSource, Is.EqualTo(OVRPlugin.VirtualKeyboardInputSource.HandRayLeft));
        Assert.That(fakeOVRPlugin_74_.SendInputInfoList[5].inputSource, Is.EqualTo(OVRPlugin.VirtualKeyboardInputSource.HandRayRight));

        OVRCameraRig rig = GameObject.FindAnyObjectByType<OVRCameraRig>();
        var mockRaycaster = rig.gameObject.AddComponent<MockOVRPhysicsRaycaster>();

        ovrVirtualKeyboard.controllerRaycaster = mockRaycaster;
        ovrVirtualKeyboard.handRaycaster = mockRaycaster;

        // Expect 4 hits (controller left, controller right, hand left, hand right)
        mockRaycaster.OnRaycast += (e, list) =>
        {
            // 1nd and 2rd return nothing, no collision with keyboard for controllers
            // 3rd - hit something that is not the keyboard with hand left
            if (mockRaycaster.OnRaycastCount == 3)
            {
                var result = new RaycastResult();
                result.gameObject = rig.gameObject;
                list.Add(result);
            }
            // 4th - hit, the only valid cast for hand right
            else if (mockRaycaster.OnRaycastCount == 4)
            {
                var result = new RaycastResult();
                result.gameObject = ovrVirtualKeyboard.Collider.gameObject;
                list.Add(result);
            }
        };

        // loop input again
        yield return null;

        // 2 for hands, 2 for controllers
        Assert.That(mockRaycaster.OnRaycastCount, Is.EqualTo(4));
        // 2 controller direct and 1 for the mocked raycaster
        Assert.That(onGetNodeCount, Is.EqualTo(4)); // 2 calls for controllers 2 calls for hands
        Assert.That(fakeOVRPlugin_74_.SendInputInfoList.Count, Is.EqualTo(3));
        Assert.That(fakeOVRPlugin_74_.SendInputInfoList[0].inputSource, Is.EqualTo(OVRPlugin.VirtualKeyboardInputSource.ControllerDirectLeft));
        Assert.That(fakeOVRPlugin_74_.SendInputInfoList[1].inputSource, Is.EqualTo(OVRPlugin.VirtualKeyboardInputSource.ControllerDirectRight));
        Assert.That(fakeOVRPlugin_74_.SendInputInfoList[2].inputSource, Is.EqualTo(OVRPlugin.VirtualKeyboardInputSource.HandRayRight));

        // disable input and loop again
        fakeOVRPlugin_74_.SendInputInfoList.Clear();
        ovrVirtualKeyboard.InputEnabled = false;
        yield return null;
        // Assert no increase
        Assert.That(fakeOVRPlugin_74_.SendInputInfoList.Count, Is.Zero);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestKeyboardCreateFailures()
    {
        // Virtual Keyboard Extension Support
        fakeOVRPlugin_74_.CreateVirtualKeyboardDefaultReturnValue = Result.Failure;
        LogAssert.Expect(LogType.Error, "Create failed: 'Failure'. Check for Virtual Keyboard Support.");
        yield return InstantiateViaAddComponent();
        GameObject.Destroy(ovrVirtualKeyboard.gameObject);

        // Virtual Keyboard Extension Support
        fakeOVRPlugin_74_.CreateVirtualKeyboardDefaultReturnValue = Result.Failure_NotInitialized;
        LogAssert.Expect(LogType.Warning, "Virtual Keyboard Unity Editor support requires Quest Link.");
        yield return InstantiateViaAddComponent();
        GameObject.Destroy(ovrVirtualKeyboard.gameObject);

        // Render Model Extension Support
        fakeOVRPlugin_74_.CreateVirtualKeyboardDefaultReturnValue = Result.Success;
        fakeOVRPlugin_68_.GetRenderModelPathsDefaultReturnValue = Result.Failure;
        LogAssert.Expect(LogType.Error, "Failed to find keyboard model.  Check Render Model support.");
        yield return InstantiateViaAddComponent();
        GameObject.Destroy(ovrVirtualKeyboard.gameObject);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestKeyboardTransformUpdates()
    {
        var kbTransform = ovrVirtualKeyboard.transform;
        kbTransform.rotation = FakeOVRPlugin74.TestRotation.FromFlippedZQuatf();
        kbTransform.position = FakeOVRPlugin74.TestPosition.FromFlippedZVector3f();
        kbTransform.localScale = UnityEngine.Vector3.one *  0.5f;
        yield return null;
        Assert.That(fakeOVRPlugin_74_.TestLocationInfo.pose.Position.FromFlippedZVector3f()
            , Is.EqualTo(kbTransform.position).Using(Vector3EqualityComparer.Instance));
        Assert.That(fakeOVRPlugin_74_.TestLocationInfo.pose.Orientation.FromFlippedZQuatf(), Is.EqualTo(kbTransform.rotation).Using(QuaternionEqualityComparer.Instance));
        Assert.That(fakeOVRPlugin_74_.TestLocationInfo.scale, Is.EqualTo(kbTransform.localScale.x));
        Assert.That(fakeOVRPlugin_74_.TestLocationInfo.trackingOriginType, Is.EqualTo(OVRPlugin.TrackingOrigin.EyeLevel));

        OVRPlugin.SetTrackingOriginType(OVRPlugin.TrackingOrigin.Stage);

        // Assert scale uniformity and tracking origin changes
        kbTransform.localScale = new UnityEngine.Vector3(5f, 1f, 1f);
        yield return null;
        Assert.That(kbTransform.localScale
            , Is.EqualTo(5f * UnityEngine.Vector3.one).Using(Vector3EqualityComparer.Instance));
        Assert.That(fakeOVRPlugin_74_.TestLocationInfo.trackingOriginType, Is.EqualTo(OVRPlugin.TrackingOrigin.Stage));
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestKeyboardSetActiveUpdatesKeyboardVisibility()
    {
        // Disable while loading
        ovrVirtualKeyboard.gameObject.SetActive(false);
        yield return null;
        // Ensure keyboard is not entirely destroyed
        Assert.IsTrue(Mathf.Equals(0, fakeOVRPlugin_74_.StopDestroyCount));
        yield return null;

        // re-enable to ensure keyboard still loads
        ovrVirtualKeyboard.gameObject.SetActive(true);
        yield return new OVRVirtualKeyboard.WaitUntilKeyboardVisible(ovrVirtualKeyboard);

        // Once the model is loaded, SetActive will propagate to the runtime.
        Assert.That(fakeOVRPlugin_83_.Visibility.Visible, Is.True);
        yield return null;
        ovrVirtualKeyboard.gameObject.SetActive(false);
        yield return null;
        Assert.That(fakeOVRPlugin_83_.Visibility.Visible, Is.False);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestKeyboardIsDestroyed()
    {
        GameObject.Destroy(ovrVirtualKeyboard.gameObject);
        yield return null;
        Assert.IsTrue(Mathf.Equals(1, fakeOVRPlugin_74_.StopDestroyCount));

        // Assert Exception "Virtual Keyboard Input Source Disposed" does not appear
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestKeyboardIsDestroyedOrCreatedWithButton()
    {
        yield return new OVRVirtualKeyboard.WaitUntilKeyboardVisible(ovrVirtualKeyboard);
        var inputHandler = GameObject.FindAnyObjectByType<OVRVirtualKeyboardSampleInputHandler>();
        var oldKeyboard = inputHandler.OVRVirtualKeyboard;
        ExecuteEvents.Execute(GameObject.Find("ButtonDestroyKeyboard"), new BaseEventData(EventSystem.current), ExecuteEvents.submitHandler);
        yield return null;
        Assert.IsTrue(Mathf.Equals(1, fakeOVRPlugin_74_.StopDestroyCount));

        // Assert Exception "Virtual Keyboard Input Source Disposed" does not appear
        yield return null;

        ExecuteEvents.Execute(GameObject.Find("ButtonShowKeyboard"), new BaseEventData(EventSystem.current), ExecuteEvents.submitHandler);
        yield return null;

        var newKeyboard = GameObject.FindAnyObjectByType<OVRVirtualKeyboard>();

        Assert.That(newKeyboard, Is.Not.EqualTo(oldKeyboard));
        Assert.That(inputHandler.OVRVirtualKeyboard, Is.Not.EqualTo(oldKeyboard));
        Assert.That(inputHandler.OVRVirtualKeyboard, Is.EqualTo(newKeyboard));

        Assert.That(newKeyboard.leftControllerDirectTransform, Is.Not.Null);
        Assert.That(newKeyboard.leftControllerRootTransform, Is.Not.Null);
        Assert.That(newKeyboard.rightControllerDirectTransform, Is.Not.Null);
        Assert.That(newKeyboard.rightControllerRootTransform, Is.Not.Null);
        Assert.That(newKeyboard.handLeft, Is.Not.Null);
        Assert.That(newKeyboard.handRight, Is.Not.Null);

        // Destroy Keyboard without waiting for it to load entirely
        GameObject.DestroyImmediate(newKeyboard);
        yield return null;
        // Check it has been destroyed without errors
        Assert.That(newKeyboard == null, Is.True);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestInstantiateViaAddComponent()
    {
        yield return InstantiateViaAddComponent();
        Assert.That(ovrVirtualKeyboard.isActiveAndEnabled, Is.True);

        // wait another frame to ensure no input errors
        yield return null;
        Assert.That(ovrVirtualKeyboard.isActiveAndEnabled, Is.True);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestOneKeyboardLimit()
    {
        // Initialize default test kb
        yield return null;

        // Create another
        var tempKeyboardGo = new GameObject("tempKeyboard");

        LogAssert.Expect(LogType.Exception, "Exception: OVRVirtualKeyboard only supports a single instance");
        var keyboard = tempKeyboardGo.AddComponent<OVRVirtualKeyboard>();
        Assert.That(keyboard.enabled, Is.False);

        Object.Destroy(tempKeyboardGo);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestSuggestNearKeyboard()
    {
        yield return null;
        ovrVirtualKeyboard.UseSuggestedLocation(OVRVirtualKeyboard.KeyboardPosition.Near);
        Assert.That(
            ovrVirtualKeyboard.transform.position,
            Is.EqualTo(NearDefaultPose.Position.FromFlippedZVector3f()).Using(Vector3EqualityComparer.Instance));
        yield return null;
        Assert.That(
            ovrVirtualKeyboard.transform.position,
            Is.EqualTo(NearDefaultPose.Position.FromFlippedZVector3f()).Using(Vector3EqualityComparer.Instance));
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestSuggestFarKeyboard()
    {
        yield return null;
        ovrVirtualKeyboard.UseSuggestedLocation(OVRVirtualKeyboard.KeyboardPosition.Far);
        Assert.That(
            ovrVirtualKeyboard.transform.position,
            Is.EqualTo(FarDefaultPose.Position.FromFlippedZVector3f()).Using(Vector3EqualityComparer.Instance));
        yield return null;
        Assert.That(
            ovrVirtualKeyboard.transform.position,
            Is.EqualTo(FarDefaultPose.Position.FromFlippedZVector3f()).Using(Vector3EqualityComparer.Instance));
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestSuggestCustomKeyboard()
    {
        yield return new OVRVirtualKeyboard.WaitUntilKeyboardVisible(ovrVirtualKeyboard);
        Assert.That(fakeOVRPlugin_74_.SuggestVirtualKeyboardLocationCallCount, Is.EqualTo(2));
        fakeOVRPlugin_74_.SuggestVirtualKeyboardLocationCallCount = 0;
        var expectedPositon = Vector3.one * 3;
        ovrVirtualKeyboard.transform.position = expectedPositon;
        ovrVirtualKeyboard.UseSuggestedLocation(OVRVirtualKeyboard.KeyboardPosition.Custom);
        Assert.That(
            ovrVirtualKeyboard.transform.position,
            Is.EqualTo(expectedPositon).Using(Vector3EqualityComparer.Instance));
        yield return null;
        Assert.That(
            ovrVirtualKeyboard.transform.position,
            Is.EqualTo(expectedPositon).Using(Vector3EqualityComparer.Instance));
        Assert.That(fakeOVRPlugin_74_.SuggestVirtualKeyboardLocationCallCount, Is.EqualTo(1));
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestVirtualKeyboardTextEvents()
    {
        var existingInputField = GameObject.FindAnyObjectByType<InputField>();
        // multiline is default
        Assert.That(existingInputField, Is.Not.Null);
        Assert.That(existingInputField.multiLine, Is.True);
        var go = new GameObject();

        var inputFieldSingleField = go.AddComponent<InputField>();
        Assert.That(inputFieldSingleField, Is.Not.Null);
        Assert.That(inputFieldSingleField.multiLine, Is.False);

        var goMultilineSubmit = new GameObject();

        var inputFieldMultilineSubmit = goMultilineSubmit.AddComponent<InputField>();
        inputFieldMultilineSubmit.lineType = InputField.LineType.MultiLineSubmit;
        Assert.That(inputFieldMultilineSubmit, Is.Not.Null);
        Assert.That(inputFieldMultilineSubmit.multiLine, Is.True);

        // Test text context changes
        Assert.That(fakeOVRPlugin_74_.LastReceivedTextContext, Is.EqualTo(string.Empty));
        LogAssert.Expect(LogType.Warning, "TextHandler text out of sync with Keyboard text context");
        ovrVirtualKeyboard.ChangeTextContext("Temp Text Context");
        Assert.That(fakeOVRPlugin_74_.LastReceivedTextContext, Is.EqualTo("Temp Text Context"));
        existingInputField.text = "Temp";
        existingInputField.text = "";
        Assert.That(fakeOVRPlugin_74_.LastReceivedTextContext, Is.EqualTo(string.Empty));

        int valueChangeCount = 0;
        string lastChangedValue = String.Empty;
        var endEditCount = 0;
        var endEditLastValue = String.Empty;
        foreach (var inputField in new [] {existingInputField, inputFieldSingleField, inputFieldMultilineSubmit})
        {
            inputField.onValueChanged.AddListener((s) =>
            {
                valueChangeCount++;
                lastChangedValue = s;
            });
            inputField.onEndEdit.AddListener((s) =>
            {
                endEditCount++;
                endEditLastValue = s;
            });
        }

        var committedText = String.Empty;
        ovrVirtualKeyboard.CommitTextEvent.AddListener((s) => committedText = s);

        EventQueue.EnqueueEventVirtualKeyboardCommitText("s");
        yield return null;
        Assert.That(committedText, Is.EqualTo("s"));
        Assert.That(lastChangedValue, Is.EqualTo("s"));
        Assert.That(valueChangeCount, Is.EqualTo(1));
        Assert.That(fakeOVRPlugin_74_.LastReceivedTextContext, Is.Empty);

        EventQueue.EnqueueEventVirtualKeyboardCommitText("Sardine ");
        yield return null;
        Assert.That(committedText, Is.EqualTo("Sardine "));
        Assert.That(lastChangedValue, Is.EqualTo("sSardine "));
        Assert.That(valueChangeCount, Is.EqualTo(2));
        Assert.That(fakeOVRPlugin_74_.LastReceivedTextContext, Is.Empty);

        var backspaceCount = 0;
        ovrVirtualKeyboard.BackspaceEvent.AddListener(() => backspaceCount++);
        EventQueue.EnqueueEventVirtualKeyboardBackspace();
        yield return null;
        Assert.That(backspaceCount, Is.EqualTo(1));
        Assert.That(lastChangedValue, Is.EqualTo("sSardine"));
        Assert.That(valueChangeCount, Is.EqualTo(3));

        var enterCount = 0;
        ovrVirtualKeyboard.EnterEvent.AddListener(() => enterCount++);
        EventQueue.EnqueueEventVirtualKeyboardEnter();
        yield return null;
        Assert.That(enterCount, Is.EqualTo(1));
        Assert.That(lastChangedValue, Is.EqualTo("sSardine\n"));
        Assert.That(valueChangeCount, Is.EqualTo(4));

        // At the moment the virtual keyboard only supports a end of text caret position
        EventSystem.current.SetSelectedGameObject(existingInputField.gameObject);
        existingInputField.caretPosition = 0;
        EventQueue.EnqueueEventVirtualKeyboardCommitText("s");
        yield return null;
        Assert.That(committedText, Is.EqualTo("s"));
        Assert.That(lastChangedValue, Is.EqualTo("sSardine\ns"));
        Assert.That(valueChangeCount, Is.EqualTo(5));
        Assert.That(existingInputField.caretPosition, Is.EqualTo(lastChangedValue.Length));

        var textHandler = ovrVirtualKeyboard.GetComponent<OVRVirtualKeyboardInputFieldTextHandler>();

        textHandler.InputField = inputFieldSingleField;
        EventQueue.EnqueueEventVirtualKeyboardCommitText("s");
        yield return null;
        EventQueue.EnqueueEventVirtualKeyboardEnter();
        yield return null;
        Assert.That(endEditLastValue, Is.EqualTo("s"));
        Assert.That(endEditCount, Is.EqualTo(1));

        textHandler.InputField = inputFieldMultilineSubmit;
        EventQueue.EnqueueEventVirtualKeyboardCommitText("s");
        yield return null;
        EventQueue.EnqueueEventVirtualKeyboardEnter();
        yield return null;
        Assert.That(endEditLastValue, Is.EqualTo("s"));
        Assert.That(endEditCount, Is.EqualTo(2));
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestKeyboardEventHiddenAndShown()
    {
        var hiddenCount = 0;
        var shownCount = 0;
        ovrVirtualKeyboard.KeyboardHiddenEvent.AddListener(() => hiddenCount++);
        ovrVirtualKeyboard.KeyboardShownEvent.AddListener(() => shownCount++);
        EventQueue.EnqueueEventVirtualKeyboardHidden();
        yield return null;
        Assert.That(hiddenCount, Is.EqualTo(1));
        Assert.That(shownCount, Is.Zero);
        Assert.That(ovrVirtualKeyboard.isActiveAndEnabled, Is.False);

        EventQueue.EnqueueEventVirtualKeyboardShown();
        yield return null;

        Assert.That(hiddenCount, Is.EqualTo(1));
        Assert.That(shownCount, Is.EqualTo(1));
        Assert.That(ovrVirtualKeyboard.isActiveAndEnabled, Is.True);
    }

    private IEnumerator EnqueueSwipeEvents(string word)
    {
        EventQueue.EnqueueEventVirtualKeyboardCommitText($"{word[0]}");
        yield return null;
        EventQueue.EnqueueEventVirtualKeyboardBackspace();
        yield return null;
        EventQueue.EnqueueEventVirtualKeyboardCommitText($"{word} ");
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestVirtualKeyboardWPMPromptGame()
    {
        var prompt = GameObject.FindAnyObjectByType<OVRVirtualKeyboardSampleWPMPromptInternal>();
        Assert.That(prompt, Is.Not.Null);

        var promptCompleteCount = 0;

        prompt.OnWritingPromptComplete += (s) =>
        {
            promptCompleteCount++;
            Assert.That(s.errorPercentage, Is.Zero);
            Assert.That(s.wordsPerMinute, Is.GreaterThan(0));
            Assert.That(s.adjustedWordsPerMinute, Is.GreaterThan(0));
            Assert.That(s.totalSeconds, Is.GreaterThan(0));
        };

        yield return EnqueueSwipeEvents("The");
        yield return EnqueueSwipeEvents("quick");
        yield return EnqueueSwipeEvents("brown");
        yield return EnqueueSwipeEvents("fox");
        yield return EnqueueSwipeEvents("jumped");
        yield return EnqueueSwipeEvents("over");
        yield return EnqueueSwipeEvents("the");
        yield return EnqueueSwipeEvents("lazy");
        yield return EnqueueSwipeEvents("dog");

        Assert.That(promptCompleteCount, Is.EqualTo(1));
    }

    private IEnumerator InstantiateViaAddComponent()
    {
         // Destroy First
        GameObject.FindAnyObjectByType<OVRVirtualKeyboardSampleControls>().DestroyKeyboard();
        yield return null;
        Assert.IsTrue(Mathf.Equals(1, fakeOVRPlugin_74_.StopDestroyCount));

        var tempKeyboardGo = new GameObject("tempKeyboard");
        ovrVirtualKeyboard = tempKeyboardGo.AddComponent<OVRVirtualKeyboard>();
        Assert.That(ovrVirtualKeyboard.enabled, Is.True);

        // simulate initial keyboard shown event
        EventQueue.EnqueueEventVirtualKeyboardShown();

        // wait a frame to ensure no loading errors
        yield return null;
    }
}

#endif
#endif
