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

using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.SceneManagement;
using Bool = OVRPlugin.Bool;
using Result = OVRPlugin.Result;

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

    public override Bool ovrp_GetAppMonoscopic()
    {
        return Bool.False;
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

public interface IOVRPluginPlayModeTestEventQueue
{
    void EnqueueEventVirtualKeyboardCommitText(string s);
    void EnqueueEventVirtualKeyboardBackspace();
    void EnqueueEventVirtualKeyboardEnter();
    void EnqueueEventVirtualKeyboardShown();
    void EnqueueEventVirtualKeyboardHidden();
}

public class FakeOVRPlugin_55_1 : OVRPlugin.OVRP_1_55_1_TEST, IOVRPluginPlayModeTestEventQueue
{
    private const int EventDataBufferSize = 4000;

    public delegate Result PollEventDelegate(ref OVRPlugin.EventType eventType, ref IntPtr eventData);

    private readonly Queue<PollEventDelegate> _eventQueue = new Queue<PollEventDelegate>();
    private readonly Queue<IntPtr> _allocatedEvents = new Queue<IntPtr>();

    public override Result ovrp_PollEvent2(ref OVRPlugin.EventType eventType, ref IntPtr eventData)
    {
        return _eventQueue.Count > 0 ? _eventQueue.Dequeue().Invoke(ref eventType, ref eventData) : Result.Success;
    }

    ~FakeOVRPlugin_55_1()
    {
        while (_allocatedEvents.Count > 0)
        {
            Marshal.FreeHGlobal(_allocatedEvents.Dequeue());
        }
    }

    public void EnqueueEvent(PollEventDelegate eventDelegate)
    {
        _eventQueue.Enqueue(eventDelegate);
    }

    public void EnqueueEventVirtualKeyboardCommitText(string text)
    {
        EnqueueEvent(((ref OVRPlugin.EventType type, ref IntPtr data) =>
        {
            type = OVRPlugin.EventType.VirtualKeyboardCommitText;
            var textArray = text.ToCharArray();
            var textSize = Marshal.SizeOf<char>() * textArray.Length;
            AllocHGlobalEventData(ref data, textSize);
            Marshal.Copy(textArray, 0, data, textSize);
            return Result.Success;
        }));
    }

    public void EnqueueEventVirtualKeyboardBackspace()
    {
        EnqueueEvent(((ref OVRPlugin.EventType type, ref IntPtr data) =>
        {
            type = OVRPlugin.EventType.VirtualKeyboardBackspace;
            AllocHGlobalEventData(ref data, 0);
            return Result.Success;
        }));
    }

    public void EnqueueEventVirtualKeyboardEnter()
    {
        EnqueueEvent(((ref OVRPlugin.EventType type, ref IntPtr data) =>
        {
            type = OVRPlugin.EventType.VirtualKeyboardEnter;
            AllocHGlobalEventData(ref data, 0);
            return Result.Success;
        }));
    }

    public void EnqueueEventVirtualKeyboardShown()
    {
        EnqueueEvent(((ref OVRPlugin.EventType type, ref IntPtr data) =>
        {
            type = OVRPlugin.EventType.VirtualKeyboardShown;
            AllocHGlobalEventData(ref data, 0);
            return Result.Success;
        }));
    }

    public void EnqueueEventVirtualKeyboardHidden()
    {
        EnqueueEvent(((ref OVRPlugin.EventType type, ref IntPtr data) =>
        {
            type = OVRPlugin.EventType.VirtualKeyboardHidden;
            AllocHGlobalEventData(ref data, 0);
            return Result.Success;
        }));
    }

    private void AllocHGlobalEventData(ref IntPtr data, int dataLength)
    {
        if (dataLength > EventDataBufferSize)
        {
            throw new Exception($"dataLength > {EventDataBufferSize}");
        }

        data = Marshal.AllocHGlobal(EventDataBufferSize);
        // zero it out
        for (int i = 0; i < EventDataBufferSize; i++)
        {
            Marshal.WriteByte(data, i, 0x00);
        }

        _allocatedEvents.Enqueue(data);
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

// Base class for any Unity test for OVRPlugin that needs to set OVRPlugin.initialized to true
// (generally recommended). Stubs out some things that break in this scenario.
public class OVRPluginPlayModeTest
{
    protected virtual bool TelemetryMockedOut => true;

    protected const int DefaultTimeoutMs = 10000;

    protected IOVRPluginPlayModeTestEventQueue EventQueue { get; private set; }

    public virtual IEnumerator UnitySetUp()
    {
        // May not be cleaned up if previous test timed out, failed during setup, etc.
        yield return CleanupLeftover();
        OVRPlugin.OVRP_1_1_0.mockObj = new FakeOVRPlugin_1();
        OVRPlugin.OVRP_1_3_0.mockObj = new FakeOVRPlugin_3();
        OVRPlugin.OVRP_1_66_0.mockObj = new FakeOVRPlugin_66();
        OVRPlugin.OVRP_1_70_0.mockObj = new FakeOVRPlugin_70();
        var fake551 = new FakeOVRPlugin_55_1();
        EventQueue = fake551;
        OVRPlugin.OVRP_1_55_1.mockObj = fake551;

        if (TelemetryMockedOut)
        {
            OVRTelemetry.Mock(value: false);
        }

        OVRTask.OnEnterPlayMode();
        OVRAnchor.Init();
    }

    public virtual IEnumerator UnityTearDown()
    {
        // Point mockObjs back at original production version of code
        OVRPlugin.OVRP_1_1_0.mockObj = new OVRPlugin.OVRP_1_1_0_TEST();
        OVRPlugin.OVRP_1_3_0.mockObj = new OVRPlugin.OVRP_1_3_0_TEST();
        OVRPlugin.OVRP_1_66_0.mockObj = new OVRPlugin.OVRP_1_66_0_TEST();
        OVRPlugin.OVRP_1_70_0.mockObj = new OVRPlugin.OVRP_1_70_0_TEST();
        OVRPlugin.OVRP_1_55_1.mockObj = new OVRPlugin.OVRP_1_55_1_TEST();
        EventQueue = null;

        if (TelemetryMockedOut)
        {
            OVRTelemetry.Unmock();
        }

        yield return CleanupLeftover();
    }

    private IEnumerator CleanupLeftover()
    {
        // Clean up any scenes or GameObjects left over from previous tests,
        // keep only the tests runner object required by the test system.

        for (int i = SceneManager.sceneCount - 1; i >= 1; i--)
        {
            var asyncOperation = SceneManager.UnloadSceneAsync(SceneManager.GetSceneAt(i).name); // Clear/reset scene
            yield return new WaitUntil(() => asyncOperation.isDone);
        }

        for (int sceneNum = 0; sceneNum < SceneManager.sceneCount; sceneNum++)
        {
            List<GameObject> rootObjects = new List<GameObject>();
            Scene scene = SceneManager.GetSceneAt(sceneNum);
            scene.GetRootGameObjects(rootObjects);

            for (int i = 0; i < rootObjects.Count; i++)
            {
                if (rootObjects[i].GetComponent("PlaymodeTestsController") == null)
                {
                    UnityEngine.Object.DestroyImmediate(rootObjects[i]);
                }
            }
        }

        yield return null;
    }

    protected IEnumerator LoadTestScene(string path)
    {
        var asyncOperation = SceneManager.LoadSceneAsync(path, LoadSceneMode.Additive);
        yield return new WaitUntil(() => asyncOperation.isDone);
    }

    protected IEnumerator UnloadTestScene(string path)
    {
        var asyncOperation = SceneManager.UnloadSceneAsync(path);
        yield return new WaitUntil(() => asyncOperation.isDone);
    }

    /// <summary>
    /// Implements the two-call idiom.
    /// </summary>
    /// <remarks>
    /// For more details, refer to the OpenXR spec section on
    /// [Buffer Size Parameters](https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#buffer-size-parameters)
    /// </remarks>
    /// <param name="requiredElementCount">The required size of <paramref name="elements"/>, in number of elements</param>
    /// <param name="src">A pointer to the source data</param>
    /// <param name="elementCapacityInput">The capacity (number of elements) of <paramref name="elements"/> provided by the caller.</param>
    /// <param name="elementCountOutput">The number of elements that will be written <paramref name="elements"/> if all checks pass.</param>
    /// <param name="elements">The buffer (typically provided by the caller) to write data to.</param>
    /// <typeparam name="T">The type of data stored in the buffer.</typeparam>
    /// <returns>
    /// If <paramref name="elementCapacityInput"/> is zero, stores the required bytes in
    /// <paramref name="elementCountOutput"/> and returns <see cref="Result.Success"/>.
    ///
    /// If <paramref name="elementCapacityInput"/> is nonzero and is less than <paramref name="requiredElementCount"/>,
    /// returns <see cref="Result.Failure_InsufficientSize"/>.
    ///
    /// If <paramref name="elementCapacityInput"/> is greater than or equal to <paramref name="requiredElementCount"/>,
    /// <paramref name="requiredElementCount"/> elements are copied from <paramref name="src"/> to <paramref name="elements"/> and
    /// returns <see cref="Result.Success"/>.
    /// </returns>
    /// <exception cref="ArgumentNullException">Thrown if <paramref name="requiredElementCount"/> is greater than zero but
    /// <paramref name="src"/> is `null`.</exception>
    protected static unsafe Result TwoCall<T>(uint requiredElementCount, T* src, uint elementCapacityInput,
        out uint elementCountOutput, T* elements) where T : unmanaged
    {
        elementCountOutput = requiredElementCount;

        if (elementCapacityInput == 0)
        {
            return Result.Success;
        }

        if (elementCapacityInput < elementCountOutput)
        {
            return Result.Failure_InsufficientSize;
        }

        if (elementCountOutput == 0)
        {
            return Result.Success;
        }

        if (elements == null)
        {
            return Result.Failure_InvalidParameter;
        }

        if (src == null)
        {
            throw new ArgumentNullException(nameof(src));
        }

        UnsafeUtility.MemCpy(elements, src, elementCountOutput * sizeof(T));

        return Result.Success;
    }

    /// <summary>
    /// Implements the <see cref="TwoCall{T}"/> for UTF-8 strings.
    /// </summary>
    /// <remarks>
    /// For more details, refer to the OpenXR spec section on
    /// [Buffer Size Parameters](https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#buffer-size-parameters)
    /// </remarks>
    /// <param name="src">The source string that serves as the data being requested.</param>
    /// <param name="elementCapacityInput">The capacity (number of elements) of <paramref name="elements"/> provided by the caller.</param>
    /// <param name="elementCountOutput">The number of elements that will be written <paramref name="elements"/> if all checks pass.</param>
    /// <param name="elements">The buffer (typically provided by the caller) to write data to.</param>
    /// <returns>Refer to
    /// [Buffer Size Parameters](https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#buffer-size-parameters)
    /// for return values.
    /// </returns>
    protected static unsafe Result TwoCall(string src, uint elementCapacityInput,
        out uint elementCountOutput, byte* elements)
    {
        if (string.IsNullOrEmpty(src))
        {
            return TwoCall(0, null, elementCapacityInput, out elementCountOutput, elements);
        }

        var encoder = System.Text.Encoding.UTF8;
        var bytes = encoder.GetBytes(src);
        fixed (byte* ptr = bytes)
        {
            return TwoCall((uint)encoder.GetByteCount(src), ptr, elementCapacityInput,
                out elementCountOutput, elements);
        }
    }
}

#endif
