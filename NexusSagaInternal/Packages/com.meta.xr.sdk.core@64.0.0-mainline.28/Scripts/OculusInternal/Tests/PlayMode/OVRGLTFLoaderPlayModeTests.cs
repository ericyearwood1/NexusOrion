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
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;

[Category("OnCall:xrinput_text_entry")]
internal class OVRGLTFLoaderPlayModeTests
{
    public struct OVRGLTFModelTestCase
    {
        public string glbFilename;
    }

    private const string ModelRoot = "Assets/Oculus/VR/Scripts/OculusInternal/Tests/PlayMode/Resources/";

    private static readonly OVRGLTFModelTestCase ControllerOvrgltfModelTestCase = new()
    {
        glbFilename = "rtouch_controller.glb"
    };

    private static readonly OVRGLTFModelTestCase VirtualKeyboardOvrgltfModelTestCase = new()
    {
        glbFilename = "virtual_keyboard.glb"
    };

    private static readonly OVRGLTFModelTestCase TrackedKeyboardOvrgltfModelTestCase = new()
    {
        glbFilename = "apple_macbook_13_Lid_AlphaBlend_sm_glb.bytes",
    };

    private static readonly OVRGLTFModelTestCase TrackedKeyboardK830OvrgltfModelTestCase = new()
    {
        glbFilename = "k830_glb.bytes",
    };

    private static readonly OVRGLTFModelTestCase[] TestCaseData = new[]
    {
        ControllerOvrgltfModelTestCase,
        VirtualKeyboardOvrgltfModelTestCase,
        TrackedKeyboardOvrgltfModelTestCase,
        TrackedKeyboardK830OvrgltfModelTestCase
    };

    [UnityTest]
    /***
     * Load every test GLTF model and ensure no errors. Use the unit test for more focused tests
     */
    public IEnumerator LoadGLBLoadsAllTestModels()
    {
        foreach(var test in TestCaseData)
        {
            var loader = new OVRGLTFLoader(ModelRoot + test.glbFilename);
            var result = loader.LoadGLB(true, true);
            yield return null;
            GameObject.Destroy(loader.scene.root);
            yield return null;
        }
    }

    [UnityTest]
    /***
     * Load every test GLTF model async and ensure no errors. Use the unit test for more focused tests
     */
    public IEnumerator LoadGLBAsyncLoadsAllTestModels()
    {
        /***
         * Load every test GLTF model and ensure no errors
         */
        foreach(var test in TestCaseData)
        {
            var loader = new OVRGLTFLoader(ModelRoot + test.glbFilename);
            yield return loader.LoadGLBCoroutine(true, true);
            yield return null;
            GameObject.Destroy(loader.scene.root);
            yield return null;
        }
    }
}

#endif
