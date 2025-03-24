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
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools.Utils;

//-------------------------------------------------------------------------------------
/// <summary>
/// Tests for the OVRVirtualKeyboard's InteractorRootTransformOverride system, to handle runtime driven poke limiting/pinning.
/// </summary>
[TestFixture]
[Category("OnCall:xrinput_text_entry")]
public class OVRVirtualKeyboardInteractorRootTransformOverrideTest
{
    [Test]
    public void TestHappyPathTransformUpdateAndRevert()
    {
        var ir = new OVRVirtualKeyboard.InteractorRootTransformOverride();

        var go = new GameObject();

        var target = go.transform;

        ir.Enqueue(target, new OVRPlugin.Posef()
        {
            Orientation = OVRPlugin.Quatf.identity,
            Position = new Vector3(1, 0, 0).ToVector3f()
        });

        Assert.That(target.position, Is.EqualTo(Vector3.zero).Using(Vector3EqualityComparer.Instance));

        ir.LateApply(null);
        Assert.That(target.position, Is.EqualTo(new Vector3(1, 0, 0)).Using(Vector3EqualityComparer.Instance));

        ir.Reset();
        Assert.That(target.position, Is.EqualTo(Vector3.zero).Using(Vector3EqualityComparer.Instance));
    }

    [Test]
    public void TestTransformDoesNotUpdateMovedTransform()
    {
        var ir = new OVRVirtualKeyboard.InteractorRootTransformOverride();

        var go = new GameObject();

        var target = go.transform;

        ir.Enqueue(target, new OVRPlugin.Posef()
        {
            Orientation = OVRPlugin.Quatf.identity,
            Position = new Vector3(1, 0, 0).ToVector3f()
        });

        target.position = new Vector3(5, 0, 0);

        ir.LateApply(null);
        Assert.That(target.position, Is.Not.EqualTo(new Vector3(1, 0, 0)).Using(Vector3EqualityComparer.Instance));
        Assert.That(target.position, Is.EqualTo(new Vector3(5, 0, 0)).Using(Vector3EqualityComparer.Instance));

        ir.Reset();
        Assert.That(target.position, Is.EqualTo(new Vector3(5, 0, 0)).Using(Vector3EqualityComparer.Instance));
    }

    [Test]
    public void TestTransformDoesNotUpdateMovedLaterTransform()
    {
        var ir = new OVRVirtualKeyboard.InteractorRootTransformOverride();

        var go = new GameObject();

        var target = go.transform;

        ir.Enqueue(target, new OVRPlugin.Posef()
        {
            Orientation = OVRPlugin.Quatf.identity,
            Position = new Vector3(1, 0, 0).ToVector3f()
        });

        ir.LateApply(null);
        Assert.That(target.position, Is.EqualTo(new Vector3(1, 0, 0)).Using(Vector3EqualityComparer.Instance));

        target.position = new Vector3(5, 0, 0);

        ir.Reset();
        Assert.That(target.position, Is.Not.EqualTo(Vector3.zero).Using(Vector3EqualityComparer.Instance));
        Assert.That(target.position, Is.EqualTo(new Vector3(5, 0, 0)).Using(Vector3EqualityComparer.Instance));
    }
}

#endif
#endif // OVRPLUGIN_TESTING
