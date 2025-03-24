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

#if OVR_INTERNAL_CODE
#if OVRPLUGIN_TESTING

using System;
using System.Collections;
using NUnit.Framework;
using UnityEngine.TestTools;

namespace Meta.XR.Editor.Tags.Tests
{
    internal class TagBehaviorTests : OVRPluginEditModeTest
    {
        [UnitySetUp]
        public override IEnumerator UnitySetUp()
        {
            yield return base.UnitySetUp();

            TagBehavior.Registry.Clear();
        }

        [UnityTearDown]
        public override IEnumerator UnityTearDown()
        {
            TagBehavior.Registry.Clear();

            yield return base.UnityTearDown();
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagBehavior_GetBehavior_ArgumentNullException()
        {
            Assert.Catch<ArgumentNullException>(() =>
            {
                var fetchedBehavior = TagBehavior.GetBehavior(null);
            });

            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagBehavior_Behavior_AddsToDictionary()
        {
            var tag = new Tag("Test");
            var tagBehavior = tag.Behavior;
            var fetchedBehavior = TagBehavior.GetBehavior(tag);

            Assert.AreSame(tagBehavior, fetchedBehavior);

            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagBehavior_GetBehavior_AddsToDictionary()
        {
            var tag = new Tag("Test");
            var fetchedBehavior = TagBehavior.GetBehavior(tag);
            var tagBehavior = tag.Behavior;

            Assert.AreEqual(tagBehavior, fetchedBehavior);

            var tag2 = new Tag("Test2");
            var tag2Behavior = tag2.Behavior;

            Assert.AreNotEqual(tagBehavior, tag2Behavior);

            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagBehavior_Behavior_IsDifferentPerTag()
        {
            var tag = new Tag("Test");
            var tag2 = new Tag("Test2");
            var tagBehavior = tag.Behavior;
            var tag2Behavior = tag2.Behavior;

            Assert.AreNotEqual(tagBehavior, tag2Behavior);

            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagBehavior_Order_SetAndGet()
        {
            var tag = new Tag("Test")
            {
                Behavior =
                {
                    Order = 5
                }
            };

            Assert.AreEqual(5, tag.Behavior.Order);

            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagBehavior_Show_SetAndGet()
        {
            var tag = new Tag("Test")
            {
                Behavior =
                {
                    Show = true
                }
            };

            Assert.IsTrue(tag.Behavior.Show);

            yield return null;
        }
    }
}

#endif // OVRPLUGIN_TESTING
#endif // OVR_INTERNAL_CODE
