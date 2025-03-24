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

using System.Collections;
using System.Linq;
using NUnit.Framework;
using UnityEngine.TestTools;

namespace Meta.XR.Editor.Tags.Tests
{
    internal class TagTests : OVRPluginEditModeTest
    {
        [UnitySetUp]
        public override IEnumerator UnitySetUp()
        {
            yield return base.UnitySetUp();

            Tag.Registry.Clear();
        }

        [UnityTearDown]
        public override IEnumerator UnityTearDown()
        {
            Tag.Registry.Clear();

            yield return base.UnityTearDown();
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator Tag_Equals_ShouldBeEqual()
        {
            var tag1 = new Tag("Test");
            var tag2 = new Tag("Test");
            Assert.IsTrue(tag1.Equals(tag2));
            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator Tag_Equals_ShouldNotBeEqual()
        {
            var tag1 = new Tag("Test1");
            var tag2 = new Tag("Test2");
            Assert.IsFalse(tag1.Equals(tag2));
            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator Tag_ImplicitConversionToString()
        {
            var tag = new Tag("Test");
            string convertedString = tag;
            Assert.AreEqual("Test", convertedString);
            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator Tag_ImplicitConversionFromString()
        {
            Tag tag = "Test";
            Assert.AreEqual("Test", tag.Name);
            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator Tag_Valid_ShouldBeTrue()
        {
            var tag = new Tag("Test");
            Assert.IsTrue(tag.Valid);
            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator Tag_Valid_ShouldBeFalse()
        {
            var tag = new Tag(null);
            Assert.IsFalse(tag.Valid);
            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator Tag_GetHashCode_ShouldBeSame()
        {
            var tag1 = new Tag("Test");
            var tag2 = new Tag("Test");
            Assert.AreEqual(tag1.GetHashCode(), tag2.GetHashCode());
            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator Tag_Registry_ShouldContainTag()
        {
            var tag = new Tag("Test");
            Assert.IsTrue(Tag.Registry.Contains(tag));
            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator Tag_Registry_ShouldContainAllDifferent()
        {
            var tag1 = new Tag("Test1");
            var tag2 = new Tag("Test2");
            Assert.AreEqual(Tag.Registry.Count(), 2);
            Assert.IsTrue(Tag.Registry.Contains(tag1));
            Assert.IsTrue(Tag.Registry.Contains(tag2));
            Tag tag3 = "Test1";
            Assert.AreEqual(Tag.Registry.Count(), 2);
            Assert.IsTrue(Tag.Registry.Contains(tag3));
            yield return null;
        }
    }
}

#endif // OVRPLUGIN_TESTING
#endif // OVR_INTERNAL_CODE
