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

using NUnit.Framework;

#if OVRPLUGIN_TESTING

namespace Meta.XR.BuildingBlocks.Editor
{
    internal class OverridableTests : OVRPluginEditModeTest
    {
        [Test]
        public void TestNoOverride()
        {
            const string originalValue = "originalValue";
            var overridable = new Overridable<string>(originalValue);
            Assert.AreEqual(originalValue, overridable.Value);
            Assert.AreEqual(originalValue, (string) overridable);
        }

        [Test]
        public void TestCanOverride()
        {
            const string originalValue = "originalValue";
            var overridable = new Overridable<string>(originalValue);
            const string newValue = "newValue";
            overridable.SetOverride(newValue);
            Assert.AreEqual(newValue, overridable.Value);
            Assert.AreEqual(newValue, (string) overridable);
        }

        [Test]
        public void TestCanRemoveOverride()
        {
            const string originalValue = "originalValue";
            var overridable = new Overridable<string>(originalValue);
            const string newValue = "newValue";
            overridable.SetOverride(newValue);
            overridable.RemoveOverride();

            Assert.AreEqual(originalValue, overridable.Value);
        }

        [Test]
        public void TestClearOverrideOnNull()
        {
            const string originalValue = "originalValue";
            var overridable = new Overridable<string>(originalValue);
            overridable.SetOverride("newValue");
            overridable.SetOverride(null);

            Assert.AreEqual(originalValue, overridable.Value);
        }

    }
}

#endif // OVRPLUGIN_TESTING
