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
using System.Runtime.InteropServices;
using NUnit.Framework;

[TestFixture]
internal class OVRPluginGuidConversionTest
{
    [Test]
    public void TestEquality()
    {
        Guid testGuid = new Guid("35918bc9-196d-40ea-9779-889d79b753f0");
        string expectedUuid = "c98b9135-6d19-ea40-9779-889d79b753f0";
        string actualUuid = OVRPlugin.GuidToUuidString(testGuid);
        Assert.AreEqual(actualUuid, expectedUuid);
    }
}

#endif // OVRPLUGIN_TESTING
