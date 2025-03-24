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

using System.Collections.Generic;
using NUnit.Framework;

public class OVRAssertZeroAllocationsTests
{
    // Test seems to be flaky; see T143613196
#if ENABLE_OVR_FLAKY_TESTS
    [Test]
    public void ShouldDetectAllocations()
    {
        Assert.Throws<OVRAssert.AssertionException>(() =>
        {
            OVRAssert.HasZeroAllocations(() =>
            {
#pragma warning disable CS0219
                // disable unused variable warning
                var list = new List<int>(100);
#pragma warning restore CS0219
            });
        });
    }
#endif

    [Test]
    public void ShouldDetectNoAllocations()
    {
        OVRAssert.HasZeroAllocations(() =>
        {
#pragma warning disable CS0219
            // disable unused variable warning
            var a = 1;
#pragma warning restore CS0219
        });
    }
}

#endif // OVRPLUGIN_TESTING
