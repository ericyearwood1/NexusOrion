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
using System.Diagnostics;

/// <summary>
/// Helper static class that contains custom assertion methods for tests in the Oculus SDKs.
/// </summary>
internal static class OVRAssert
{
    public class AssertionException : Exception
    {
        public AssertionException(string message) : base(message)
        {
        }
    }

    /// <summary>
    /// Asserts that the given code has zero heap allocations.
    /// </summary>
    /// <param name="code">The code to be tested for zero heap allocations.</param>
    /// <param name="nRuns">The number of times <see cref="code"/> will be run.
    /// We run it multiple times because GC.GetTotalMemory gives us an approximation only, running it few times
    /// may lead to allocations going undetected.
    /// Defaults to 100</param>
    [Conditional("OVRPLUGIN_TESTING_GC")]
    public static void HasZeroAllocations(Action code, int nRuns = 100)
    {
        var before = GC.GetTotalMemory(false);
        for (var i = 0; i < nRuns; i++)
        {
            code?.Invoke();
        }

        var after = GC.GetTotalMemory(false);
        if (after > before)
        {
            throw new AssertionException($"Expected zero allocations but had {(after - before) / nRuns}");
        }
    }
}

#endif // OVRPLUGIN_TESTING
