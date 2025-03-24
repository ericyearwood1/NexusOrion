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
using Result = OVRPlugin.Result;

class TwoCallMockFixture : OVRPluginPlayModeTest
{
    [Test]
    public unsafe void TwoCallIdiomMockCanBeUsedForStrings()
    {
        const string expectedValue = "TwoCall (ツーコール) Idiom";

        Assert.That(TwoCall(expectedValue, 0, out var count1, null), Is.EqualTo(Result.Success));
        var bytes = stackalloc byte[(int)count1];
        Assert.That(TwoCall(expectedValue, count1, out var count2, bytes), Is.EqualTo(Result.Success));
        Assert.That(count1, Is.EqualTo(count2));
        var actualValue = Marshal.PtrToStringUTF8(new IntPtr(bytes), (int)count2);
        Assert.That(actualValue, Is.EqualTo(expectedValue));
    }

    [Test]
    public unsafe void TwoCallIdiomMockReturnsInvalidParameterWhenDestinationBufferIsNull()
    {
        Assert.That(TwoCall<int>(1, null, 1, out _, null),
            Is.EqualTo(Result.Failure_InvalidParameter));
    }

    [Test]
    public unsafe void TwoCallIdiomMockReturnsInsufficientSize()
    {
        Assert.That(TwoCall<int>(4, null, 3, out _, null),
            Is.EqualTo(Result.Failure_InsufficientSize));
    }

    [Test]
    public unsafe void TwoCallIdiomMockReturnsCorrectArray()
    {
        var values = new[] { 1, 2, 3, 4 };
        fixed (int* src = values)
        {
            var requiredElementCount = (uint)values.Length;
            Assert.That(TwoCall(requiredElementCount, src, 0, out var count1, null),
                Is.EqualTo(Result.Success));
            Assert.That(count1, Is.EqualTo(requiredElementCount));

            var buffer = stackalloc int[(int)count1];
            Assert.That(TwoCall(requiredElementCount, src, count1, out var count2, buffer),
                Is.EqualTo(Result.Success));
            Assert.That(count1, Is.EqualTo(count2));

            for (var i = 0; i < (int)count1; i++)
            {
                Assert.That(buffer[i], Is.EqualTo(values[i]));
            }
        }
    }
}

#endif
