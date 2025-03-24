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

using System;
using NUnit.Framework;

#if OVRPLUGIN_TESTING

namespace Meta.XR.BuildingBlocks.Editor
{
internal class SessionQueueTests : OVRPluginEditModeTest
{
    private const string QueueKey = "SessionQueueTests.QueueKey";

    [Serializable]
    private struct DummyStruct
    {
        public int number;
    }

    [SetUp]
    public void SetUp()
    {
        SessionQueue.Clear(QueueKey);
    }

    [TearDown]
    public void TearDown()
    {
        SessionQueue.Clear(QueueKey);
    }

    [Test, Timeout(DefaultTimeoutMs)]
    public void TestEnqueue()
    {
        Assert.AreEqual(0, SessionQueue.Count<DummyStruct>(QueueKey));
        SessionQueue.Enqueue(new DummyStruct
        {
            number = 42
        }, QueueKey);
        Assert.AreEqual(1, SessionQueue.Count<DummyStruct>(QueueKey));
    }

    [Test, Timeout(DefaultTimeoutMs)]
    public void TestDequeue()
    {
        var dummyObj = new DummyStruct
        {
            number = 42
        };
        SessionQueue.Enqueue(dummyObj, QueueKey);
        var dequeueObj = SessionQueue.Dequeue<DummyStruct>(QueueKey);
        Assert.AreEqual(0, SessionQueue.Count<DummyStruct>(QueueKey));
        Assert.IsTrue(dequeueObj.HasValue);
        Assert.AreEqual(dummyObj.number, dequeueObj.Value.number);
    }

    [Test, Timeout(DefaultTimeoutMs)]
    public void TestDequeueEmpty()
    {
        var dequeueObj = SessionQueue.Dequeue<DummyStruct>(QueueKey);
        Assert.AreEqual(0, SessionQueue.Count<DummyStruct>(QueueKey));
        Assert.IsFalse(dequeueObj.HasValue);
    }
}

}

#endif // OVRPLUGIN_TESTING
