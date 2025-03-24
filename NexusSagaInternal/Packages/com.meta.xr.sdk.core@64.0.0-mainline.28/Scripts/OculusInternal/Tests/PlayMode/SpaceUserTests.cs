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
using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine.TestTools;
using Assert = UnityEngine.Assertions.Assert;
using Result = OVRPlugin.Result;

internal class SpaceUserTests : OVRPluginPlayModeTest
{
    private class Mock : OVRPlugin.OVRP_1_79_0_TEST
    {
        private Dictionary<ulong, ulong> _handleToId = new Dictionary<ulong, ulong>();

        private ulong _nextSpaceUserHandle = 1;

        public override Result ovrp_GetSpaceUserId(in ulong spaceUserHandle,
            out UInt64 spaceUserId)
            => _handleToId.TryGetValue(spaceUserHandle, out spaceUserId)
                ? Result.Success
                : Result.Failure;

        public override Result ovrp_CreateSpaceUser(in ulong spaceUserId, out ulong spaceUserHandle)
        {
            _handleToId.Add(_nextSpaceUserHandle, spaceUserId);
            spaceUserHandle = _nextSpaceUserHandle++;
            return Result.Success;
        }

        public override Result ovrp_DestroySpaceUser(in ulong userHandle)
        {
            _handleToId.Remove(userHandle);
            return Result.Success;
        }
    }

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        OVRPlugin.OVRP_1_79_0.mockObj = new Mock();
    }

    [OneTimeTearDown]
    public void OneTimeTearDown()
        => OVRPlugin.OVRP_1_79_0.mockObj = new OVRPlugin.OVRP_1_79_0_TEST();

    [Test]
    public void CanRoundtripUserId()
    {
        const ulong userId = 0xdeadbeef;
        var user = new OVRSpaceUser(userId);
        Assert.IsTrue(user.Valid);
        Assert.AreEqual(userId, user.Id);
    }

    [Test]
    public void DisposeDestroysSpaceUser()
    {
        var user = new OVRSpaceUser(123);
        Assert.IsTrue(user.Valid);

        user.Dispose();
        Assert.IsFalse(user.Valid);
        Assert.AreEqual(0, user.Id);
    }

    [Test]
    public void SpaceUserCanBeUsedInUsingStatement()
    {
        var user = new OVRSpaceUser(456);
        using (user)
        {
            Assert.IsTrue(user.Valid);
            Assert.AreEqual(456, user.Id);
            // Goes out of scope using a using statement
        }

        Assert.IsFalse(user.Valid);
        Assert.AreEqual(0, user.Id);
    }

    [Test]
    public void CanCreateMultipleUsers()
    {
        var user1 = new OVRSpaceUser(123);
        var user2 = new OVRSpaceUser(456);
        Assert.AreEqual(123, user1.Id);
        Assert.AreEqual(456, user2.Id);

        user2.Dispose();
        Assert.IsFalse(user2.Valid);
        var user3 = new OVRSpaceUser(789);
        user1.Dispose();
        Assert.IsFalse(user1.Valid);
        Assert.AreEqual(789, user3.Id);
        Assert.IsTrue(user3.Valid);
    }
}

#endif // OVRPLUGIN_TESTING
