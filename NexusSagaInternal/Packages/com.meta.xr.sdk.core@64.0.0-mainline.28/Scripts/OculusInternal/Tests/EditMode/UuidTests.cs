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
using System.Collections.Generic;
using NUnit.Framework;

[TestFixture]
class UuidTests
{
    [Test]
    public void TestEquality()
    {
        var guid1 = Guid.NewGuid();
        var guid2 = Guid.NewGuid();

        Assert.AreEqual(guid1, guid1);
        Assert.AreEqual(default(Guid), default(Guid));
        Assert.AreNotEqual(guid1, default(Guid));
        Assert.AreNotEqual(default(Guid), guid1);
        Assert.AreNotEqual(guid1, guid2);
    }

    [Test]
    public void TestGetHashCode()
    {
        var uuid1 = Guid.NewGuid();
        var uuid2 = Guid.NewGuid();
        Assert.AreEqual(uuid1.GetHashCode(), uuid1.GetHashCode());
        Assert.AreEqual(uuid2.GetHashCode(), uuid2.GetHashCode());
        Assert.AreNotEqual(uuid1.GetHashCode(), uuid2.GetHashCode());
    }

    [Test]
    public void TestHashSet()
    {
        var count = 10;
        var uuids = new Guid[count];
        for (var i = 0; i < count; i++)
        {
            uuids[i] = Guid.NewGuid();
        }

        var hashSet = new HashSet<Guid>();

        // Adding each one should succeed b/c they are all different
        foreach (var uuid in uuids)
        {
            Assert.IsTrue(hashSet.Add(uuid));
        }

        // If we add a second time, they should all fail
        foreach (var uuid in uuids)
        {
            Assert.IsFalse(hashSet.Add(uuid));
        }
    }
}

#endif // OVRPLUGIN_TESTING
