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
using System.Linq;
using NUnit.Framework;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEditor;
using UnityEngine;
using Random = UnityEngine.Random;

[TestFixture]
internal class OVRAnchorEditModeTests
{
    private const ulong Handle = 42;
    private readonly Guid _uuid = Guid.NewGuid();

    [Test]
    public void TestOVRAnchorConstructor()
    {
        var anchor = new OVRAnchor(Handle, _uuid);

        // Testing Constructor
        Assert.AreEqual(anchor.Handle, Handle);
        Assert.AreEqual(anchor.Uuid, _uuid);

        // Testing against Null
        Assert.AreNotEqual(anchor, OVRAnchor.Null);

        // Testing IEquatable implementation
        var otherAnchor = new OVRAnchor(Handle, _uuid);
        Assert.AreEqual(anchor, otherAnchor);
        Assert.AreEqual(anchor.GetHashCode(), otherAnchor.GetHashCode());
    }

    [Test]
    public void TestOVRInvalidAnchor()
    {
        var anchor = new OVRAnchor(0, Guid.Empty);

        // Testing against Null
        Assert.AreEqual(anchor, OVRAnchor.Null);
    }

    [Test]
    public void FetchByUuidMatchesSpaceQueryInfo()
    {
        const double timeout = 42.12345;
        var uuids = Enumerable.Range(0, 100).Select(_ => Guid.NewGuid()).ToArray();
        var query = OVRAnchor.GetQueryInfo(uuids, OVRSpace.StorageLocation.Cloud, timeout);

        Assert.That(query.IdInfo.NumIds, Is.EqualTo(uuids.Length));
        Assert.That(query.IdInfo.Ids.Take(uuids.Length), Is.EqualTo(uuids));
        Assert.That(query.ComponentsInfo.NumComponents, Is.Zero);
        Assert.That(query.ActionType, Is.EqualTo(OVRPlugin.SpaceQueryActionType.Load));
        Assert.That(query.QueryType, Is.EqualTo(OVRPlugin.SpaceQueryType.Action));
        Assert.That(query.Location, Is.EqualTo(OVRPlugin.SpaceStorageLocation.Cloud));
        Assert.That(query.MaxQuerySpaces, Is.EqualTo(OVRSpaceQuery.Options.MaxUuidCount));
        Assert.That(query.Timeout, Is.EqualTo(timeout));
    }

    [Test]
    public void FetchByUuidThrowsIfUuidsAreNull()
        => Assert.Throws<ArgumentNullException>(() => OVRAnchor.FetchAnchorsAsync(null, new List<OVRAnchor>()));

    [Test]
    public void FetchByUuidThrowsIfAnchorListIsNull()
        => Assert.Throws<ArgumentNullException>(() => OVRAnchor.FetchAnchorsAsync(Array.Empty<Guid>(), null));

    private void TestImplementation<T>() where T : struct, IOVRAnchorComponent<T>
    {
        var component = default(T);

        // Default is Invalid and Disabled
        Assert.IsTrue(component.IsNull);
        Assert.IsFalse(component.IsEnabled);

        // Give an invalid anchor
        var anchor = default(OVRAnchor);
        component = component.FromAnchor(anchor);

        // Still Invalid and Disabled
        Assert.AreEqual(anchor.Handle, component.Handle);
        Assert.IsTrue(component.IsNull);
        Assert.IsFalse(component.IsEnabled);

        var validAnchor = new OVRAnchor(Handle, _uuid);
        component = component.FromAnchor(validAnchor);

        // Valid Yet Disabled
        Assert.AreEqual(validAnchor.Handle, component.Handle);
        Assert.IsFalse(component.IsNull);
        Assert.IsFalse(component.IsEnabled);
    }

    [Test]
    public void TestOVRSemanticLabels()
    {
        TestImplementation<OVRSemanticLabels>();
    }

    [Test]
    public void TestOVRBounded2D()
    {
        TestImplementation<OVRBounded2D>();
    }

    [Test]
    public void TestOVRBounded3D()
    {
        TestImplementation<OVRBounded3D>();
    }

    [Test]
    public void TestOVRLocatable()
    {
        TestImplementation<OVRLocatable>();
    }

    [Test]
    public void TestOVRRoomLayout()
    {
        TestImplementation<OVRRoomLayout>();
    }

    [Test]
    public void TestOVRAnchorContainer()
    {
        TestImplementation<OVRAnchorContainer>();
    }

#if OVR_INTERNAL_CODE // Room entity info
    [Test]
    public void TestOVRRoomLabel()
    {
        TestImplementation<OVRRoomLabel>();
    }
#endif

#if OVR_INTERNAL_CODE
    [Test]
    public void FetchOptionsThrowsWhenComponentTypeDoesNotImplementIOVRAnchorComponent()
    {
        Assert.Throws<ArgumentException>(() =>
        {
            new OVRAnchor.FetchOptions
            {
                SingleComponentType = typeof(int),
            }.DiscoverSpaces(out _);
        });
    }

    [Test]
    public void FetchOptionsThrowsWhenComponentTypeCollectionContainsNullComponentType()
    {
        Assert.Throws<ArgumentNullException>(() =>
        {
            new OVRAnchor.FetchOptions
            {
                ComponentTypes = new[]
                {
                    typeof(OVRBounded2D), null, typeof(OVRBounded3D)
                }
            }.DiscoverSpaces(out _);
        });
    }

    [Test]
    public void AllComponentTypesAreMappedByDiscoveryFilters()
    {
        var componentTypes = Enum.GetValues(typeof(OVRPlugin.SpaceComponentType))
            .Cast<OVRPlugin.SpaceComponentType>()
            .ToHashSet();

        Assert.That(OVRAnchor._typeMap.Values.ToHashSet(), Is.EquivalentTo(componentTypes));
    }

    [Test]
    public void EraseAsyncThrowsIfBothInputsAreNull()
    {
        Assert.Throws<ArgumentException>(() =>
        {
            OVRAnchor.EraseAsync(null, null);
        });
    }

    [Test]
    public void EraseAsyncReturnsSuccessIfCollectionsAreEmpty()
    {
        var task = OVRAnchor.EraseAsync(Array.Empty<OVRAnchor>(), Array.Empty<Guid>());
        Assert.That(task.IsCompleted, Is.True);
        Assert.That(task.GetResult().Success, Is.True);
    }
#endif // OVR_INTERNAL_CODE
}

static partial class OVRAnchorComponentEditModeTests
{
    [TestFixture]
    abstract class Fixture<T> where T : struct, IOVRAnchorComponent<T>
    {
        static T GetComponent(OVRAnchor anchor) => default(T).FromAnchor(anchor);

        static OVRAnchor CreateRandomAnchor()
            => new OVRAnchor((ulong)Random.Range(0, int.MaxValue), Guid.NewGuid());

        protected (T, T) GetRandomComponents()
            => (GetComponent(CreateRandomAnchor()), GetComponent(CreateRandomAnchor()));

        protected OVRPlugin.SpaceComponentType GetSpaceComponentType() => default(T).Type;

        [Test]
        public void Equals()
        {
            var (componentA, componentB) = GetRandomComponents();
            Assert.AreEqual(componentA, componentA);
            Assert.AreNotEqual(componentA, componentB);
        }

        [Test]
        public void EqualsObject()
        {
            var (componentA, componentB) = GetRandomComponents();
            Assert.IsTrue(componentA.Equals(componentA as object));
            Assert.IsFalse(componentA.Equals(componentB as object));
        }

        [Test]
        public new void GetHashCode()
        {
            var (componentA, componentB) = GetRandomComponents();
            var set = new HashSet<T>
            {
                componentA
            };

            Assert.IsTrue(set.Contains(componentA));
            Assert.IsFalse(set.Contains(componentB));
        }

        [Test]
        public void FetchByComponentTypeMatchesSpaceQueryInfo()
        {
            const int maxResults = 123;
            const double timeout = 42.12345;
            var type = default(T).Type;
            var query = OVRAnchor.GetQueryInfo(type, OVRSpace.StorageLocation.Cloud,
                maxResults, timeout);

            Assert.That(query.IdInfo.NumIds, Is.Zero);
            Assert.That(query.ComponentsInfo.NumComponents, Is.EqualTo(1));
            Assert.That(query.ComponentsInfo.Components[0], Is.EqualTo(type));
            Assert.That(query.ActionType, Is.EqualTo(OVRPlugin.SpaceQueryActionType.Load));
            Assert.That(query.QueryType, Is.EqualTo(OVRPlugin.SpaceQueryType.Action));
            Assert.That(query.Location, Is.EqualTo(OVRPlugin.SpaceStorageLocation.Cloud));
            Assert.That(query.MaxQuerySpaces, Is.EqualTo(maxResults));
            Assert.That(query.Timeout, Is.EqualTo(timeout));
        }

        [Test]
        public void FetchByComponentThrowsIfAnchorListIsNull()
            => Assert.Throws<ArgumentNullException>(() => OVRAnchor.FetchAnchorsAsync<T>(null));
    }
}

#endif // OVRPLUGIN_TESTING
