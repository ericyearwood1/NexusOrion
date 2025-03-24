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
using System.Linq;
using NUnit.Framework;
using Unity.Collections;

public class OVRNonAllocEnumerableTests
{
    Guid[] _guids;
    NativeArray<Guid> _nativeArray;

    [SetUp]
    public void SetUp()
    {
        _guids = Enumerable.Range(0, 10).Select(_ => Guid.NewGuid()).ToArray();
    }

    [TearDown]
    public void TearDown()
    {
        if (_nativeArray.IsCreated)
        {
            _nativeArray.Dispose();
            _nativeArray = default;
        }
    }

    IEnumerable<Guid> EnumerableGuids()
    {
        foreach (var guid in _guids)
        {
            yield return guid;
        }
    }

    [Test]
    public void NonAllocEnumerableDoesNotAllocateWhenItIsAnIReadOnlyList()
    {
        OVRAssert.HasZeroAllocations(() =>
        {
            foreach (var item in _guids.ToNonAlloc())
            {
            }
        });
    }

    [Test]
    public void NonAllocEnumerableDoesNotAllocateWhenItIsAHashSet()
    {
        var set = new HashSet<Guid>(_guids);
        OVRAssert.HasZeroAllocations(() =>
        {
            foreach (var item in set.ToNonAlloc())
            {
            }
        });
    }

    [Test]
    public void NonAllocEnumerableDoesNotAllocateWhenItIsAQueue()
    {
        var queue = new Queue<Guid>(_guids);
        OVRAssert.HasZeroAllocations(() =>
        {
            foreach (var item in queue.ToNonAlloc())
            {
            }
        });
    }

    [Test]
    public void ResultsMatchWhenUnderlyingStorageIsIReadOnlyList()
    {
        Assert.AreEqual(_guids, _guids.ToNonAlloc());

        var index = 0;
        foreach (var item in _guids.ToNonAlloc())
        {
            Assert.AreEqual(_guids[index++], item);
        }
    }

    [Test]
    public void ResultsMatchWhenUnderlyingStorageIsList()
        => Assert.AreEqual(_guids, new List<Guid>(_guids).ToNonAlloc());

    [Test]
    public void ResultsMatchWhenUnderlyingStorageIsArray()
        => Assert.AreEqual(_guids, _guids.ToNonAlloc());

    [Test]
    public void ResultsMatchWhenUnderlyingStorageIsEnumerable()
        => Assert.AreEqual(_guids, EnumerableGuids().ToNonAlloc());

    [Test]
    public void ResultsMatchWhenUnderlyingStorageIsHashSet()
        => Assert.AreEqual(_guids, new HashSet<Guid>(_guids).ToNonAlloc());

    [Test]
    public void ResultsMatchWhenUnderlyingStorageIsQueue()
        => Assert.AreEqual(_guids, new Queue<Guid>(_guids).ToNonAlloc());

    [Test]
    public void CountIsCorrectWhenUnderlyingStorageIsList()
        => Assert.That(_guids.Length, Is.EqualTo(new List<Guid>(_guids).ToNonAlloc().GetCount()));

    [Test]
    public void CountIsCorrectWhenUnderlyingStorageIsArray()
        => Assert.That(_guids.Length, Is.EqualTo(_guids.ToNonAlloc().GetCount()));

    [Test]
    public void CountIsCorrectWhenUnderlyingStorageIsEnumerable()
        => Assert.That(_guids.Length, Is.EqualTo(EnumerableGuids().ToNonAlloc().GetCount()));

    [Test]
    public void CountIsCorrectWhenUnderlyingStorageIsHashSet()
        => Assert.That(_guids.Length, Is.EqualTo(new HashSet<Guid>(_guids).ToNonAlloc().GetCount()));

    [Test]
    public void CountIsCorrectWhenUnderlyingStorageIsQueue()
        => Assert.That(_guids.Length, Is.EqualTo(new Queue<Guid>(_guids).ToNonAlloc().GetCount()));

    [Test]
    public void ThrowsWhenListIsMutatedDuringEnumeration()
    {
        var list = _guids.ToList();
        Assert.Throws<InvalidOperationException>(() =>
        {
            foreach (var item in list.ToNonAlloc())
            {
                list.RemoveAt(0);
            }
        });
    }

    [Test]
    public void EnumeratorCanBeResetWhenUnderlyingStorageIsIReadOnlyList()
    {
        void AssertAreEqual(ref OVREnumerable<Guid>.Enumerator enumerator)
        {
            var index = 0;
            while (enumerator.MoveNext())
            {
                Assert.AreEqual(_guids[index++], enumerator.Current);
            }

            Assert.AreEqual(_guids.Length, index);
        }

        var e = _guids.ToNonAlloc().GetEnumerator();
        AssertAreEqual(ref e);
        e.Reset();
        AssertAreEqual(ref e);
        e.Dispose();
    }

    [Test]
    public void ToNativeArrayWithFixedArray()
    {
        OVRAssert.HasZeroAllocations(() =>
        {
            _nativeArray = _guids.ToNativeArray(Allocator.Temp);
            _nativeArray.Dispose();
        });

        _nativeArray = _guids.ToNativeArray(Allocator.Temp);
        Assert.AreEqual(_guids, _nativeArray);
    }

    [Test]
    public void ToNativeArrayWithList()
    {
        var list = _guids.ToList();
        OVRAssert.HasZeroAllocations(() =>
        {
            _nativeArray = list.ToNativeArray(Allocator.Temp);
            _nativeArray.Dispose();
        });

        _nativeArray = list.ToNativeArray(Allocator.Temp);
        Assert.AreEqual(_guids, _nativeArray);
    }

    class Collection<T> : ICollection<T>
    {
        readonly List<T> _data;
        public Collection(IEnumerable<T> data) => _data = data.ToList();
        public IEnumerator<T> GetEnumerator() => _data.GetEnumerator();
        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

        public void Add(T item) => throw new NotImplementedException();
        public void Clear() => throw new NotImplementedException();
        public bool Contains(T item) => throw new NotImplementedException();
        public void CopyTo(T[] array, int arrayIndex) => throw new NotImplementedException();
        public bool Remove(T item) => throw new NotImplementedException();
        public int Count => _data.Count;
        public bool IsReadOnly => true;
    }

    [Test]
    public void ToNativeArrayWithCollection()
    {
        _nativeArray = new Collection<Guid>(_guids).ToNativeArray(Allocator.Temp);
        Assert.AreEqual(_guids, _nativeArray);
    }

    [Test]
    public void ToNativeArrayWithReadOnlyCollection()
    {
        _nativeArray = _guids.ToList().AsReadOnly().ToNativeArray(Allocator.Temp);
        Assert.AreEqual(_guids, _nativeArray);
    }

    [Test]
    public void ToNativeArrayWithIEnumerable()
    {
        _nativeArray = EnumerableGuids().ToNativeArray(Allocator.Temp);
        Assert.AreEqual(_guids, _nativeArray);
    }

    [Test]
    public void ToNativeArrayWithHashSet()
    {
        var set = new HashSet<Guid>(_guids);
        OVRAssert.HasZeroAllocations(() =>
        {
            _nativeArray = set.ToNativeArray(Allocator.Temp);
            _nativeArray.Dispose();
        });

        _nativeArray = new HashSet<Guid>(_guids).ToNativeArray(Allocator.Temp);
        Assert.AreEqual(_guids, _nativeArray);
    }

    [Test]
    public void ToNativeArrayWithQueue()
    {
        var queue = new Queue<Guid>(_guids);
        OVRAssert.HasZeroAllocations(() =>
        {
            _nativeArray = queue.ToNativeArray(Allocator.Temp);
            _nativeArray.Dispose();
        });

        _nativeArray = new Queue<Guid>(_guids).ToNativeArray(Allocator.Temp);
        Assert.AreEqual(_guids, _nativeArray);
    }

    [Test]
    public void GetCountReturnsZeroForNullEnumerable()
        => Assert.That(((IEnumerable<int>)null).ToNonAlloc().GetCount(), Is.Zero);

    [Test]
    public void CanEnumerateNullEnumerable()
    {
        foreach (var item in ((IEnumerable<int>)null).ToNonAlloc())
        {
            Assert.Fail("The collection should not contain any elements");
        }
    }
}

#endif
