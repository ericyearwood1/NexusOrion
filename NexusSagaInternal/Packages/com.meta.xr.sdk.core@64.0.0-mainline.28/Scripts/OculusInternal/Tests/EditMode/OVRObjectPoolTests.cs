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

[TestFixture]
public class OVRObjectPoolTests
{
    [SetUp]
    public void SetUp()
    {
        OVRObjectPool.Clear<List<int>>();
        OVRObjectPool.Clear<Dictionary<int, int>>();
        OVRObjectPool.Clear<Queue<int>>();
        OVRObjectPool.Clear<Stack<int>>();
        OVRObjectPool.Clear<HashSet<int>>();
        OVRObjectPool.Clear<IntStorage>();
    }

    [Test]
    public void TestShouldCreateNewObject()
    {
        var newList = OVRObjectPool.Get<List<int>>();
        Assert.IsNotNull(newList);
        Assert.AreEqual(0, newList.Count);
    }

    [Test]
    public void TestShouldReturnStoredObject()
    {
        var list = new List<int>();
        var hashCode = list.GetHashCode();
        OVRObjectPool.Return(list);

        var poolList = OVRObjectPool.Get<List<int>>();
        Assert.AreEqual(list, poolList);
        Assert.AreEqual(hashCode, poolList.GetHashCode());
    }

    [Test]
    public void TestReturnMultipleTimes()
    {
        var first = new IntStorage(10);
        var second = new IntStorage(20);

        OVRObjectPool.Return(first);
        OVRObjectPool.Return(second);

        Assert.AreEqual(first, OVRObjectPool.Get<IntStorage>());
        Assert.AreEqual(second, OVRObjectPool.Get<IntStorage>());
        Assert.AreEqual(new IntStorage(), OVRObjectPool.Get<IntStorage>());
    }

    [Test]
    public void TestReturnTheSameObjectTwice()
    {
        var first = new IntStorage(10);

        OVRObjectPool.Return(first);
        OVRObjectPool.Return(first);

        Assert.AreEqual(first, OVRObjectPool.Get<IntStorage>());
        Assert.AreEqual(new IntStorage(), OVRObjectPool.Get<IntStorage>());
    }

    [Test]
    public void TestReturnNull()
    {
        IntStorage @null = null;
        OVRObjectPool.Return(@null);
        Assert.AreEqual(new IntStorage(), OVRObjectPool.Get<IntStorage>());
    }

    private static List<int> GetTestList()
    {
        return new List<int> { 1, 2, 3 };
    }

    [Test]
    public void TestShouldClearListOnReturn()
    {
        var testList = GetTestList();
        OVRObjectPool.Return(testList);

        Assert.AreEqual(GetTestList().Capacity, testList.Capacity);
        Assert.AreEqual(0, testList.Count);
    }

    [Test]
    public void ClearsListOnGet()
    {
        var list = new List<int>();
        OVRObjectPool.Return(list);
        list.Add(1);
        Assert.AreEqual(0, OVRObjectPool.List<int>().Count);

        OVRObjectPool.Return(list);
        list.Add(1);
        Assert.AreEqual(0, OVRObjectPool.Get<List<int>>().Count);
    }

    [Test]
    public void ClearsDictionaryOnGet()
    {
        var dict = new Dictionary<int, int>();
        OVRObjectPool.Return(dict);
        dict.Add(1, 2);
        Assert.AreEqual(0, OVRObjectPool.Dictionary<int, int>().Count);

        OVRObjectPool.Return(dict);
        dict.Add(1, 2);
        Assert.AreEqual(0, OVRObjectPool.Get<Dictionary<int, int>>().Count);
    }

    [Test]
    public void ClearsSetOnGet()
    {
        var set = new HashSet<int>();
        OVRObjectPool.Return(set);
        set.Add(1);
        Assert.AreEqual(0, OVRObjectPool.HashSet<int>().Count);
    }

    [Test]
    public void ClearsStackOnGet()
    {
        var stack = new Stack<int>();
        OVRObjectPool.Return(stack);
        stack.Push(1);
        Assert.AreEqual(0, OVRObjectPool.Stack<int>().Count);
    }

    [Test]
    public void ClearsQueueOnGet()
    {
        var queue = new Queue<int>();
        OVRObjectPool.Return(queue);
        queue.Enqueue(1);
        Assert.AreEqual(0, OVRObjectPool.Queue<int>().Count);
    }

    [Test]
    public void ClearsSetOnReturn()
    {
        var item = new HashSet<int> { 1, 2, 3 };
        OVRObjectPool.Return(item);
        Assert.AreEqual(0, item.Count);
    }

    [Test]
    public void ClearsDictionaryOnReturn()
    {
        var item = new Dictionary<int, int>
        {
            { 1, 2 }
        };
        OVRObjectPool.Return(item);
        Assert.AreEqual(0, item.Count);
    }

    [Test]
    public void ClearsStackOnReturn()
    {
        var item = new Stack<int>();
        item.Push(1);
        OVRObjectPool.Return(item);
        Assert.AreEqual(0, item.Count);
    }

    [Test]
    public void ClearsQueueOnReturn()
    {
        var item = new Queue<int>();
        item.Enqueue(1);
        OVRObjectPool.Return(item);
        Assert.AreEqual(0, item.Count);
    }

    [Test]
    public void ClearsListOnScopeExit()
    {
        List<int> list;
        using (new OVRObjectPool.ListScope<int>(out list))
        {
            list.Add(42);
        }

        Assert.That(list.Count, Is.Zero);
    }

    [Test]
    public void ClearsListOnScopeExitWhenUsedAsGenericItem()
    {
        List<int> list;
        using (new OVRObjectPool.ItemScope<List<int>>(out list))
        {
            list.Add(42);
        }

        Assert.That(list.Count, Is.Zero);
    }

    [Test]
    public void ClearsDictionaryOnScopeExit()
    {
        Dictionary<int, int> dictionary;
        using (new OVRObjectPool.DictionaryScope<int, int>(out dictionary))
        {
            dictionary.Add(1, 42);
        }

        Assert.That(dictionary.Count, Is.Zero);
    }

    [Test]
    public void ClearsHashSetOnScopeExit()
    {
        HashSet<int> set;
        using (new OVRObjectPool.HashSetScope<int>(out set))
        {
            set.Add(42);
        }

        Assert.That(set.Count, Is.Zero);
    }

    [Test]
    public void ClearsStackOnScopeExit()
    {
        Stack<int> stack;
        using (new OVRObjectPool.StackScope<int>(out stack))
        {
            stack.Push(42);
        }

        Assert.That(stack.Count, Is.Zero);
    }

    [Test]
    public void ClearsQueueOnScopeExit()
    {
        Queue<int> queue;
        using (new OVRObjectPool.QueueScope<int>(out queue))
        {
            queue.Enqueue(42);
        }

        Assert.That(queue.Count, Is.Zero);
    }

    private class IntStorage
    {
        public int Value { get; }

        public IntStorage()
        {
        }

        public IntStorage(int value)
        {
            Value = value;
        }

        public override bool Equals(object obj)
        {
            //Check for null and compare run-time types.
            if ((obj == null) || GetType() != obj.GetType())
            {
                return false;
            }

            var intStorage = (IntStorage)obj;
            return Value == intStorage.Value;
        }

        protected bool Equals(IntStorage other)
        {
            return Value == other.Value;
        }

        public override int GetHashCode()
        {
            return Value;
        }
    }
}

#endif // OVRPLUGIN_TESTING
