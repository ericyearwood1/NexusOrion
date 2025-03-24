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
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using Random = UnityEngine.Random;

[TestFixture]
internal class OVRTaskEditModeTests
{
    private const int TimeoutInMs = 10000;
    private const int Expected = 3;
    private const int Zero = 0;
    private const ulong DummyUlong = 42;
    private readonly Guid _dummyGuid = Guid.NewGuid();

    [SetUp]
    public void BeforeEach()
    {
        OVRTask.OnEnterPlayMode();
    }

    [TearDown]
    public void CleanupAfterEach()
    {
        OVRTask.OnEnterPlayMode();
    }

    [Test]
    public void TestTaskClear()
    {
        // Creating an Async Task
        var task = OVRTask.FromGuid<int>(_dummyGuid);

        // Testing not completed, because async and not completed
        Assert.IsFalse(task.IsCompleted);

        // Clearing the Store, this will remove the task from the store, it's not pending anymore
        OVRTask<int>.Clear();

        // Testing completed, because doesn't exist anymore. Clear has succeeded.
        Assert.IsTrue(task.IsCompleted);
    }

    [Test]
    public void TestTaskFromQuery()
    {
        // Creating an Async Task
        var task = OVRTask.FromGuid<int>(_dummyGuid);

        // Testing not completed, because async and not completed
        Assert.IsFalse(task.IsCompleted);

        // Completing the task, setting the result
        task.SetResult(Expected);

        // Testing completed, and result
        Assert.IsTrue(task.IsCompleted);
        Assert.AreEqual(Expected, task.GetResult());
    }

    [Test]
    public void TestUlongToGuid()
    {
        // Creating an Async Task
        var task = OVRTask.FromRequest<int>(DummyUlong);

        // Checking Ulong and Guid consistency
        var guid = OVRTask.GetId(DummyUlong);
        var guidTask = OVRTask.GetExisting<int>(guid);
        // Proving Ulong to Guid consistency
        // And task equality
        Assert.AreEqual(guidTask, task);
    }

    [Test]
    public void TestTaskFromResult()
    {
        // Creating an Sync, already completed Task
        var task = OVRTask.FromResult(Expected);

        // Testing completed, and result
        Assert.IsTrue(task.IsCompleted);
        Assert.AreEqual(Expected, task.GetResult());
    }

    private void TestTaskInternalCapture(Action<OVRTask<int>> completeTaskAction)
    {
        var list = OVRObjectPool.Get<List<int>>();
        var task = OVRTask.FromGuid<int>(_dummyGuid);
        task.SetInternalData(list);

        Assert.IsTrue(task.TryGetInternalData<List<int>>(out var listCapture));
        Assert.AreEqual(list, listCapture);

        completeTaskAction?.Invoke(task);

        Assert.IsFalse(task.TryGetInternalData<List<int>>(out var listCaptureAfterDispose));
        Assert.AreEqual(default(List<int>), listCaptureAfterDispose);
    }

    [Test]
    public void TestTaskInternalCaptureAndComplete()
    {
        TestTaskInternalCapture(task => task.SetResult(Expected));
    }

    [Test]
    public void TestTaskInternalCaptureAndDispose()
    {
        TestTaskInternalCapture(task => task.Dispose());
    }

    [Test]
    public void TestTaskInternalCaptureAndClear()
    {
        TestTaskInternalCapture(task => OVRTask<int>.Clear());
    }

    [Test]
    public void TestTaskSync()
    {
        // Creating the AsyncTest dummy class
        var test = new AsyncTest(_dummyGuid);

        // Stage 1 : Created, result is 0
        Assert.AreEqual(AsyncTest.Stages.Created, test.Stage);
        Assert.AreEqual(default(int), test.Result);

        // Calling the Process as Sync
        test.ProcessSync(Expected);

        // Stage 3 : Completed, result is Expected
        Assert.AreEqual(AsyncTest.Stages.Completed, test.Stage);
        Assert.AreEqual(Expected, test.Result);
    }

    [UnityTest]
    [Timeout(TimeoutInMs)]
    public IEnumerator TestTaskAsync()
    {
        // Creating the AsyncTest dummy class
        var test = new AsyncTest(_dummyGuid);

        // Stage 1 : Created, result is 0
        Assert.AreEqual(AsyncTest.Stages.Created, test.Stage);
        Assert.AreEqual(default(int), test.Result);

        // Calling the Process as Async
        test.ProcessAsync();

        // Wait one frame
        yield return null;

        // Stage 2 : Requested, but not completed, result is still 0
        Assert.AreEqual(AsyncTest.Stages.Requested, test.Stage);
        Assert.AreEqual(default(int), test.Result);

        // Wait one frame
        yield return null;

        // Complete the request and set the result
        test.CompleteTask(Expected);

        // Stage 3 : Completed, result is Expected
        Assert.AreEqual(AsyncTest.Stages.Completed, test.Stage);
        Assert.AreEqual(Expected, test.Result);
    }

    [UnityTest]
    [Timeout(TimeoutInMs)]
    public IEnumerator TestTaskWithCompletionDelegate()
    {
        // Creating the AsyncTest dummy class
        var test = new AsyncTest(_dummyGuid);

        // Stage 1 : Created, result is 0
        Assert.AreEqual(AsyncTest.Stages.Created, test.Stage);
        Assert.AreEqual(default(int), test.Result);

        // Calling the Process as Async
        test.ProcessWithDelegate();

        // Wait one frame
        yield return null;

        // Stage 2 : Requested, but not completed, result is still 0
        Assert.AreEqual(AsyncTest.Stages.Requested, test.Stage);
        Assert.AreEqual(default(int), test.Result);

        // Wait one frame
        yield return null;

        // Complete the request and set the result
        test.CompleteTask(Expected);

        // Stage 3 : Completed, result is Expected
        Assert.AreEqual(AsyncTest.Stages.Completed, test.Stage);
        Assert.AreEqual(Expected, test.Result);
    }

    [Test]
    public void TestTaskAlreadyCompletedWithCompletionDelegate()
    {
        // Creating the AsyncTest dummy class
        var test = new AsyncTest(_dummyGuid);

        test.ProcessAndCompleteWithDelegate(Expected);

        // Stage 3 (Already!) : Completed, result is Expected
        Assert.AreEqual(AsyncTest.Stages.Completed, test.Stage);
        Assert.AreEqual(Expected, test.Result);
    }

    [Test]
    public void AwaitTaskFollowedByContinueWithThrows()
    {
        var task = OVRTask.FromGuid<int>(Guid.NewGuid());

        // Trigger a continuation
        async void EntryPoint() => await task;
        EntryPoint();

        // This should throw because we are already awaiting
        Assert.Throws<InvalidOperationException>(() => task.ContinueWith(_ => { }));
    }

    [Test]
    public void ContinueWithFollowedByAwaitThrows()
    {
        var task = OVRTask.FromGuid<int>(Guid.NewGuid());

        task.ContinueWith(_ => { });

        int? result = null;
        async void StartAwaiting() => result = await task;

        // This should throw because we are already using a continue with
        StartAwaiting();

        task.SetResult(42);

        // The result should not have completed because await threw an exception
        Assert.That(result.HasValue, Is.False);
    }

    [UnityTest]
    [Timeout(TimeoutInMs)]
    public IEnumerator TestTaskWithCaptureAsync()
    {
        // Creating the AsyncTest dummy class
        var test = new AsyncTest(_dummyGuid);

        // Stage 1 : Created, result is 0
        Assert.AreEqual(AsyncTest.Stages.Created, test.Stage);
        Assert.AreEqual(default(int), test.Result);

        // Calling the Process as Async
        test.ProcessWithCapture(Expected);

        // Wait one frame
        yield return null;

        // Stage 2 : Requested, but not completed, result is still 0
        Assert.AreEqual(AsyncTest.Stages.Requested, test.Stage);
        Assert.AreEqual(default(int), test.Result);

        // Wait one frame
        yield return null;

        var task = test.GetTask();
        Assert.IsTrue(task.TryGetInternalData<int>(out var captureOutput));
        Assert.AreEqual(Expected, captureOutput);

        // Complete the request and set the result
        test.CompleteTask(captureOutput);

        // Stage 3 : Completed, result is Expected
        Assert.AreEqual(AsyncTest.Stages.Completed, test.Stage);
        Assert.AreEqual(captureOutput, test.Result);
    }

    private class AsyncTest
    {
        public enum Stages
        {
            Created,
            Requested,
            Completed
        }

        public Stages Stage { get; private set; }
        public int Result { get; private set; }
        private Guid TaskId { get; }

        public AsyncTest(Guid taskId)
        {
            Stage = Stages.Created;
            Result = default;
            TaskId = taskId;
        }

        private OVRTask<int> CreateTask(bool sync, int expected = 0)
        {
            return sync ? OVRTask.FromResult(Expected) : OVRTask.FromGuid<int>(TaskId);
        }

        public void CompleteTask(int expected)
        {
            OVRTask.GetExisting<int>(TaskId).SetResult(expected);
        }

        public OVRTask<int> GetTask()
        {
            return OVRTask.GetExisting<int>(TaskId);
        }

        public async void ProcessSync(int expected)
        {
            Stage = Stages.Requested;
            // Even though we're awaiting, the FromResult should return immediately
            // This is tested in TestTaskSync
            Result = await CreateTask(true, expected);
            Stage = Stages.Completed;
        }

        public async void ProcessAsync()
        {
            Stage = Stages.Requested;
            // This Request will need to wait until its result is set
            // This is tested in TestTaskAsync
            Result = await CreateTask(false);
            Stage = Stages.Completed;
        }

        public void ProcessWithContinuation()
        {
            Stage = Stages.Requested;
            ((INotifyCompletion)CreateTask(false).GetAwaiter()).OnCompleted(() => { });
        }

        public void ProcessWithDelegate()
        {
            Stage = Stages.Requested;
            // This Request will need to wait until its result is set
            // This is tested in TestTaskAsync
            CreateTask(false).ContinueWith(OnCompleted);
        }

        public void ProcessAndCompleteWithDelegate(int expected)
        {
            Stage = Stages.Requested;
            var task = CreateTask(false);
            CompleteTask(expected);
            task.ContinueWith(OnCompleted);
        }

        public async void ProcessWithCapture<T>(T capture)
        {
            Stage = Stages.Requested;
            // This Request will need to wait until its result is set
            // This is tested in TestTaskAsync
            var task = CreateTask(false);
            task.SetInternalData(capture);
            Result = await task;
            Stage = Stages.Completed;
        }

        private void OnCompleted(int result)
        {
            Result = result;
            Stage = Stages.Completed;
        }
    }

    [Test]
    public void DelegateCanCaptureAdditionalDataWhenAddedAfterCompletion()
    {
        bool delegateInvoked = false;
        float expectedCapture = 123;
        int expectedResult = 42;
        OVRTask.FromResult(expectedResult).ContinueWith((actualResult, actualCapture) =>
        {
            delegateInvoked = true;
            Assert.AreEqual(expectedCapture, actualCapture);
            Assert.AreEqual(expectedResult, actualResult);
        }, expectedCapture);
        Assert.IsTrue(delegateInvoked);
    }

    [Test]
    public void DelegateCanCaptureAdditionalDataWhenAddedBeforeCompletion()
    {
        bool delegateInvoked = false;
        float expectedCapture = 123;
        int expectedResult = 42;
        var task = OVRTask.FromGuid<int>(Guid.NewGuid());
        task.ContinueWith((actualResult, actualCapture) =>
        {
            delegateInvoked = true;
            Assert.AreEqual(expectedCapture, actualCapture);
            Assert.AreEqual(expectedResult, actualResult);
        }, expectedCapture);
        task.SetResult(expectedResult);
        Assert.IsTrue(delegateInvoked);
    }

    [Test]
    public void WithIncrementalResults()
    {
        Random.InitState(174339311);

        var taskId = Guid.NewGuid();
        var actual = new List<int>();
        var task = OVRTask.FromGuid<bool>(taskId);
        var expected = Enumerable.Range(0, 10)
            .Select(_ => Random.Range(0, int.MaxValue))
            .ToArray();

        task.SetIncrementalResultCallback<int>(value => actual.Add(value));

        foreach (var value in expected)
        {
            OVRTask.GetExisting<bool>(taskId).NotifyIncrementalResult(value);
        }

        OVRTask.SetResult(taskId, true);

        Assert.That(actual, Is.EqualTo(expected));
    }

    struct TaskData
    {
        public int A;
        public float B;
        public char C;
    }

    [Test]
    public void AllTaskDataIsResetOnEnterPlayMode()
    {
        var task1 = OVRTask.FromRequest<int>(1234);
        var task2 = OVRTask.FromRequest<int>(5678);
        task2.SetInternalData(42);
        var task3 = OVRTask.FromResult<float>(3.1415f);
        var guid = Guid.NewGuid();
        var task4 = OVRTask.FromGuid<TaskData>(guid);
        var continueWithCalled = false;
        task4.ContinueWith(_ => continueWithCalled = true);

        Assert.That(task1.IsCompleted, Is.False);
        Assert.That(task2.TryGetInternalData<int>(out _), Is.True);

        OVRTask.OnEnterPlayMode();

        Assert.That(task1.IsCompleted, Is.True);
        Assert.That(task2.TryGetInternalData<int>(out _), Is.False);
        Assert.Throws<InvalidOperationException>(() => task3.GetResult());

        // Mark it as complete but nothing should happen
        OVRTask.SetResult(guid, default(TaskData));
        Assert.That(continueWithCalled, Is.False);
    }

    [Test]
    public void RequestIdCanBeReusedAfterEnterPlayMode()
    {
        var continueWithCount = 0;
        var requestId = (ulong)134;
        OVRTask.FromRequest<int>(requestId).ContinueWith(_ => continueWithCount++);

        OVRTask.OnEnterPlayMode();

        // Recreate it after domain reload and set a different callback
        var task = OVRTask.FromRequest<int>(requestId);
        task.ContinueWith(_ => continueWithCount++);
        task.SetResult(42);

        Assert.That(continueWithCount, Is.EqualTo(1));
    }

    [Test]
    public void WhenAllResultsAreInTheCorrectOrder()
    {
        const int count = 10;
        var tasks = Enumerable
            .Range(0, count)
            .Select(index => OVRTask.FromRequest<int>((ulong)index))
            .ToArray();

        var combinedTask = OVRTask.WhenAll(tasks);

        Assert.That(combinedTask.IsPending, Is.True);

        var expectedResults = Enumerable.Range(0, count).Select(_ => Random.Range(0, int.MaxValue)).ToArray();

        // Complete in random order
        var pendingIndices = Enumerable.Range(0, count).Select(index => index).ToList();
        while (pendingIndices.Count > 0)
        {
            Assert.That(combinedTask.IsCompleted, Is.False);

            // Pick a pending task at random
            var i = Random.Range(0, pendingIndices.Count - 1);
            var taskIndex = pendingIndices[i];
            pendingIndices.RemoveAt(i);

            tasks[taskIndex].SetResult(expectedResults[taskIndex]);
        }

        Assert.That(combinedTask.IsCompleted, Is.True);
        Assert.That(combinedTask.GetResult(), Is.EquivalentTo(expectedResults));
    }

    [Test]
    public void WhenAllThrowsIfTaskListIsNull()
    {
        Assert.Throws<ArgumentNullException>(() => OVRTask.WhenAll<int>(null));
    }

    [Test]
    public void WhenAllThrowsIfTaskListOrResultListIsNull()
    {
        var tasks = Array.Empty<OVRTask<int>>();
        var results = new List<int>();
        Assert.Throws<ArgumentNullException>(() => OVRTask.WhenAll(tasks, null));
        Assert.Throws<ArgumentNullException>(() => OVRTask.WhenAll(null, results));
        Assert.Throws<ArgumentNullException>(() => OVRTask.WhenAll<int>(null, null));
    }

    [Test]
    public void WhenAllCompletesWithZeroTasks()
    {
        var taskCompleted = false;
        var inputResultsList = new[] { 1, 2, 3 }.ToList();
        OVRTask.WhenAll(Array.Empty<OVRTask<int>>(), inputResultsList).ContinueWith(results =>
        {
            taskCompleted = true;
            Assert.That(results.Count, Is.EqualTo(0));
        });

        Assert.That(taskCompleted, Is.True);
    }

    struct CompoundTaskResult
    {
        public float Pi;
        public int Answer;
        public string Question;
    }

    [Test]
    public void CanCombineTasksOfDifferentResultTypes()
    {
        var floatRequestId = (ulong)Random.Range(0, int.MaxValue);
        var floatTask = OVRTask.FromRequest<float>(floatRequestId);

        var intRequestId = (ulong)Random.Range(0, int.MaxValue);
        var intTask = OVRTask.FromRequest<int>(intRequestId);

        var expectedResult = new CompoundTaskResult
        {
            Pi = 3f,
            Answer = 42,
            Question = "What is the meaning of life, the universe, and everything?"
        };
        var completedTask = OVRTask.FromResult(expectedResult);

        // Set the result before combining to ensure that will work ok
        floatTask.SetResult(3.14f);

        var taskCompleted = false;
        OVRTask.WhenAll(floatTask, intTask, completedTask).ContinueWith(result =>
        {
            taskCompleted = true;
            Assert.That(result.Item1, Is.EqualTo(3.14f));
            Assert.That(result.Item2, Is.EqualTo(42));
            Assert.That(result.Item3, Is.EqualTo(expectedResult));
        });

        Assert.That(taskCompleted, Is.False);

        // This should be the last task to be completed.
        intTask.SetResult(42);

        Assert.That(taskCompleted, Is.True);
    }

    [Test]
    public void CanQueryHasResultOnADefaultTask()
    {
        Assert.That(new OVRTask<int>().HasResult, Is.False);
    }

    [Test]
    public void HasResultAndTryGetResultReturnCorrectValuesBeforeAndAfterSettingTheResult()
    {
        var task = OVRTask.FromGuid<int>(Guid.NewGuid());

        // Should be false because it is pending
        Assert.That(task.TryGetResult(out _), Is.False);
        Assert.That(task.HasResult, Is.False);

        task.SetResult(42);

        Assert.That(task.HasResult, Is.True);
        Assert.That(task.TryGetResult(out var result), Is.True);
        Assert.That(result, Is.EqualTo(42));
    }

    [Test]
    public void CanBeConvertedToValueTask()
    {
        var guid = Guid.NewGuid();
        var task = OVRTask.FromGuid<int>(guid);
        var valueTask = task.ToValueTask();
        Assert.That(valueTask.IsCompleted, Is.False);

        OVRTask.SetResult(guid, 42);
        Assert.That(valueTask.IsCompleted, Is.True);
        Assert.That(valueTask.Result, Is.EqualTo(42));
    }

    [Test, Timeout(1000)]
    public async Task ToValueTaskCanBeAwaitedAfterCompletion()
    {
        Assert.That(await OVRTask.FromResult(42).ToValueTask(), Is.EqualTo(42));
    }

    [UnityTest, Timeout(1000)]
    public IEnumerator ToValueTaskCanBeAwaitedBeforeCompletion()
    {
        int? result = null;
        async void AwaitResult(ValueTask<int> task) => result = await task;

        var task = OVRTask.FromGuid<int>(Guid.NewGuid());

        // Start awaiting
        AwaitResult(task.ToValueTask());

        // Set the result
        task.SetResult(42);

        // Tick until the task completes (or the test times out)
        while (result == null)
        {
            yield return null;
        }

        // Should have invoked the continuation
        Assert.That(result.HasValue, Is.True);
        Assert.That(result.Value, Is.EqualTo(42));
    }

    [Test]
    public void ToValueTaskThrowsWhenDefaultConstructed()
        => Assert.Throws<InvalidOperationException>(() => default(OVRTask<int>).ToValueTask());

    [Test]
    public void ToValueTaskThrowsWhenCombinedWithContinueWith()
    {
        var task = OVRTask.FromGuid<int>(Guid.NewGuid());
        task.ContinueWith(_ => { });
        Assert.Throws<InvalidOperationException>(() => task.ToValueTask());
    }

    [Test]
    public void ToValueTaskThrowsWhenCombinedWithAwait()
    {
        async void AwaitResult(OVRTask<int> task) => await task;

        var task = OVRTask.FromGuid<int>(Guid.NewGuid());
        AwaitResult(task);
        Assert.Throws<InvalidOperationException>(() => task.ToValueTask());
        task.SetResult(42); // so the awaiter finishes
    }

    [Test]
    public void ContinueWithThrowsIfToValueTaskHasBeenCalled()
    {
        var task = OVRTask.FromGuid<int>(Guid.NewGuid());
        task.ToValueTask();
        Assert.Throws<InvalidOperationException>(() => task.ContinueWith(_ => { }));
    }

    [Test]
    public void CallingToValueTaskTwiceThrows()
    {
        var task = OVRTask.FromGuid<int>(Guid.NewGuid());
        task.ToValueTask();
        Assert.Throws<InvalidOperationException>(() => task.ToValueTask());
    }

    [Test]
    public void CanAwaitMultipleValueTasks()
    {
        int[] results = null;
        async void AwaitAll(IEnumerable<ValueTask<int>> tasks)
        {
            results = await Task.WhenAll(tasks.Select(task => task.AsTask()));
        }

        var count = 10;
        var tasks = Enumerable
            .Range(0, count)
            .Select(_ => OVRTask.FromGuid<int>(Guid.NewGuid()))
            .ToArray();

        var expectedResults = Enumerable
            .Range(0, count)
            .Select(_ => Random.Range(0, int.MaxValue))
            .ToArray();

        AwaitAll(tasks.Select(task => task.ToValueTask()));

        for (var i = 0; i < count; i++)
        {
            tasks[i].SetResult(expectedResults[i]);
        }

        Assert.That(results, Is.EquivalentTo(expectedResults));
    }

#if UNITY_2023_1_OR_NEWER
    [UnityTest, Timeout(1000)]
    public IEnumerator ToAwaitableCanBeAwaitedBeforeCompletion()
    {
        int? result = null;
        async void AwaitResult(Awaitable<int> task) => result = await task;

        var task = OVRTask.FromGuid<int>(Guid.NewGuid());

        // Start awaiting
        AwaitResult(task.ToAwaitable());

        // Set the result
        task.SetResult(42);

        // Tick until the task completes (or the test times out)
        while (result == null)
        {
            yield return null;
        }

        // Should have invoked the continuation
        Assert.That(result.HasValue, Is.True);
        Assert.That(result.Value, Is.EqualTo(42));
    }

    [Test]
    public void ConversionToBothAwaitableAndValueTaskThrows()
    {
        var expectedResult = Random.Range(int.MinValue, int.MaxValue);
        var task = OVRTask.FromGuid<int>(Guid.NewGuid());
        var valueTask = task.ToValueTask();

        // This invalidates the task, so it can't be used again
        Assert.Throws<InvalidOperationException>(() => task.ToAwaitable());

        // Complete the task
        task.SetResult(expectedResult);

        // Ensure ValueTask was not affected
        Assert.That(valueTask.IsCompleted, Is.True);
        Assert.That(valueTask.Result, Is.EqualTo(expectedResult));
    }

    [Test]
    public async Task ContinueWithThrowsWhenUsedAsAwaitable()
    {
        var expectedResult = Random.Range(int.MinValue, int.MaxValue);
        var task = OVRTask.FromGuid<int>(Guid.NewGuid());
        var awaitable = task.ToAwaitable();
        Assert.Throws<InvalidOperationException>(() => task.ContinueWith(_ => { }));
        task.SetResult(expectedResult);
        Assert.That(await awaitable, Is.EqualTo(expectedResult));
    }
#endif

    [UnityTest, Timeout(1000)]
    public IEnumerator CanBeUsedAsTaskLikeObject()
    {
        bool? result = null;
        var task = OVRTask.FromGuid<int>(Guid.NewGuid());

        async OVRTask<bool> AwaitTask()
        {
            await task;
            return true;
        }

        async void EntryPoint() => result = await AwaitTask();

        EntryPoint();

        yield return null;
        task.SetResult(42);
        yield return null;

        Assert.That(result.HasValue, Is.True);
        Assert.That(result.Value, Is.EqualTo(true));
    }

    [UnityTest, Timeout(1000)]
    public IEnumerator OVRTaskRethrowsException()
    {
        var task = OVRTask.FromGuid<int>(Guid.NewGuid());
        object obj = null;

        async OVRTask<int> ComputeAsync()
        {
            var sum = await task + 42;
            obj!.ToString();
            return sum;
        }

        int? result = null;
        Exception exception = null;

        async void EntryPoint()
        {
            try
            {
                result = await ComputeAsync();
            }
            catch (Exception ex)
            {
                exception = ex;
            }
        }

        EntryPoint();

        yield return null;

        // The first task hasn't completed, so no exception thrown yet
        Assert.That(exception, Is.Null);

        // Setting the result should immediately invoke the continuation and throw
        task.SetResult(42);

        Assert.That(exception, Is.TypeOf<NullReferenceException>());
        Assert.That(result.HasValue, Is.False);
    }

    [Test]
    public void ContinueWithThrowsImmediately()
    {
        var task = OVRTask.FromGuid<int>(Guid.NewGuid());
        object obj = null;

        async OVRTask<int> ComputeAsync()
        {
            var sum = await task + 42;
            obj!.ToString();
            return sum;
        }

        int? result = null;

        ComputeAsync().ContinueWith(r => result = r);

        // Completing the first task will invoke the continuation which then immediately throws
        Assert.Throws<NullReferenceException>(() => task.SetResult(42));
        Assert.That(result.HasValue, Is.False);
    }

    [UnityTest, Timeout(1000)]
    public IEnumerator ValueTaskRethrowsException()
    {
        var task = OVRTask.FromGuid<int>(Guid.NewGuid());
        var valueTask = task.ToValueTask();
        object obj = null;

        async OVRTask<int> ComputeAsync()
        {
            var sum = await valueTask + 42;
            obj!.ToString();
            return sum;
        }

        int? result = null;
        Exception exception = null;

        async void EntryPoint()
        {
            try
            {
                result = await ComputeAsync();
            }
            catch (Exception ex)
            {
                exception = ex;
            }
        }

        EntryPoint();

        // The first task hasn't completed, so no exception thrown yet
        Assert.That(exception, Is.Null);

        // Setting the result should eventually invoke the continuation and throw
        task.SetResult(42);

        // The continuation for a ValueType task isn't invoked immediately upon completion (unlike OVRTasks) so
        // wait until we get the exception or timeout.
        while (exception == null)
        {
            yield return null;
        }

        Assert.That(exception, Is.TypeOf<NullReferenceException>());
        Assert.That(result.HasValue, Is.False);
    }

    [Test]
    public void IsFaultedIsTrueAndExceptionCanBeExtracted()
    {
        var task = OVRTask.FromGuid<int>(Guid.NewGuid());
        var additionalAmount = Random.Range(int.MinValue, int.MaxValue);
        int? sum = null;

        async OVRTask<int> ComputeAsync()
        {
            sum = await task + additionalAmount;
            throw new Exception();
        }

        var generatedTask = ComputeAsync();

        Assert.That(generatedTask.IsCompleted, Is.False);
        Assert.That(generatedTask.IsFaulted, Is.False);

        task.SetResult(42);

        Assert.That(generatedTask.IsCompleted, Is.True);
        Assert.That(generatedTask.IsFaulted, Is.True);
        Assert.That(generatedTask.GetException(), Is.TypeOf<Exception>());
        Assert.That(sum.HasValue, Is.True);
        Assert.That(sum.Value, Is.EqualTo(42 + additionalAmount));
    }

    [Test]
    public void CreateThrowsIfTheSameIdIsUsedMultipleTimes()
    {
        var id = Guid.NewGuid();
        var task = OVRTask.Create<int>(id);
        Assert.Throws<ArgumentException>(() => OVRTask.Create<int>(id));
        task.SetResult(42);
    }

    [Test]
    public void CanSetResultById()
    {
        var id = Guid.NewGuid();
        var task = OVRTask.Create<int>(id);

        Assert.That(task.IsCompleted, Is.False);
        Assert.That(task.HasResult, Is.False);

        OVRTask.SetResult(id, 42);

        Assert.That(task.IsCompleted, Is.True);
        Assert.That(task.HasResult, Is.True);
        Assert.That(task.GetResult(), Is.EqualTo(42));
    }

    [Test]
    public void SetResultThrowsWhenAResultIsAlreadySet()
    {
        var id = Guid.NewGuid();
        OVRTask.Create<int>(id);
        OVRTask.SetResult(id, 42);
        Assert.Throws<InvalidOperationException>(() => OVRTask.SetResult(id, 42));
    }
}

#endif // OVRPLUGIN_TESTING
