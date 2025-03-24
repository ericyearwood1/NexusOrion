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

#if OVR_INTERNAL_CODE
#if OVRPLUGIN_TESTING

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Meta.XR.Editor.Tags;
using NUnit.Framework;
using NUnit.Framework.Interfaces;
using Unity.Collections.LowLevel.Unsafe;
using UnityEditor;
using UnityEngine;

[TestFixture]
internal class OVRResultEditModeTests
{
    private const int ExpectedValue = 42;

    [OVRResultStatus]
    public enum MockStatus
    {
        Success = OVRPlugin.Result.Success,
        OtherSuccess = OVRPlugin.Result.Success_EventUnavailable,
        Failure = OVRPlugin.Result.Failure,
        FailureDataIsInvalid = OVRPlugin.Result.Failure_DataIsInvalid
    }

    public OVRTask<OVRResult<MockStatus>> TestMockStatus()
    {
        return OVRTask.FromResult(OVRResult<MockStatus>.FromSuccess(MockStatus.Success));
    }

    public OVRTask<OVRResult<int, MockStatus>> TestMockStatusWithValue()
    {
        return OVRTask.FromResult(OVRResult<int, MockStatus>.FromSuccess(ExpectedValue, MockStatus.Success));
    }

    public static IEnumerable<Type> OVRResultStatuses
    {
        get
        {
            var types = Enumerable.Empty<Type>();
            foreach (Assembly assembly in AppDomain.CurrentDomain.GetAssemblies())
            {
                var statusesWithAttributes = assembly.GetTypes().Where(t => t.GetCustomAttribute<OVRResultStatus>() != null);
                var tasksResults = assembly.GetTypes().SelectMany(t =>
                        t.GetMethods(BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static |
                                     BindingFlags.NonPublic))
                    .Where(t => t.ReturnType.IsGenericType &&
                                t.ReturnType.GetGenericTypeDefinition() == typeof(OVRTask<>))
                    .Select(t => t.ReturnType.GetGenericArguments()[0]).ToList();
                var statusesFromResults = tasksResults
                    .Where(t => t.IsGenericType && t.GetGenericTypeDefinition() == typeof(OVRResult<>))
                    .Select(t => t.GetGenericArguments()[0]);
                var statusesFromResultsWithValue = tasksResults
                    .Where(t => t.IsGenericType && t.GetGenericTypeDefinition() == typeof(OVRResult<,>))
                    .Select(t => t.GetGenericArguments()[1]);
                types = types.Union(statusesWithAttributes)
                    .Union(statusesFromResults)
                    .Union(statusesFromResultsWithValue);
            }

            return types.Distinct();
        }
    }

    [Test, TestCaseSource(nameof(OVRResultStatuses))]
    public void OVRResult_OVRResultStatusesAreValid(Type type)
    {
        //Forcing Attribute OVRResultStatus
        Assert.IsNotNull(type.GetCustomAttribute<OVRResultStatus>());

        //Forcing int size
        Assert.AreEqual(UnsafeUtility.SizeOf(type), sizeof(int));

        // Forcing Success, Failure and Failure_DataIsInvalid to be defined
        var statuses = Enum.GetValues(type).Cast<int>().ToList();
        Assert.IsTrue(statuses.Contains((int)OVRPlugin.Result.Success));
        Assert.IsTrue(statuses.Contains((int)OVRPlugin.Result.Failure));
        Assert.IsTrue(statuses.Contains((int)OVRPlugin.Result.Failure_DataIsInvalid));

        // Forcing Status to be a subset of Result
        var results = Enum.GetValues(typeof(OVRPlugin.Result)).Cast<int>().ToList();
        Assert.IsTrue(statuses.All((status) => results.Contains(status)));
    }

    [Test]
    public void OVRResult_DefaultIsNotSuccess()
    {
        var result = default(OVRResult<MockStatus>);
        Assert.IsFalse(result.Success);

        var expectedStatus = MockStatus.FailureDataIsInvalid;
        Assert.AreEqual(expectedStatus, result.Status);
    }

    [Test]
    public void OVRResult_From_ValidStatus()
    {
        var expectedStatus = MockStatus.OtherSuccess;
        var result = OVRResult<MockStatus>.From(expectedStatus);
        Assert.That(result.Status, Is.EqualTo(expectedStatus));
    }

    [Test]
    public void OVRResult_Equals_TwoEqualResultsAreEqual()
    {
        var status1 = MockStatus.Success;
        var status2 = MockStatus.OtherSuccess;
        var result1 = OVRResult<MockStatus>.From(status1);
        var result2 = OVRResult<MockStatus>.From(status1);
        Assert.That(result1, Is.EqualTo(result2));
        var result3 = OVRResult<MockStatus>.From(status2);
        Assert.That(result1, Is.Not.EqualTo(result3));
    }

    [Test]
    public void OVRResult_FromSuccess_Success()
    {
        var success = MockStatus.Success;
        var result = OVRResult<MockStatus>.FromSuccess(success);
        Assert.IsTrue(result.Success);
        Assert.That(result.Status, Is.EqualTo(success));
    }

    [Test]
    public void OVRResult_FromSuccess_SuccessOtherSuccess()
    {
        var otherSuccess = MockStatus.OtherSuccess;
        var result = OVRResult<MockStatus>.FromSuccess(otherSuccess);
        Assert.IsTrue(result.Success);
        Assert.That(result.Status, Is.EqualTo(otherSuccess));
    }

    [Test]
    public void OVRResult_FromSuccess_Throws()
    {
        Assert.Throws<ArgumentException>((() => OVRResult<MockStatus>.FromSuccess(MockStatus.Failure)));
    }

    [Test]
    public void OVRResult_FromFailure_Failure()
    {
        var failure = MockStatus.Failure;
        var result = OVRResult<MockStatus>.FromFailure(failure);
        Assert.IsFalse(result.Success);
        Assert.That(result.Status, Is.EqualTo(failure));
    }

    [Test]
    public void OVRResult_FromFailure_OtherFailure()
    {
        var otherFailure = MockStatus.FailureDataIsInvalid;
        var result = OVRResult<MockStatus>.FromFailure(otherFailure);
        Assert.IsFalse(result.Success);
        Assert.That(result.Status, Is.EqualTo(otherFailure));
    }

    [Test]
    public void OVRResult_FromFailure_Throws()
    {
        Assert.Throws<ArgumentException>((() => OVRResult<MockStatus>.FromFailure(MockStatus.Success)));
    }

    [Test]
    public void OVRResultWithValue_DefaultIsNotSuccess()
    {
        var result = default(OVRResult<int, MockStatus>);
        Assert.IsFalse(result.Success);

        var expectedStatus = MockStatus.FailureDataIsInvalid;
        Assert.AreEqual(expectedStatus, result.Status);
        Assert.Throws<InvalidOperationException>(() => { var val = result.Value; });
    }

    [Test]
    public void OVRResultWithValue_FromSuccess_Success()
    {
        var status = MockStatus.Success;
        var result = OVRResult<int, MockStatus>.FromSuccess(ExpectedValue, status);
        Assert.IsTrue(result.Success);
        Assert.That(result.Status, Is.EqualTo(status));

        var hasValue = result.HasValue;
        Assert.IsTrue(hasValue);

        var value = result.Value;
        Assert.AreEqual(ExpectedValue, value);

        var success = result.TryGetValue(out value);
        Assert.IsTrue(success);
        Assert.AreEqual(ExpectedValue, value);
    }

    [Test]
    public void OVRResultWithValue_FromSuccess_SuccessOtherSuccess()
    {
        var status = MockStatus.OtherSuccess;
        var result = OVRResult<int, MockStatus>.FromSuccess(ExpectedValue, status);
        Assert.IsTrue(result.Success);
        Assert.That(result.Status, Is.EqualTo(status));

        var hasValue = result.HasValue;
        Assert.IsTrue(hasValue);

        var value = result.Value;
        Assert.AreEqual(ExpectedValue, value);

        var success = result.TryGetValue(out value);
        Assert.IsTrue(success);
        Assert.AreEqual(ExpectedValue, value);
    }

    [Test]
    public void OVRResultWithValue_FromSuccess_Throws()
    {
        Assert.Throws<ArgumentException>((() => OVRResult<int, MockStatus>.FromSuccess(ExpectedValue, MockStatus.Failure)));
    }

    [Test]
    public void OVRResultWithValue_FromFailure_Failure()
    {
        var status = MockStatus.Failure;
        var result = OVRResult<int, MockStatus>.FromFailure(status);
        Assert.IsFalse(result.Success);
        Assert.That(result.Status, Is.EqualTo(status));

        var hasValue = result.HasValue;
        Assert.IsFalse(hasValue);

        Assert.Throws<InvalidOperationException>((() =>
        {
            var value = result.Value;
        }));

        var success = result.TryGetValue(out var value);
        Assert.IsFalse(success);
        Assert.AreEqual(default(int), value);
    }

    [Test]
    public void OVRResultWithValue_FromFailure_OtherFailure()
    {
        var status = MockStatus.FailureDataIsInvalid;
        var result = OVRResult<int, MockStatus>.FromFailure(status);
        Assert.IsFalse(result.Success);
        Assert.That(result.Status, Is.EqualTo(status));

        var hasValue = result.HasValue;
        Assert.IsFalse(hasValue);

        Assert.Throws<InvalidOperationException>((() =>
        {
            var value = result.Value;
        }));

        var success = result.TryGetValue(out var value);
        Assert.IsFalse(success);
        Assert.AreEqual(default(int), value);
    }

    [Test]
    public void OVRResultWithValue_FromFailure_Throws()
    {
        Assert.Throws<ArgumentException>((() => OVRResult<int, MockStatus>.FromFailure(MockStatus.Success)));
    }

    [Test]
    public void OVRResultWithValue_Equals_TwoEqualResultsAreEqual()
    {
        var status1 = MockStatus.Success;
        var status2 = MockStatus.OtherSuccess;
        var result1 = OVRResult<int, MockStatus>.From(ExpectedValue, status1);
        var result2 = OVRResult<int, MockStatus>.From(ExpectedValue, status1);
        Assert.That(result1, Is.EqualTo(result2));
        var result3 = OVRResult<int, MockStatus>.From(ExpectedValue + 1, status1);
        Assert.That(result1, Is.Not.EqualTo(result3));
        var result4 = OVRResult<int, MockStatus>.From(ExpectedValue, status2);
        Assert.That(result1, Is.Not.EqualTo(result4));
    }
}

#endif // OVRPLUGIN_TESTING
#endif // OVR_INTERNAL_CODE
