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
using System.Runtime.InteropServices;
using NUnit.Framework;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine.TestTools;

class QplEditModeTests : OVRPluginEditModeTest
{
    [UnitySetUp]
    public override IEnumerator UnitySetUp() => base.UnitySetUp();

    [UnityTearDown]
    public override IEnumerator UnityTearDown() => base.UnityTearDown();

    [Test]
    public unsafe void PointDataMeetsExpectations()
    {
        const int markerId = 1234;

        var longValues = stackalloc long[5];
        for (var i = 0; i < 5; i++)
        {
            longValues[i] = i * 2;
        }

        var boolValues = stackalloc OVRPlugin.Bool[5];
        for (var i = 0; i < 5; i++)
        {
            boolValues[i] = i % 2 == 0 ? OVRPlugin.Bool.True : OVRPlugin.Bool.False;
        }

        var doubleValues = stackalloc double[5];
        for (var i = 0; i < 5; i++)
        {
            doubleValues[i] = Math.Log(i);
        }

        var strings = new NativeArray<IntPtr>(3, Allocator.Temp);
        strings[0] = Marshal.StringToCoTaskMemUTF8("hello world");
        strings[1] = Marshal.StringToCoTaskMemUTF8("I am not a teapot.");
        strings[2] = Marshal.StringToCoTaskMemUTF8("The quick brown fox");
        var stringValues = (byte**)strings.GetUnsafeReadOnlyPtr();

        var stringValue = Marshal.StringToCoTaskMemUTF8("string value");

        try
        {
            using (var builder = OVRPlugin.Qpl.Annotation.Builder.Create())
            {
                builder
                    .Add("string_value", (byte*)stringValue)
                    .Add("string_value_from_string", "some string")
                    .Add("long_value", 42)
                    .Add("double_value", 3.1415)
                    .Add("true_bool", true)
                    .Add("long_array", longValues, 5)
                    .Add("double_array", doubleValues, 5)
                    .Add("bool_array", boolValues, 5)
                    .Add("string_array", stringValues, strings.Length);

                OVRTelemetry
                    .Expect(markerId)
                    .AddAnnotation("string_annotation", "string annotation value")
                    .AddAnnotation("long_annotation", 123)
                    .AddAnnotation("double_annotation", Math.PI)
                    .AddAnnotation("bool_annotation", true)
                    .AddAnnotation("long_array_annotation", longValues, 5)
                    .AddAnnotation("double_array_annotation", doubleValues, 5)
                    .AddAnnotation("bool_array_annotation", boolValues, 5)
                    .AddAnnotation("string_array_annotation", stringValues, strings.Length)
                    .AddPoint("my_point")
                    .AddPoint("my_point_with_data", builder);
            }

            using (var builder = OVRPlugin.Qpl.Annotation.Builder.Create())
            {
                builder
                    .Add("double_array", doubleValues, 5)
                    .Add("bool_array", boolValues, 5)
                    .Add("string_value", (byte*)stringValue)
                    .Add("double_value", 3.1415)
                    .Add("long_array", longValues, 5)
                    .Add("true_bool", true)
                    .Add("string_value_from_string", "some string")
                    .Add("long_value", 42)
                    .Add("string_array", stringValues, strings.Length)
                    .Add("false_bool", false)
                    .Add("point_that_is_not_expected_but_allowed", 123);

                OVRTelemetry
                    .Start(markerId)
                    .AddAnnotation("string_annotation", "string annotation value")
                    .AddAnnotation("long_annotation", 123)
                    .AddAnnotation("string_array_annotation", stringValues, strings.Length)
                    .AddAnnotation("bool_annotation", true)
                    .AddAnnotation("long_array_annotation", longValues, 5)
                    .AddAnnotation("double_annotation", Math.PI)
                    .AddAnnotation("double_array_annotation", doubleValues, 5)
                    .AddAnnotation("bool_array_annotation", boolValues, 5)
                    .AddPoint("my_point")
                    .AddPoint("my_point_with_data", builder)
                    .AddPoint("another_point")
                    .SetResult(OVRPlugin.Qpl.ResultType.Success)
                    .Send();
            }

            OVRTelemetry.TestExpectations();
        }
        finally
        {
            foreach (var str in strings)
            {
                Marshal.FreeCoTaskMem(str);
            }
            strings.Dispose();

            Marshal.FreeCoTaskMem(stringValue);
        }
    }
}

#endif
