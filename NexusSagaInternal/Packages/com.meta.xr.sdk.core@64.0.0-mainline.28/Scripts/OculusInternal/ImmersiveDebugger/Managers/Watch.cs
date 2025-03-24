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

using Meta.XR.ImmersiveDebugger.Utils;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using UnityEngine;

namespace Meta.XR.ImmersiveDebugger.Manager
{
    public static class WatchUtils
    {
        internal static readonly Dictionary<Type, Type> Types = new Dictionary<Type, Type>();

        static WatchUtils()
        {
            Register<float>((float value, ref string[] valuesContainer) =>
            {
                valuesContainer[0] = value.ToString("0.00", CultureInfo.InvariantCulture);
            }, 1);
            Register<bool>((bool value, ref string[] valuesContainer) =>
            {
                valuesContainer[0] = value ? "True" : "False";
            }, 1);
            Register<Vector3>((Vector3 value, ref string[] valuesContainer) =>
            {
                valuesContainer[0] = value.x.ToString("0.00", CultureInfo.InvariantCulture);
                valuesContainer[1] = value.y.ToString("0.00", CultureInfo.InvariantCulture);
                valuesContainer[2] = value.z.ToString("0.00", CultureInfo.InvariantCulture);
            }, 3);
            Register<Vector2>((Vector2 value, ref string[] valuesContainer) =>
            {
                valuesContainer[0] = value.x.ToString("0.00", CultureInfo.InvariantCulture);
                valuesContainer[1] = value.y.ToString("0.00", CultureInfo.InvariantCulture);
            }, 2);
        }

        public static Watch Create(MemberInfo memberInfo, object instance)
        {
            var type = memberInfo.GetDataType();
            if (!Types.TryGetValue(type, out var createdType))
            {
                createdType = Register(type);
            }

            return Activator.CreateInstance(createdType, memberInfo, instance) as Watch;
        }

        private static Type Register<T>(Watch<T>.ToDisplayStringSignature toDisplayString, int numberOfValues)
        {
            Watch<T>.Setup(toDisplayString, numberOfValues);
            var createdType = typeof(Watch<T>);
            Types.Add(typeof(T), createdType);
            return createdType;
        }

        private static Type Register(Type type)
        {
            var genericType = typeof(Watch<>);
            var createdType = genericType.MakeGenericType(type);
            Types.Add(type, createdType);
            return createdType;
        }
    }

    public abstract class Watch
    {
        public abstract string Value { get; }
        public abstract string[] Values { get; }
        public abstract int NumberOfValues { get; }
    }

    public class Watch<T> : Watch
    {
        public delegate void ToDisplayStringSignature(T value, ref string[] valuesContainer);
        public static ToDisplayStringSignature ToDisplayStringsDelegate { get; private set; } = null;
        public static int NumberOfDisplayStrings { get; private set; } = 1;
        private static string[] _buffer = new string[1];

        public override int NumberOfValues => NumberOfDisplayStrings;

        internal static void ResetBuffer()
        {
            _buffer = new string[NumberOfDisplayStrings];
        }

        public static void Setup(ToDisplayStringSignature del, int numberOfValues)
        {
            ToDisplayStringsDelegate = del;
            NumberOfDisplayStrings = numberOfValues;
            ResetBuffer();
        }

        public static string[] ToDisplayStrings(T value)
        {
            if (ToDisplayStringsDelegate != null)
            {
                ToDisplayStringsDelegate.Invoke(value, ref _buffer);
            }
            else
            {
                _buffer[0] = value.ToString();
            }
            return _buffer;
        }

        private MemberInfo _memberInfo;
        private readonly Func<T> _getter;

        public override string[] Values => ToDisplayStrings(_getter.Invoke());
        public override string Value => Values[0];

        public Watch(MemberInfo memberInfo, object instance)
        {
            _memberInfo = memberInfo;
            _getter = () => (T)memberInfo.GetValue(instance);
        }
    }
}

#endif // OVR_INTERNAL_CODE
