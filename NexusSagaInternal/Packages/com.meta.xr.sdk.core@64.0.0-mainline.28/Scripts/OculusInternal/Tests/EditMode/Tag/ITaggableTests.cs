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
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;

namespace Meta.XR.Editor.Tags.Tests
{
    internal class ITaggableTests : OVRPluginEditModeTest
    {
        internal class TaggableTest : MonoBehaviour, ITaggable
        {
            [SerializeField] private TagArray tags = new TagArray();

            string ITaggable.SerializedPropertyName => nameof(tags);

            TagArray ITaggable.Tags => tags;
        }

        [UnitySetUp]
        public override IEnumerator UnitySetUp()
        {
            yield return base.UnitySetUp();

            Tag.Registry.Clear();

            Tag.Registry.Add(new Tag("Tag1"));
            Tag.Registry.Add(new Tag("Tag2"));
            Tag.Registry.Add(new Tag("Tag3"));
        }

        [UnityTearDown]
        public override IEnumerator UnityTearDown()
        {
            Tag.Registry.Clear();

            yield return base.UnityTearDown();
        }

        public static IEnumerable<Type> ITaggableImplementations
        {
            get
            {
                List<Type> types = new List<Type>();

                foreach (Assembly assembly in AppDomain.CurrentDomain.GetAssemblies())
                {
                    types.AddRange(assembly.GetTypes().Where(t => t.GetInterfaces().Contains(typeof(ITaggable))
                                                                  && !t.IsInterface
                                                                  && !t.IsAbstract));
                }

                return types;
            }
        }

        [Test, Timeout(DefaultTimeoutMs), TestCaseSource(nameof(ITaggableImplementations))]
        public void ITaggable_ImplementationContract(Type type)
        {
            var tag1 = new Tag("Tag1");
            var tag2 = new Tag("Tag2");
            var tag3 = new Tag("Tag3");
            var list = new List<Tag>();

            ITaggable instance = null;
            UnityEngine.Object instanceAsObject = null;
            if (typeof(MonoBehaviour).IsAssignableFrom(type))
            {
                var gameObject = new GameObject();
                instanceAsObject = gameObject.AddComponent(type);
                instance = instanceAsObject as ITaggable;
            }
            else if (typeof(ScriptableObject).IsAssignableFrom(type))
            {
                var scriptableObject = ScriptableObject.CreateInstance(type);
                instance = scriptableObject as ITaggable;
                instanceAsObject = scriptableObject;
            }
            var tags = new[] { new Tag("Tag1")};
            instance.Tags.SetArray(tags.Clone() as Tag[]);
            Assert.IsTrue(instance.Tags.Contains(tag1));
            Assert.IsFalse(instance.Tags.Contains(tag2));
            Assert.IsFalse(instance.Tags.Contains(tag3));

            var serializedObject = new SerializedObject(instanceAsObject);
            var tagArrayProperty = serializedObject.FindProperty(instance.SerializedPropertyName);
            TagArrayDrawer.AddTag(tagArrayProperty, tag2);
            Assert.IsTrue(instance.Tags.Contains(tag1));
            Assert.IsTrue(instance.Tags.Contains(tag2));
            Assert.IsFalse(instance.Tags.Contains(tag3));

            list.Add(tag3);
            Assert.IsFalse(instance.Tags.HasAnyTag(list));
            list.Add(tag2);
            Assert.IsTrue(instance.Tags.HasAnyTag(list));
        }
    }
}

#endif // OVRPLUGIN_TESTING
#endif // OVR_INTERNAL_CODE
