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

using System.Collections;
using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;

namespace Meta.XR.Editor.Tags.Tests
{
    internal class TagArrayDrawerTests : OVRPluginEditModeTest
    {
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

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagArrayDrawer_GetTagOptions()
        {
            var options = TagArrayDrawer.GetTagOptions();
            Assert.AreEqual(Tag.Registry.Count() + 1, options.Length);
            var index = 0;
            foreach (var tag in Tag.Registry)
            {
                Assert.AreEqual(tag.Name, options[index++]);
            }

            Assert.AreEqual(TagArrayDrawer.AddNewTag, options[index]);
            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagArrayDrawer_GetCurrentMask()
        {
            var gameObject = new GameObject();
            var component = gameObject.AddComponent<ITaggableTests.TaggableTest>();
            var tagable = component as ITaggable;
            var tags = new[] { new Tag("Tag1"), new Tag("Tag2") };
            var tagArray = new TagArray();
            tagable.Tags.SetArray(tags.Clone() as Tag[]);

            var serializedObject = new SerializedObject(component);

            var tagArrayProperty = serializedObject.FindProperty(tagable.SerializedPropertyName);
            var mask = TagArrayDrawer.GetCurrentMask(tagArrayProperty, TagArrayDrawer.GetTagOptions());

            Assert.AreEqual(0b11, mask);

            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagArrayDrawer_UpdateTagList()
        {
            var gameObject = new GameObject();
            var component = gameObject.AddComponent<ITaggableTests.TaggableTest>();
            var tagable = component as ITaggable;
            var tags = new[] { new Tag("Tag1"), new Tag("Tag2") };
            var expectedTags = new [] { new Tag("Tag1"), new Tag("Tag3") };
            tagable.Tags.SetArray(tags.Clone() as Tag[]);

            var serializedObject = new SerializedObject(component);

            var tagArrayProperty = serializedObject.FindProperty(tagable.SerializedPropertyName);
            TagArrayDrawer.UpdateTagList(tagArrayProperty, 0b101, TagArrayDrawer.GetTagOptions());

            Assert.IsTrue(tagable.Tags.SequenceEqual(expectedTags));

            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagArrayDrawer_HasTag()
        {
            var gameObject = new GameObject();
            var component = gameObject.AddComponent<ITaggableTests.TaggableTest>();
            var tagable = component as ITaggable;
            var tag1 = new Tag("Tag1");
            var tag2 = new Tag("Tag2");
            var tag3 = new Tag("Tag3");
            var tags = new[] { new Tag("Tag1"), new Tag("Tag2")};
            tagable.Tags.SetArray(tags.Clone() as Tag[]);

            var serializedObject = new SerializedObject(component);

            var tagArrayProperty = serializedObject.FindProperty(tagable.SerializedPropertyName);
            Assert.IsTrue(TagArrayDrawer.HasTag(tagArrayProperty, tag1));
            Assert.IsTrue(TagArrayDrawer.HasTag(tagArrayProperty, tag2));
            Assert.IsFalse(TagArrayDrawer.HasTag(tagArrayProperty, tag3));

            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagArrayDrawer_AddTag()
        {
            var gameObject = new GameObject();
            var component = gameObject.AddComponent<ITaggableTests.TaggableTest>();
            var tagable = component as ITaggable;
            var tag1 = new Tag("Tag1");
            var tag2 = new Tag("Tag2");
            var tags = new[] { new Tag("Tag1")};
            var expectedTags = new [] { new Tag("Tag1"), new Tag("Tag2") };
            tagable.Tags.SetArray(tags.Clone() as Tag[]);

            var serializedObject = new SerializedObject(component);

            var tagArrayProperty = serializedObject.FindProperty(tagable.SerializedPropertyName);
            TagArrayDrawer.AddTag(tagArrayProperty, tag2);

            Assert.IsTrue(tagable.Tags.SequenceEqual(expectedTags));

            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagArrayDrawer_AddTagMultipleTimes()
        {
            var gameObject = new GameObject();
            var component = gameObject.AddComponent<ITaggableTests.TaggableTest>();
            var tagable = component as ITaggable;
            var tag1 = new Tag("Tag1");
            var tag2 = new Tag("Tag2");
            var tags = new[] { new Tag("Tag1")};
            var expectedTags = new [] { new Tag("Tag1"), new Tag("Tag2") };
            tagable.Tags.SetArray(tags.Clone() as Tag[]);

            var serializedObject = new SerializedObject(component);

            var tagArrayProperty = serializedObject.FindProperty(tagable.SerializedPropertyName);
            TagArrayDrawer.AddTag(tagArrayProperty, tag2);

            TagArrayDrawer.AddTag(tagArrayProperty, tag2);

            Assert.IsTrue(tagable.Tags.SequenceEqual(expectedTags));

            yield return null;
        }
    }
}

#endif // OVRPLUGIN_TESTING
#endif // OVR_INTERNAL_CODE
