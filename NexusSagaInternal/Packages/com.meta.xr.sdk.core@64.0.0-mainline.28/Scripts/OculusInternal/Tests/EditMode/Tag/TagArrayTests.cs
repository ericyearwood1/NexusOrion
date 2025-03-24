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
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace Meta.XR.Editor.Tags.Tests
{
    internal class TagArrayTests : OVRPluginEditModeTest
    {
        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagArray_IEnumerableContract()
        {
            var tagArray = new TagArray();
            Assert.AreEqual(0, tagArray.Count());

            var tags = new[] { new Tag("Tag1"), new Tag("Tag2"), new Tag("Tag3") };
            tagArray.SetArray(tags.Clone() as Tag[]);

            Assert.AreEqual(tags.Length, tagArray.Count());

            var tagList = new List<Tag>();
            foreach (var tag in tagArray)
            {
                tagList.Add(tag);
            }

            Assert.AreEqual(tags.Length, tagList.Count);

            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagArray_SerializableContract()
        {
            var tags = new[] { new Tag("Tag1"), new Tag("Tag2"), new Tag("Tag3") };
            var tagArray = new TagArray();
            tagArray.SetArray(tags.Clone() as Tag[]);

            var serializedArray = JsonUtility.ToJson(tagArray);
            var deserializedTagArray = JsonUtility.FromJson<TagArray>(serializedArray);
            Assert.IsTrue(tagArray.SequenceEqual(deserializedTagArray));

            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagArray_Add_AddsNewTag_ToArray()
        {
            var tag1 = new Tag("Tag1");
            var tagArray = new TagArray();
            tagArray.Add(tag1);
            Assert.IsTrue(tagArray.Contains(tag1));

            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagArray_Remove_RemovesTag()
        {
            var tag1 = new Tag("Tag1");
            var tag2 = new Tag("Tag2");
            var tagArray = new TagArray
            {
                tag1,
                tag2
            };
            tagArray.Remove(tag1);
            Assert.IsFalse(tagArray.Contains(tag1));
            Assert.IsTrue(tagArray.Contains(tag2));

            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator TagArray_Add_DoesNotAddDuplicateTag()
        {
            var tag1 = new Tag("Tag1");
            var tagArray = new TagArray();
            tagArray.Add(tag1);
            var originalCount = tagArray.Count();
            tagArray.Add(tag1);
            Assert.AreEqual(originalCount, tagArray.Count());

            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator SetArray_SetsArrayContent()
        {
            var tag1 = new Tag("Tag1");
            var tag2 = new Tag("Tag2");
            var tag3 = new Tag("Tag3");
            var tags = new[] { new Tag("Tag1"), new Tag("Tag2"), new Tag("Tag3") };
            var tagArray = new TagArray();
            tagArray.SetArray(tags);
            Assert.IsTrue(tagArray.SequenceEqual(tags));

            yield return null;
        }

        [UnityTest, Timeout(DefaultTimeoutMs)]
        public IEnumerator Clear_RemovesAllTags()
        {
            var tagArray = new TagArray();
            var tag1 = new Tag("Tag1");
            tagArray.Add(tag1);
            tagArray.Clear();
            Assert.AreEqual(0, tagArray.Count());

            yield return null;
        }

        [Test]
        public void SortedTags_SortsTagsCorrectly()
        {
            var tag1 = new Tag("Tag1");
            var tag2 = new Tag("Tag2");
            var tag3 = new Tag("Tag3");
            var tags = new[] { new Tag("Tag1"), new Tag("Tag2"), new Tag("Tag3") }.ToList();
            var tagArray = new TagArray();
            tagArray.SetArray(tags);

            var sorted = tagArray.SortedTags;
            tags.Sort(Tag.Sorter);
            Assert.IsTrue(tagArray.SequenceEqual(tags));
        }

    }
}

#endif // OVRPLUGIN_TESTING
#endif // OVR_INTERNAL_CODE
