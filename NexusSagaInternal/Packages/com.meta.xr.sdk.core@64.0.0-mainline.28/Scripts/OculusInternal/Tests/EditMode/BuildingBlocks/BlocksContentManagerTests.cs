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

using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

#if OVRPLUGIN_TESTING

namespace Meta.XR.BuildingBlocks.Editor
{
    internal class BlocksContentManagerTests : OVRPluginEditModeTest
    {
        [Test, Timeout(DefaultTimeoutMs)]
        [TestCase(null)]
        [TestCase("")]
        [TestCase("{}")]
        [TestCase("{\"content\": \"string\"}")]
        [TestCase("{\"content\": 44}")]
        public void TestInvalidJsonParsing(string jsonData)
        {
            Assert.Null(BlocksContentManager.ParseJsonData(jsonData));
        }

        [Test, Timeout(DefaultTimeoutMs)]
        public void TestValidJsonParsing()
        {
            var response = new BlocksContentManager.BlockDataResponse()
            {
                content = new[]
                {
                    new BlocksContentManager.BlockData()
                    {
                        blockName = "abc",
                        id = "123"
                    },
                    new BlocksContentManager.BlockData()
                    {
                        blockName = "abc",
                        id = "123"
                    }
                }
            };

            var jsonData = JsonUtility.ToJson(response);
            var parsedData = BlocksContentManager.ParseJsonData(jsonData);
            Assert.NotNull(BlocksContentManager.ParseJsonData(jsonData));

            for (var i = 0; i < response.content.Length; i++)
            {
                Assert.AreEqual(response.content[i].id, parsedData[i].id);
                Assert.AreEqual(response.content[i].blockName, parsedData[i].blockName);
            }
        }

        [Test, Timeout(DefaultTimeoutMs)]
        public void TestContentFiltering()
        {
            var content = new List<BlockBaseData>
            {
                BuildingBlockDataTests.CreateMockBlockData("a"),
                BuildingBlockDataTests.CreateMockBlockData("b"),
                BuildingBlockDataTests.CreateMockBlockData("c"),
            };

            var filter = new[]
            {
                new BlocksContentManager.BlockData()
                {
                    id = content[0].Id
                }
            };

            var filteredContent = BlocksContentManager.FilterBlockWindowContent(content, filter);

            Assert.AreEqual(filter.Length, filteredContent.Count);
            foreach (var blockData in filteredContent)
            {
                Assert.IsTrue(filter.Any(block => block.id == blockData.Id));
            }

            foreach (var blockData in content)
            {
                BuildingBlockDataTests.DeleteBlockData(blockData as BlockData);
            }
        }

        [Test, Timeout(DefaultTimeoutMs)]
        public void TestContentOrdering()
        {
            var content = new List<BlockBaseData>
            {
                BuildingBlockDataTests.CreateMockBlockData("a"),
                BuildingBlockDataTests.CreateMockBlockData("b"),
                BuildingBlockDataTests.CreateMockBlockData("c"),
            };

            var filter = new[]
            {
                new BlocksContentManager.BlockData()
                {
                    id = content[2].Id
                },
                new BlocksContentManager.BlockData()
                {
                    id = content[1].Id
                },
                new BlocksContentManager.BlockData()
                {
                    id = content[0].Id
                },
            };

            var filteredContent = BlocksContentManager.FilterBlockWindowContent(content, filter);

            for (var i = 0; i < filter.Length; i++)
            {
                Assert.AreEqual(filter[i].id, filteredContent[i].Id);
            }

            foreach (var blockData in content)
            {
                BuildingBlockDataTests.DeleteBlockData(blockData as BlockData);
            }
        }

        [Test, Timeout(DefaultTimeoutMs)]
        public void TestContentOverriding()
        {
            const string originalName = "originalName";
            const string overridenName = "overridenName";

            var content = new List<BlockBaseData>
            {
                BuildingBlockDataTests.CreateMockBlockData(originalName),
                BuildingBlockDataTests.CreateMockBlockData(originalName),
            };

            var filter = new[]
            {
                new BlocksContentManager.BlockData()
                {
                    id = content[0].Id
                },
                new BlocksContentManager.BlockData()
                {
                    id = content[1].Id,
                    blockName = overridenName
                },
            };

            var filteredContent = BlocksContentManager.FilterBlockWindowContent(content, filter);

            for (var i = 0; i < filter.Length; i++)
            {
                Assert.AreEqual(string.IsNullOrEmpty(filter[i].blockName) ? originalName : overridenName, (string) filteredContent[i].BlockName);
            }

            foreach (var blockData in content)
            {
                BuildingBlockDataTests.DeleteBlockData(blockData as BlockData);
            }
        }
    }
}

#endif // OVRPLUGIN_TESTING
