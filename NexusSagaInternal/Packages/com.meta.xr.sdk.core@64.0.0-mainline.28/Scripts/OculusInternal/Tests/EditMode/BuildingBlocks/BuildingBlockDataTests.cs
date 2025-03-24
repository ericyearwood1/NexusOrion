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
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using Meta.XR.BuildingBlocks;
using Meta.XR.BuildingBlocks.Editor;
using Meta.XR.Editor.Tags;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using Object = UnityEngine.Object;

internal class BuildingBlockDataTests
{
    private const int DefaultTimeoutMs = 10000;

    [UnitySetUp]
    public IEnumerator UnitySetUp()
    {
        while (!BlockBaseData.Registry.Values.Any())
        {
            // In CI the asset loading may take some time
            yield return null;
        }
    }

    [UnityTearDown]
    public IEnumerator UnityTearDown()
    {
        BlockBaseData.Registry.TestOverride.Clear();
        BlockBaseData.Registry.MarkAsDirty();
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestIsCyclic()
    {
        var b1 = CreateMockBlockData("b1");
        var b2 = CreateMockBlockData("b2");
        var b3 = CreateMockBlockData("b3");
        var b4 = CreateMockBlockData("b4");
        var b5 = CreateMockBlockData("b5");

        b1.dependencies = new List<string> { b2.Id, b3.Id };
        b2.dependencies = new List<string> { b3.Id, b4.Id };
        b3.dependencies = new List<string> { b4.Id, b1.Id };
        b4.dependencies = new List<string> { b5.Id };
        b5.dependencies = new List<string>();

        Assert.IsTrue(b1.HasCyclicDependencies());

        DeleteBlockData(b1);
        DeleteBlockData(b2);
        DeleteBlockData(b3);
        DeleteBlockData(b4);
        DeleteBlockData(b5);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestIsNotCyclic()
    {
        var b1 = CreateMockBlockData("b1");
        var b2 = CreateMockBlockData("b2");
        var b3 = CreateMockBlockData("b3");
        var b4 = CreateMockBlockData("b4");
        var b5 = CreateMockBlockData("b5");

        b1.dependencies = new List<string> { b2.Id, b3.Id };
        b2.dependencies = new List<string> { b3.Id, b4.Id };
        b3.dependencies = new List<string> { b4.Id };
        b4.dependencies = new List<string> { b5.Id };
        b5.dependencies = new List<string>();

        Assert.IsFalse(b1.HasCyclicDependencies());

        DeleteBlockData(b1);
        DeleteBlockData(b2);
        DeleteBlockData(b3);
        DeleteBlockData(b4);
        DeleteBlockData(b5);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestDataOverrides()
    {
        var b1 = CreateMockBlockData();

        const string originalName = "originalName";
        b1.blockName = originalName;

        const string originalDescription = "originalDescription";
        b1.description = originalDescription;

        var originalTagArray = new TagArray() { new Tag("tag1"), new Tag("tag2") };
        b1.tags = originalTagArray;

        b1.OnEnable();

        Assert.AreEqual(originalName, (string) b1.BlockName);
        Assert.AreEqual(originalDescription, (string) b1.Description);
        Assert.AreEqual(originalTagArray, b1.OverridableTags.Value);
        Assert.AreEqual(originalTagArray, b1.Tags);

        const string blockNameOverride = "blockNameOverride";
        b1.BlockName.SetOverride(blockNameOverride);

        const string descriptionOverride = "descriptionOverride";
        b1.Description.SetOverride(descriptionOverride);

        var tagArrayOverride = new TagArray() { new Tag("tag1"), new Tag("tag2") };
        b1.OverridableTags.SetOverride(tagArrayOverride);

        Assert.AreEqual(blockNameOverride, (string) b1.BlockName);
        Assert.AreEqual(descriptionOverride, (string) b1.Description);
        Assert.AreEqual(tagArrayOverride,b1.OverridableTags.Value);
        Assert.AreEqual(tagArrayOverride, b1.Tags);
        DeleteBlockData(b1);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestHasNoDuplicateDependencies()
    {
        Assert.IsFalse(new List<string> { "a", "b", "c" }.HasDuplicates());
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestHasDuplicateDependencies()
    {
        Assert.IsTrue(new List<string> { "a", "b", "c", "a" }.HasDuplicates());
        yield return null;
    }

    [UnityTest]
    [TestCase("com.example.department.project", true, ExpectedResult = null)]
    [TestCase("com.example", true, ExpectedResult = null)]
    [TestCase("com..example", false, ExpectedResult = null)]
    [TestCase("com", false, ExpectedResult = null)]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestValidPackageId(string packageId, bool expectedResult)
    {
        Assert.AreEqual(expectedResult, Utils.IsValidPackageId(packageId));
        yield return null;
    }

    public static BlockBaseData[] BlockDataArray => BlockBaseData.Registry.Values.ToArray();

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator ValidateBlockData([ValueSource(nameof(BlockDataArray))] BlockBaseData blockData)
    {
        blockData.Validate();
        yield return null;
    }

    public static BlockBaseData[] PublicBlockDataArray => Utils.PublicBlockData.ToArray();

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestBlocksHaveNoHardcodedId()
    {
        const string projectPath = "Assets";
        var allowListPaths = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
        {
            Path.Combine("Assets", "Oculus", "VR", "Editor", "BuildingBlocks", "BlockDataIds.cs"),
            Path.Combine("Assets", "InteractionOVR", "Editor", "Blocks", "BlockDataIds.cs"),
            Path.Combine("Assets", "MRUtilityKit", "Editor", "BuildingBlocks", "BlockDataIds.cs")
        };

        var files = Directory.GetFiles(projectPath, "*.cs", SearchOption.AllDirectories)
            .Where(f => !allowListPaths.Contains(f));

        var uuidPattern = string.Join("|", Utils.PublicBlockData.Select(blockData => blockData.Id));
        var regex = new Regex(uuidPattern);

        foreach (var file in files)
        {
            var fileContent = File.ReadAllText(file);

            var match = regex.Match(fileContent);
            if (match.Success)
            {
                Assert.Fail($"Hardcoded block Id {match.Value} ({Utils.GetBlockData(match.Value)}) found in {file}. Please use a const string stored in an allow-listed path");
            }
        }

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestBlockDataHasTestSet([ValueSource(nameof(PublicBlockDataArray))] BlockBaseData blockData)
    {
        Assert.IsNotNull(Utils.GetTestClassForBlockData<BuildingBlocksTest>(blockData.Id), $"BlockData {blockData.BlockName.Value} is missing a test class");
        yield return null;
    }

#if OVR_INTERNAL_CODE
    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestBlockDataHasThumbnail([ValueSource(nameof(PublicBlockDataArray))] BlockBaseData blockData)
    {
        if (!blockData.Internal)
        {
            Assert.IsNotNull(blockData.thumbnail);
        }

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestBlockDataHasTags([ValueSource(nameof(PublicBlockDataArray))] BlockBaseData blockData)
    {
        if (!blockData.Internal)
        {
            Assert.Greater(blockData.Tags.Count(tag => tag != Utils.InternalTag && tag != Utils.ExperimentalTag), 0);
        }

        yield return null;
    }
#endif // OVR_INTERNAL_CODE

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestNoDuplicateIds()
    {
        var idSet = new HashSet<string>();

        foreach (var blockData in BlockBaseData.Registry.Values)
        {
            Assert.IsTrue(idSet.Add(blockData.Id), $"{blockData.BlockName} has a duplicate Id");
        }

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CanInstallBlock()
    {
        var b1 = CreateMockBlockData("b1", 10);
        var spawnedObjects = b1.Install();

        Assert.IsTrue(
            spawnedObjects.Select(obj => obj.GetComponent<BuildingBlock>())
                .All(block => block != null
                              && block.BlockId == b1.Id
                              && block.Version == b1.Version
                              && !string.IsNullOrEmpty(block.InstanceId))
        );

        foreach (var spawnedObject in spawnedObjects)
        {
            Object.DestroyImmediate(spawnedObject);
        }

        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CanUpdateBlock()
    {
        const int initialVersion = 10;
        var b1 = CreateMockBlockData("b1", initialVersion);
        b1.Install();

        var blocksOfTypeB1 = Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Where(b => b.BlockId == b1.Id)
            .ToList();

        Assert.AreEqual(1, blocksOfTypeB1.Count);
        var initialBlock = blocksOfTypeB1[0];
        Assert.IsNotNull(initialBlock);
        Assert.IsTrue(initialBlock.BlockId == b1.Id && initialBlock.Version == b1.Version);

        b1.version++;
        b1.UpdateBlockToLatestVersion(initialBlock);

        blocksOfTypeB1 = Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Where(b => b.BlockId == b1.Id)
            .ToList();

        Assert.AreEqual(1, blocksOfTypeB1.Count);
        var updatedBlock = blocksOfTypeB1[0];
        Assert.IsNotNull(updatedBlock);
        Assert.IsTrue(updatedBlock.BlockId == b1.Id && updatedBlock.Version == b1.Version && updatedBlock.Version > initialVersion);

        Object.DestroyImmediate(updatedBlock);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CantUpdateBlockWithoutNewVersionAvailable()
    {
        const int initialVersion = 10;
        var b1 = CreateMockBlockData("b1", initialVersion);
        b1.Install();

        var blocksOfTypeB1 = Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Where(b => b.BlockId == b1.Id)
            .ToList();

        Assert.AreEqual(1, blocksOfTypeB1.Count);
        var initialBlock = blocksOfTypeB1[0];
        Assert.IsNotNull(initialBlock);
        Assert.IsTrue(initialBlock.BlockId == b1.Id && initialBlock.Version == b1.Version);

        Assert.Throws<InvalidOperationException>(() =>
        {
            b1.UpdateBlockToLatestVersion(initialBlock);
        });

        blocksOfTypeB1 = Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Where(b => b.BlockId == b1.Id)
            .ToList();

        Assert.AreEqual(1, blocksOfTypeB1.Count);
        var updatedBlock = blocksOfTypeB1[0];
        Assert.IsNotNull(updatedBlock);
        Assert.AreEqual(initialBlock, updatedBlock);
        Assert.IsTrue(updatedBlock.BlockId == b1.Id && updatedBlock.Version == b1.Version && updatedBlock.Version == initialVersion);

        Object.DestroyImmediate(updatedBlock);
        yield return null;
    }

    internal static BlockData CreateMockBlockData(string blockName = "", int version = 1)
    {
        var blockData = ScriptableObject.CreateInstance<BlockData>();
        blockData.blockName = blockName;
        blockData.version = version;
        blockData.description = "foobar";
        blockData.dependencies = new List<string>();
        blockData.externalBlockDependencies = new List<string>();
        var go = new GameObject
        {
            name = $"[BB_TEST] {blockName}"
        };
        blockData.prefab = go;
        go.SetActive(false); // simulate as if it's a prefab
        BlockBaseData.Registry.TestOverride.Add(blockData);
        BlockBaseData.Registry.MarkAsDirty();

        blockData.OnEnable();

        return blockData;
    }

    internal static void DeleteBlockData(BlockData blockData)
    {
        BlockBaseData.Registry.TestOverride.Remove(blockData);
        BlockBaseData.Registry.MarkAsDirty();
        Object.DestroyImmediate(blockData.Prefab);
        Object.DestroyImmediate(blockData);
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CanInstallDependencies()
    {
        var nStartingBlocks = Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Length;

        var b1 = CreateMockBlockData("b1");
        var b2 = CreateMockBlockData("b2");
        var b3 = CreateMockBlockData("b3");

        b1.dependencies = new List<string> { b2.Id, b3.Id };
        b2.dependencies = new List<string> { b3.Id };

        b1.InstallWithDependencies();

        Assert.AreEqual(3 + nStartingBlocks, Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Length);

        DeleteBlockData(b1);
        DeleteBlockData(b2);
        DeleteBlockData(b3);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator DoesntDuplicateDependencies()
    {
        var nStartingBlocks = Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Length;

        var b1 = CreateMockBlockData("b1");
        var b2 = CreateMockBlockData("b2");
        var b3 = CreateMockBlockData("b3");

        b1.dependencies = new List<string> { b3.Id };
        b2.dependencies = new List<string> { b3.Id };

        b1.InstallWithDependencies();
        b2.InstallWithDependencies();

        Assert.AreEqual(3 + nStartingBlocks, Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Length);

        DeleteBlockData(b1);
        DeleteBlockData(b2);
        DeleteBlockData(b3);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CanInstallMultiple()
    {
        var nStartingBlocks = Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Length;
        var b1 = CreateMockBlockData("b1");

        b1.InstallWithDependencies();
        b1.InstallWithDependencies();

        Assert.AreEqual(2 + nStartingBlocks, Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Length);

        DeleteBlockData(b1);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator CantInstallMultipleSingletons()
    {
        var nStartingBlocks = Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Length;
        var b1 = CreateMockBlockData("b1");
        b1.isSingleton = true;

        b1.InstallWithDependencies();

        Assert.Throws<InvalidOperationException>(() => { b1.InstallWithDependencies(); });

        Assert.AreEqual(1 + nStartingBlocks, Object.FindObjectsByType<BuildingBlock>(FindObjectsSortMode.None).Length);

        DeleteBlockData(b1);
        yield return null;
    }

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator TestHelpers()
    {
        var b1 = CreateMockBlockData("b1");
        var b1Instance = b1.Install()[0];
        var b1Component = b1Instance.GetComponent<BuildingBlock>();

        Assert.That(b1, Is.EqualTo(Utils.GetBlockData(b1.Id)));
        Assert.That(b1Component, Is.EqualTo(b1.GetBlock()));
        Assert.That(b1Component, Is.EqualTo(Utils.GetBlock(b1.Id)));
        Assert.That(b1, Is.EqualTo(b1Component.GetBlockData()));

        var b1SecondInstance = b1.Install()[0];
        var b1SecondComponent = b1SecondInstance.GetComponent<BuildingBlock>();

        var list = new[] { b1Component, b1SecondComponent }.ToList();
        Assert.That(list, Is.EquivalentTo(b1.GetBlocks()));

        DeleteBlockData(b1);
        yield return null;
    }
}

#endif // OVRPLUGIN_TESTING
