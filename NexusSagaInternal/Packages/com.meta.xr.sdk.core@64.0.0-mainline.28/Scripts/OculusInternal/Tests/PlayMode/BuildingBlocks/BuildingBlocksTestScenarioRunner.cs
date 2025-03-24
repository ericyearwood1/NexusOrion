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

using System.Collections;
using System.Linq;
using Meta.XR.BuildingBlocks.Editor;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using Object = UnityEngine.Object;

#if OVRPLUGIN_TESTING

public class BuildingBlocksTestScenarioRunner
{
    private TestContext _testContext;
    private const int DefaultTimeoutMs = 10000;

    [UnitySetUp]
    public IEnumerator UnitySetUp()
    {
        while (!BlockBaseData.Registry.Values.Any())
        {
            // In CI the asset loading may take some time
            yield return null;
        }

        _testContext = new TestContext();
        _testContext.Mock();
        yield return null;
    }

    [UnityTearDown]
    public IEnumerator UnityTearDown()
    {
        foreach (var block in Utils.GetBlocksInScene())
        {
            Object.DestroyImmediate(block.gameObject);
        }

        _testContext.Unmock();

        yield return null;
    }

    public static readonly BlockData[]
        PublicBlockDataArray = Utils.PublicBlockData.OfType<BlockData>().ToArray();

    [UnityTest]
    [Timeout(DefaultTimeoutMs)]
    public IEnumerator RunBlockTestSet([ValueSource(nameof(PublicBlockDataArray))] BlockData blockData)
    {
        var testClass = Utils.GetTestClassForBlockData<BuildingBlocksTest>(blockData.Id);
        Assert.IsNotNull(testClass);
        Assume.That(!testClass.Skip);

        if (!blockData.CanBeAdded)
        {
            yield break;
        }

        blockData.InstallWithDependencies();

        yield return testClass.Setup(_testContext);

        foreach (var testCase in testClass.TestCaseList)
        {
            yield return testCase;
        }

        yield return testClass.TearDown(_testContext);
    }

    [UnityTest]
    [Timeout(10*DefaultTimeoutMs)]
    public IEnumerator RunScenarioWithAllBlocks()
    {
        // TODO: remove this expected error log once we actually implement that test case
        LogAssert.Expect(LogType.Error, "Passthrough layer is not configured for surface projected passthrough.");

        var testDictionary = PublicBlockDataArray
            .Select(blockData => Utils.GetTestClassForBlockData<BuildingBlocksTest>(blockData.Id))
            .Where(testClass => testClass != null)
            .ToDictionary(test => test.BlockId);

        foreach (var blockData in PublicBlockDataArray)
        {
            if (testDictionary.TryGetValue(blockData.Id, out var testClass))
            {
                if (testClass.Skip || !testClass.CanRunWithAllBlocks)
                {
                    continue;
                }
            }

            if (blockData.CanBeAdded)
            {
                blockData.InstallWithDependencies();
            }
        }

        foreach (var testClass in testDictionary.Values.Where(
                     testClass => !testClass.Skip && testClass.CanRunWithAllBlocks))
        {
            yield return testClass.Setup(_testContext);
        }

        foreach (var testClass in testDictionary.Values.Where(
                     testClass => !testClass.Skip && testClass.CanRunWithAllBlocks))
        {
            foreach (var testCase in testClass.TestCaseList)
            {
                yield return testCase;
            }
        }

        foreach (var testClass in testDictionary.Values.Where(
                     testClass => !testClass.Skip && testClass.CanRunWithAllBlocks))
        {
            yield return testClass.TearDown(_testContext);
        }
    }
}

#endif // OVRPLUGIN_TESTING
