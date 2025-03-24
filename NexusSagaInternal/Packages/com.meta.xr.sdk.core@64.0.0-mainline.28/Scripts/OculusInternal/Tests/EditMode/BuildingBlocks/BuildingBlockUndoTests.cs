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
#if OVR_INTERNAL_CODE

using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Meta.XR.BuildingBlocks.Editor;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using Object = UnityEngine.Object;

internal class BuildingBlockUndoTests : OVRPluginEditModeTest
{
    private static readonly BlockData[]
        PublicBlockDataArray = Utils.PublicBlockData.OfType<BlockData>().ToArray();

    private void ClearScene()
    {
        var gos = SceneManager.GetActiveScene().GetRootGameObjects();
        foreach (var t in gos)
        {
            Object.DestroyImmediate(t);
        }
        Undo.ClearAll();
    }

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();
        ClearScene();
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        ClearScene();
        yield return base.UnityTearDown();
    }

    [UnityTest, Timeout(DefaultTimeoutMs)]
    public IEnumerator ValidateUndoRedoForBlockData([ValueSource(nameof(PublicBlockDataArray))] BlockData blockData)
    {
        var testClass = Utils.GetTestClassForBlockData<BuildingBlocksTest>(blockData.Id);
        if (testClass != null)
        {
            Assume.That(!testClass.SkipUndoTest);
        }

        if (!blockData.CanBeAdded)
        {
            yield break;
        }

        blockData.InstallWithDependencies();
        BlockData.FixSetupRules();
        yield return new WaitUntil(() => OVRProjectSetup.IsReady);

        // Get current state
        var gameObjectsBeforeUndo = GetAllGameObjects();
        var componentsBeforeUndo = GetAllComponents();

        // Do Undo
        Undo.PerformUndo();
        yield return null;

        // Expect no gameobject(s)
        Assert.Zero(GetAllGameObjects().Count);

        // Do Redo
        Undo.PerformRedo();
        yield return null;

        BlockData.FixSetupRules();
        yield return new WaitUntil(() => OVRProjectSetup.IsReady);

        // Compare: Expect same state with block(s) in scene after Redo
        Assert.True(GetAllGameObjects().SetEquals(gameObjectsBeforeUndo));

        var latestComponents = GetAllComponents();
        foreach (var (component, serializedObjectAfterRedo) in latestComponents)
        {
            Assert.True(componentsBeforeUndo.TryGetValue(component, out var so));

            var propertyBeforeUndo = so.GetIterator();
            while (propertyBeforeUndo.NextVisible(true))
            {
                var propertyAfterRedo = serializedObjectAfterRedo.FindProperty(propertyBeforeUndo.propertyPath);
                Assert.NotNull(propertyAfterRedo);
                Assert.True(SerializedProperty.DataEquals(propertyAfterRedo, propertyBeforeUndo));
            }
        }

        yield return null;
    }

    private Dictionary<Component, SerializedObject> GetAllComponents()
    {
        var componentMap = new Dictionary<Component, SerializedObject>();
        var gameObjects = GetAllGameObjects();
        foreach (var gameObject in gameObjects)
        {
            var components = gameObject.GetComponents<Component>().ToList();
            components.ForEach(c => componentMap.Add(c, new SerializedObject(c)));
        }

        return componentMap;
    }

    private HashSet<GameObject> GetAllGameObjects()
    {
        var gameObjects = new HashSet<GameObject>();

        foreach (GameObject gameObject in SceneManager.GetActiveScene().GetRootGameObjects())
        {
            gameObjects.Add(gameObject);
            GetChildren(gameObject.transform, gameObjects);
        }

        return gameObjects;
    }

    private void GetChildren(Transform root, HashSet<GameObject> gameObjects)
    {
        foreach (Transform child in root)
        {
            gameObjects.Add(child.gameObject);
            GetChildren(child, gameObjects);
        }
    }
}

#endif // OVR_INTERNAL_CODE
#endif // OVRPLUGIN_TESTING
