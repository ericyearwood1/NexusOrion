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

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEngine;
using UnityEngine.Assertions;

[CustomPropertyDrawer(typeof(OVRMappedFace.Mapping))]
class OVRMappedFaceMappingPropertyDrawer : PropertyDrawer
{
    enum FieldError
    {
        None,
        DuplicatedTarget,
        IncorrectTarget,
        IncorrectSource
    }

    private static GUIContent DuplicatedTargetStatus = EditorGUIUtility.IconContent("console.erroricon.sml",
        "That Renderer blend shape name should be used only once");

    private static GUIContent IncorrectTargetStatus = EditorGUIUtility.IconContent("console.erroricon.sml",
        "That name doesn't exist in the Renderer mesh");

    private static GUIContent IncorrectSourceStatus = EditorGUIUtility.IconContent("console.erroricon.sml",
        "That name doesn't exist in the Generator output list");

    public override void OnGUI(Rect rect, SerializedProperty property, GUIContent label)
    {
        var targetObject = property.serializedObject.targetObject;
        var targetAsScript = (OVRMappedFace)targetObject;

        var renderer = targetAsScript.RetrieveSkinnedMeshRenderer();
        var generator = targetAsScript._generator;

        var targetNameError = FieldError.None;
        var sourceNameError = FieldError.None;

        var incorrectSourceNames = OVRObjectPool.HashSet<string>();
        var incorrectTargetNames = OVRObjectPool.HashSet<string>();
        var unmappedTargetNames = OVRObjectPool.HashSet<string>();
        var duplicatedTargetNames = OVRObjectPool.HashSet<string>();

        if (generator != null && renderer != null && renderer.sharedMesh != null)
        {
            targetAsScript.CalculateMappingCache(incorrectTargetNames, incorrectSourceNames, unmappedTargetNames,
                duplicatedTargetNames);
        }

        var meshField = property.FindPropertyRelative(nameof(OVRMappedFace.Mapping.MeshBlendShapeName));
        Assert.IsNotNull(meshField);

        var currentTargetValue = meshField.stringValue;
        if (duplicatedTargetNames.Contains(currentTargetValue))
        {
            targetNameError = FieldError.DuplicatedTarget;
        }

        int targetCount, targetIndex;
        GUIContent[] targetChoices;
        if (renderer != null && renderer.sharedMesh != null)
        {
            var mesh = renderer.sharedMesh;

            targetCount = mesh.blendShapeCount;
            targetChoices = new GUIContent[targetCount];
            targetIndex = -1;

            if (targetCount > 0)
            {
                for (int i = 0; i < targetCount; i++)
                {
                    var targetName = mesh.GetBlendShapeName(i);
                    targetChoices[i] = new GUIContent(targetName);
                    if (currentTargetValue == targetName)
                    {
                        targetIndex = i;
                    }
                }
            }

            if (targetIndex < 0)
            {
                targetIndex = targetChoices.Length;
                targetChoices = targetChoices.Append(new GUIContent(currentTargetValue)).ToArray();
                targetNameError = FieldError.IncorrectTarget;
            }
        }
        else
        {
            targetCount = 1;
            targetIndex = 0;
            targetChoices = new[] { new GUIContent(currentTargetValue) };
        }

        var generatorField = property.FindPropertyRelative(nameof(OVRMappedFace.Mapping.GeneratorWeightName));
        Assert.IsNotNull(generatorField);

        var currentSourceValue = generatorField.stringValue;
        int sourceIndex, sourceCount;
        GUIContent[] sourceChoices;
        if (generator != null)
        {
            sourceCount = generator.WeightCount;
            sourceChoices = new GUIContent[sourceCount];
            sourceIndex = -1;
            if (sourceCount > 0)
            {
                for (int i = 0; i < sourceCount; i++)
                {
                    var sourceName = generator.GetWeightName(i);
                    sourceChoices[i] = new GUIContent(sourceName);
                    if (sourceName == currentSourceValue)
                    {
                        sourceIndex = i;
                    }
                }
            }

            if (sourceIndex < 0)
            {
                sourceIndex = sourceChoices.Length;
                sourceCount = sourceChoices.Length + 1;
                sourceChoices = sourceChoices.Append(new GUIContent(currentSourceValue)).ToArray();
                sourceNameError = FieldError.IncorrectSource;
            }
        }
        else
        {
            sourceIndex = 0;
            sourceCount = 1;
            sourceChoices = new[] { new GUIContent(currentSourceValue) };
        }

        using (new EditorGUI.PropertyScope(rect, label, property))
        {
            var r = rect;
            r.width /= 2;

            EditorGUIUtility.labelWidth = r.height;

            var newTargetIndex = EditorGUI.Popup(r, MakeLabel(targetNameError), targetIndex, targetChoices);
            if (newTargetIndex < targetCount)
            {
                meshField.stringValue = targetChoices[newTargetIndex].text;
            }

            r.x += r.width + 16;
            r.width -= 16;
            var newSourceIndex = EditorGUI.Popup(r, MakeLabel(sourceNameError), sourceIndex, sourceChoices);
            if (newSourceIndex < sourceCount)
            {
                generatorField.stringValue = sourceChoices[newSourceIndex].text;
            }
        }

        OVRObjectPool.Return(incorrectSourceNames);
        OVRObjectPool.Return(incorrectTargetNames);
        OVRObjectPool.Return(unmappedTargetNames);
        OVRObjectPool.Return(duplicatedTargetNames);
    }

    private GUIContent MakeLabel(FieldError error) => error switch
    {
        FieldError.None => GUIContent.none,
        FieldError.DuplicatedTarget => DuplicatedTargetStatus,
        FieldError.IncorrectTarget => IncorrectTargetStatus,
        FieldError.IncorrectSource => IncorrectSourceStatus,
        _ => throw new ArgumentOutOfRangeException(nameof(error), error, null)
    };
}

[CustomEditor(typeof(OVRMappedFace))]
public class OVRMappedFaceEditor : Editor
{
    public override void OnInspectorGUI()
    {
        var targetAsScript = (OVRMappedFace)target;

        var incorrectSourceNames = OVRObjectPool.HashSet<string>();
        var incorrectTargetNames = OVRObjectPool.HashSet<string>();
        var unmappedTargetNames = OVRObjectPool.HashSet<string>();
        var duplicatedTargetNames = OVRObjectPool.HashSet<string>();

        if (targetAsScript._generator != null &&
            targetAsScript._faceExpressions != null &&
            targetAsScript.RetrieveSkinnedMeshRenderer() != null)
        {
            targetAsScript.CalculateMappingCache(incorrectTargetNames, incorrectSourceNames, unmappedTargetNames,
                duplicatedTargetNames);

            WarningIfRequired("Generator blend shape names are not known", incorrectSourceNames);
            WarningIfRequired("Target mesh blend shape names do not exist in the mesh", incorrectTargetNames);
            WarningIfRequired("Target mesh blend shapes have no defined source", unmappedTargetNames);
            WarningIfRequired("Target mesh blend shapes are duplicated", duplicatedTargetNames);
        }

        DrawDefaultInspector();

        OVRObjectPool.Return(incorrectSourceNames);
        OVRObjectPool.Return(incorrectTargetNames);
        OVRObjectPool.Return(unmappedTargetNames);
        OVRObjectPool.Return(duplicatedTargetNames);

        if (GUILayout.Button("Automap") &&
            targetAsScript._generator != null &&
            targetAsScript.RetrieveSkinnedMeshRenderer() != null)
        {
            var renderer = targetAsScript.RetrieveSkinnedMeshRenderer();
            var generator = targetAsScript._generator;

            var newMapping = GenerateMapping(renderer, generator);

            saveMapping(targetAsScript, newMapping.ToArray());
        }

        serializedObject.ApplyModifiedProperties();
    }

    private int calculateLevenshteinDistance(string source1, string source2)
    {
        var source1Length = source1.Length;
        var source2Length = source2.Length;

        var matrix = new int[source1Length + 1, source2Length + 1];

        // First calculation, if one entry is empty return full length
        if (source1Length == 0)
            return source2Length;

        if (source2Length == 0)
            return source1Length;

        // Initialization of matrix with row size source1Length and columns size source2Length
        for (var i = 0; i <= source1Length; matrix[i, 0] = i++)
        {
        }

        for (var j = 0; j <= source2Length; matrix[0, j] = j++)
        {
        }

        // Calculate rows and collumns distances
        for (var i = 1; i <= source1Length; i++)
        {
            for (var j = 1; j <= source2Length; j++)
            {
                var cost = (source2[j - 1] == source1[i - 1]) ? 0 : 1;

                matrix[i, j] = Math.Min(
                    Math.Min(matrix[i - 1, j] + 1, matrix[i, j - 1] + 1),
                    matrix[i - 1, j - 1] + cost);
            }
        }

        // return result
        return matrix[source1Length, source2Length];
    }

    private (string, string)[] substitution =
    {
        ("left", "l"),
        ("right", "r")
    };

    private string UnifyDirections(string name)
    {
        var res = name.ToLower();
        foreach (var s in substitution)
        {
            res = Regex.Replace(res, s.Item1, s.Item2);
        }

        return res;
    }

    private int CalculateSmartDistance(string a, string b)
    {
        return calculateLevenshteinDistance(UnifyDirections(a), UnifyDirections(b));
    }


    private List<(string, string)> GenerateMapping(SkinnedMeshRenderer renderer, OVRExpressionWeightGenerator generator)
    {
        Assert.IsNotNull(renderer, "OVRMappedFace should be placed to an object with SkinnedMeshRenderer and SharedMesh");
        Assert.IsNotNull(renderer.sharedMesh, "OVRMappedFace should be placed to an object with SkinnedMeshRenderer and SharedMesh");
        Assert.IsNotNull(generator);
        var targetBlendshapeCount = renderer.sharedMesh.blendShapeCount;
        var sourceBlendshapeCount = generator.WeightCount;

        var newMapping = new List<(string, string)>();

        if (targetBlendshapeCount == 0 || sourceBlendshapeCount == 0)
        {
            return newMapping;
        }

        HashSet<int> inUseSources = new HashSet<int>();
        Dictionary<int, List<int>> ambiguousTargets = new Dictionary<int, List<int>>();

        var generatorPrefix = FindTargetPrefix(renderer);

        // step 1. find the best one
        for (int i = 0; i < targetBlendshapeCount; i++)
        {
            var targetName = renderer.sharedMesh.GetBlendShapeName(i);

            int minDistance = Int32.MaxValue;
            List<int> candidates = new List<int>();
            for (int j = 0; j < sourceBlendshapeCount; j++)
            {
                var sourceName = generator.GetWeightName(j);
                if (sourceName == "")
                {
                    continue;
                }

                int d = CalculateSmartDistance(targetName.Substring(generatorPrefix), sourceName);
                if (d < minDistance)
                {
                    minDistance = d;
                    candidates.Clear();
                }

                if (d == minDistance)
                {
                    candidates.Add(j);
                }
            }

            if (candidates.Count == 1)
            {
                inUseSources.Add(candidates[0]);
                newMapping.Add((targetName, generator.GetWeightName(candidates[0])));
            }
            else
            {
                ambiguousTargets.Add(i, candidates);
            }
        }

        // step 2. from equal candidates try to take a not used one
        int currentAmbiguousTargetsCount = Int32.MaxValue;
        while (ambiguousTargets.Count > 0 && currentAmbiguousTargetsCount > ambiguousTargets.Count)
        {
            currentAmbiguousTargetsCount = ambiguousTargets.Count;
            Dictionary<int, List<int>> ambiguousTargetsToKeep = new Dictionary<int, List<int>>();

            foreach (var ambiguousTarget in ambiguousTargets)
            {
                var candidates = ambiguousTarget.Value;
                var freeCandidates = candidates.Except(inUseSources).ToList();
                if (freeCandidates.Count == 1)
                {
                    inUseSources.Add(ambiguousTarget.Key);
                    newMapping.Add((renderer.sharedMesh.GetBlendShapeName(ambiguousTarget.Key),
                        generator.GetWeightName(freeCandidates[0])));
                }
                else
                {
                    ambiguousTargetsToKeep.Add(ambiguousTarget.Key, ambiguousTarget.Value);
                }
            }

            ambiguousTargets = ambiguousTargetsToKeep;
        }

        // step 3. take the first one
        foreach (var ambiguousTarget in ambiguousTargets)
        {
            newMapping.Add((renderer.sharedMesh.GetBlendShapeName(ambiguousTarget.Key),
                generator.GetWeightName(ambiguousTarget.Value[0])));
        }

        return newMapping;
    }

    private static int FindTargetPrefix(SkinnedMeshRenderer renderer)
    {
        var blendshapeCount = renderer.sharedMesh.blendShapeCount;
        var prefix = 0;
        if (blendshapeCount > 1)
        {
            var etalon = renderer.sharedMesh.GetBlendShapeName(0);
            prefix = etalon.Length;

            for (int i = 1; i < blendshapeCount; i++)
            {
                var name = renderer.sharedMesh.GetBlendShapeName(i);
                if (prefix > name.Length)
                {
                    prefix = name.Length;
                }

                while (prefix > 0 &&
                       etalon.Substring(0, prefix) !=
                       name.Substring(0, prefix))
                {
                    prefix--;
                }

                if (prefix == 0)
                {
                    break;
                }
            }
        }

        return prefix;
    }

    private void saveMapping(OVRMappedFace targetAsScript, (string, string)[] template)
    {
        targetAsScript._mappings =
            new OVRMappedFace.Mapping[template.Length];
        for (int i = 0; i < template.Length; i++)
        {
            targetAsScript._mappings[i] = new OVRMappedFace.Mapping()
            {
                GeneratorWeightName = template[i].Item2,
                MeshBlendShapeName = template[i].Item1
            };
            ;
        }

        EditorUtility.SetDirty(target);
    }

    private void WarningIfRequired(string message, HashSet<string> badValues)
    {
        if (badValues.Count == 0)
        {
            return;
        }

        StringBuilder sb = new StringBuilder().Append(message).Append(": ");
        bool firstOne = true;
        foreach (var v in badValues)
        {
            if (!firstOne)
            {
                sb.Append(", ");
            }

            sb.Append("'").Append(v).Append("'");
            firstOne = false;
        }

        EditorGUILayout.HelpBox(sb.ToString(), MessageType.Error);
    }
}
