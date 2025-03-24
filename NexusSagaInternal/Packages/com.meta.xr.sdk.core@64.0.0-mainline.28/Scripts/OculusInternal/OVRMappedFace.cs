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
using System.Text;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Serialization;

/// <summary>
/// Class provides a way for mapping the mesh blend shapes to any OVRExpressionWeightGenerator source
/// </summary>
[HelpURL("https://developer.oculus.com/reference/unity/latest/class_o_v_r_mapped_face")]
public class OVRMappedFace : OVRFace
{
    /// <summary>
    /// Data class for mapping between mesh and generator blend shapes
    /// </summary>
    [Serializable]
    public class Mapping
    {
        /// <summary>
        /// Name of the mesh blend shape
        /// </summary>
        public string MeshBlendShapeName;

        /// <summary>
        /// Name of the generator blend shape
        /// </summary>
        public string GeneratorWeightName;
    }

    [SerializeField] internal protected OVRExpressionWeightGenerator _generator;

    [SerializeField] internal protected Mapping[] _mappings;

    private int[] _mappingCache = Array.Empty<int>();

    /// <inheritdoc />
    protected internal override bool GetWeightValue(int blendShapeIndex, out float weightValue)
    {
        Assert.IsNotNull(_generator);
        Assert.IsNotNull(_faceExpressions);

        if (blendShapeIndex >= 0 && blendShapeIndex < _mappingCache.Length &&
            _generator.CalculateCustomWeight(
                _mappingCache[blendShapeIndex], _faceExpressions,
                out var generatedWeightValue))
        {
            weightValue = generatedWeightValue * _blendShapeStrengthMultiplier;
            return true;
        }

        weightValue = 0f;
        return false;
    }

    private void RecalculateMappingCacheWithWarning()
    {
        var incorrectSourceNames = OVRObjectPool.HashSet<string>();
        var incorrectTargetNames = OVRObjectPool.HashSet<string>();
        var unmappedTargetNames = OVRObjectPool.HashSet<string>();
        var duplicatedTargetNames = OVRObjectPool.HashSet<string>();

        CalculateMappingCache(incorrectTargetNames, incorrectSourceNames, unmappedTargetNames, duplicatedTargetNames);

        MaybeLogBadValues("Generator blend shape names are not known", incorrectSourceNames);
        MaybeLogBadValues("Target mesh blend shape names do not exist in the mesh", incorrectTargetNames);
        MaybeLogBadValues("Target mesh blend shapes have no defined source", unmappedTargetNames);
        MaybeLogBadValues("Target mesh blend shapes are duplicated", duplicatedTargetNames);

        OVRObjectPool.Return(incorrectSourceNames);
        OVRObjectPool.Return(incorrectTargetNames);
        OVRObjectPool.Return(unmappedTargetNames);
        OVRObjectPool.Return(duplicatedTargetNames);
    }

    internal void CalculateMappingCache(HashSet<string> incorrectTargetNames,
        HashSet<string> incorrectSourceNames, HashSet<string> unmappedTargetNames, HashSet<string> duplicateTargetNames)
    {
        var objectRenderer = RetrieveSkinnedMeshRenderer();
        if (objectRenderer == null || objectRenderer.sharedMesh == null)
        {
            return;
        }

        var sourceBlendshapeCount = _generator.WeightCount;
        var sourceNames = OVRObjectPool.Dictionary<string, int>();
        for (int i = 0; i < sourceBlendshapeCount; i++)
        {
            var weightName = _generator.GetWeightName(i);
            if (weightName != "" && !sourceNames.ContainsKey(weightName))
            {
                sourceNames.Add(weightName, i);
            }
        }

        var targetBlendshapeCount = objectRenderer.sharedMesh.blendShapeCount;
        var targetNames = OVRObjectPool.Dictionary<string, int>();
        for (int i = 0; i < targetBlendshapeCount; i++)
        {
            targetNames.Add(objectRenderer.sharedMesh.GetBlendShapeName(i), i);
        }

        if (_mappingCache.Length != targetBlendshapeCount)
        {
            _mappingCache = new int[targetBlendshapeCount];
        }

        for (int i = 0; i < targetBlendshapeCount; i++)
        {
            _mappingCache[i] = -1;
        }

        var uniqueTargetNames = OVRObjectPool.HashSet<string>();
        for (int i = 0; i < _mappings.Length; i++)
        {
            var targetBlendshapeName = _mappings[i].MeshBlendShapeName;
            var sourceBlendshapeName = _mappings[i].GeneratorWeightName;
            if (uniqueTargetNames.Contains(targetBlendshapeName))
            {
                duplicateTargetNames.Add(targetBlendshapeName);
            }
            else
            {
                uniqueTargetNames.Add(targetBlendshapeName);
            }

            if (!targetNames.ContainsKey(targetBlendshapeName))
            {
                incorrectTargetNames.Add(targetBlendshapeName);
                continue;
            }

            if (!sourceNames.ContainsKey(sourceBlendshapeName))
            {
                incorrectSourceNames.Add(sourceBlendshapeName);
                continue;
            }

            _mappingCache[targetNames[targetBlendshapeName]] = sourceNames[sourceBlendshapeName];
        }

        for (int i = 0; i < targetBlendshapeCount; i++)
        {
            if (_mappingCache[i] == -1)
            {
                unmappedTargetNames.Add(objectRenderer.sharedMesh.GetBlendShapeName(i));
            }
        }

        OVRObjectPool.Return(sourceNames);
        OVRObjectPool.Return(targetNames);
        OVRObjectPool.Return(uniqueTargetNames);
    }

    private void MaybeLogBadValues(string message, HashSet<string> badValues)
    {
        if (badValues.Count == 0)
        {
            return;
        }

        StringBuilder sb = new StringBuilder().Append(message).Append(": ");
        bool firstOne = true;
        foreach (var v in badValues)
        {
            sb.Append("'").Append(v).Append("'");
            if (!firstOne)
            {
                sb.Append(", ");
            }

            firstOne = false;
        }

        Debug.LogError(sb.ToString(), this);
    }

    protected override void Awake()
    {
        base.Awake();
        if (_mappings == null)
        {
            _mappings = Array.Empty<Mapping>();
        }
    }

    protected override void Start()
    {
        base.Start();
        Assert.IsNotNull(_generator, "Any instance of OVRLinearExpressionWeightGenerator should be linked");

        RecalculateMappingCacheWithWarning();
    }
}
