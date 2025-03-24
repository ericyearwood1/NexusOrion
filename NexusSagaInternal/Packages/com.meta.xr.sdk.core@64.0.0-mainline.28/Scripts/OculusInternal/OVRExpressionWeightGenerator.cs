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

using UnityEngine;

/// <summary>
/// Base class for custom blend shape transformations
/// </summary>
[HelpURL("https://developer.oculus.com/reference/unity/latest/class_o_v_r_expression_weight_generator")]
public abstract class OVRExpressionWeightGenerator : ScriptableObject
{
    /// <summary>
    /// Calculate a specific custom blend shape weight
    /// </summary>
    /// <param name="customBlendshapeIndex">Index of requested blend shape</param>
    /// <param name="source">Provider of Oculus Face Expression values</param>
    /// <param name="weightValue">Returned a calculated weight for requested blend shape</param>
    /// <returns>True if Face Expressions are active, False if a weight cannot be calculated</returns>
    public abstract bool CalculateCustomWeight(int customBlendshapeIndex, OVRFaceExpressions.WeightProvider source,
        out float weightValue);

    /// <summary>
    /// Amount of provided custom blend shapes
    /// </summary>
    public abstract int WeightCount { get; }

    /// <summary>
    /// Name of a specific custom blend shape
    /// </summary>
    /// <param name="blendshapeIndex">Index of requested blend shape</param>
    /// <returns>Name of requested blend shape</returns>
    public abstract string GetWeightName(int blendshapeIndex);
}
