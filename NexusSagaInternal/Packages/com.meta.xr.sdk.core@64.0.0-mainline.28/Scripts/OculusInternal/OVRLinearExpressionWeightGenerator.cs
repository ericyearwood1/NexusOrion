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
using UnityEngine;
using UnityEngine.Serialization;

/// <summary>
/// This class provides the face expression weight mapping using a linear transformation.
/// Instantiate that class and define your custom blend shape set with transformation coefficients.
/// </summary>
[HelpURL("https://developer.oculus.com/reference/unity/latest/class_o_v_r_linear_expression_weight_generator")]
[CreateAssetMenu(fileName = "OVRLinearExpressionWeightGenerator",
    menuName = "ScriptableObjects/OVRLinearExpressionWeightGenerator", order = 1)]
public class OVRLinearExpressionWeightGenerator : OVRExpressionWeightGenerator
{
    /// <summary>
    /// Data class for transformation coefficients.
    /// </summary>
    [Serializable]
    public class OVRCustomExpressionField
    {
        /// <summary>
        /// Oculus face expression blend shape.
        /// </summary>
        public OVRFaceExpressions.FaceExpression Expression = OVRFaceExpressions.FaceExpression.Invalid;

        /// <summary>
        /// Contribution of this value to the overall result.
        /// </summary>
        [Range(0.0f, 1.0f)] public float Contribution = 1f;
    }

    /// <summary>
    /// Data class for transformation formulas.
    /// It represents a linear transformation by the formula "Σ(weight(i) * coefficient(i)) * MULT" clamped
    /// to a selected range.
    /// </summary>
    [Serializable]
    public class OVRCustomExpressionFormula
    {
        /// <summary>
        /// Name of the custom blend shape set
        /// </summary>
        [FormerlySerializedAs("name")] public string Name;

        /// <summary>
        /// List of source face expression blend shapes with coefficients
        /// </summary>
        public OVRCustomExpressionField[] Components;

        /// <summary>
        /// The minimum clamped blendshape weight for this set of facial expressions.
        /// </summary>
        [Range(0.0f, 2.0f)] public float MinValue = 0.0f;

        /// <summary>
        /// The maximum clamped blendshape weight for this set of facial expressions.
        /// </summary>
        [Range(0.0f, 2.0f)] public float MaxValue = 1.0f;

        /// <summary>
        /// The blendshape weight multiplier for this set of facial expressions.
        /// </summary>
        [Range(0.0f, 2.0f)] public float Multiplier = 1.0f;
    }

    [SerializeField]
    [Tooltip("Transformation Formulas")]
    protected internal OVRCustomExpressionFormula[] _customExpressionFormulas;

    private void OnValidate()
    {
        if (_customExpressionFormulas == null)
        {
            _customExpressionFormulas = Array.Empty<OVRCustomExpressionFormula>();
        }
    }

    /// <inheritdoc />
    public override int WeightCount
    {
        get => _customExpressionFormulas.Length;
    }

    /// <inheritdoc />
    public override string GetWeightName(int blendshapeIndex)
    {
        return _customExpressionFormulas[blendshapeIndex].Name;
    }

    /// <inheritdoc />
    public override bool CalculateCustomWeight(int customBlendshapeIndex,
        OVRFaceExpressions.WeightProvider source,
        out float weightValue)
    {
        if (source == null || customBlendshapeIndex < 0 || customBlendshapeIndex >= _customExpressionFormulas.Length)
        {
            weightValue = 0f;
            return false;
        }

        OVRCustomExpressionFormula formula = _customExpressionFormulas[customBlendshapeIndex];
        float currentWeight = 0f;
        float overallProportion = 0f;
        if (formula.Components != null && formula.Components.Length > 0)
        {
            foreach (var field in formula.Components)
            {
                if (field.Expression >= 0 && field.Expression < OVRFaceExpressions.FaceExpression.Max)
                {
                    currentWeight += source.GetWeight(field.Expression) * field.Contribution;
                    overallProportion += field.Contribution;
                }
            }
        }

        if (overallProportion > Single.Epsilon)
        {
            currentWeight /= overallProportion;
        }
        else
        {
            currentWeight = 0f;
        }

        weightValue = Mathf.Clamp(currentWeight * formula.Multiplier,
            formula.MinValue, formula.MaxValue);

        return true;
    }
}
