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
using NUnit.Framework;
using UnityEngine;
using Object = UnityEngine.Object;
using Random = UnityEngine.Random;

//-------------------------------------------------------------------------------------
/// <summary>
/// Tests for the OVRFaceExpressions class.
/// </summary>
[TestFixture]
public class OVRFaceExpressionsTests
{
    private OVRFaceExpressions _faceExpressions;

    [SetUp]
    public void SetUp()
    {
        var go = new GameObject();
        _faceExpressions = go.AddComponent<OVRFaceExpressions>();
    }

    [TearDown]
    public void CleanUp()
    {
        Object.DestroyImmediate(_faceExpressions.gameObject);
    }

    [Test]
    public void TestInvalidFaceExpressionsGetCall()
    {
        _faceExpressions.SetFaceState(false, new OVRPlugin.FaceState());

        const OVRFaceExpressions.FaceExpression faceExpression = OVRFaceExpressions.FaceExpression.JawDrop;
        Assert.Throws<InvalidOperationException>(() =>
        {
            var result = _faceExpressions[faceExpression];
        });

        {
            var result = _faceExpressions.TryGetFaceExpressionWeight(faceExpression, out var weight);
            Assert.IsFalse(result);
            Assert.AreEqual(weight, 0.0f);
        }
    }

    [Test]
    public void TestInvalidRangeGetCall()
    {
        _faceExpressions.SetFaceState(true, new OVRPlugin.FaceState());

        const OVRFaceExpressions.FaceExpression faceExpression = OVRFaceExpressions.FaceExpression.Max + 1;
        Assert.Throws<ArgumentOutOfRangeException>(() =>
        {
            var result = _faceExpressions[faceExpression];
        });

        {
            var result = _faceExpressions.TryGetFaceExpressionWeight(faceExpression, out var weight);
            Assert.IsFalse(result);
            Assert.AreEqual(weight, 0.0f);
        }
    }

    [Test]
    public void TestValidGetCall()
    {
        var faceState = new OVRPlugin.FaceState
        {
            ExpressionWeights = new float[(int)OVRPlugin.FaceConstants.MaxFaceExpressions]
        };
        const OVRFaceExpressions.FaceExpression faceExpression = OVRFaceExpressions.FaceExpression.JawDrop;

        var realWeight = Random.Range(0, 1);
        faceState.ExpressionWeights[(int)faceExpression] = realWeight;

        _faceExpressions.SetFaceState(true, faceState);

        var result1 = _faceExpressions[faceExpression];
        var result2 = _faceExpressions.TryGetFaceExpressionWeight(faceExpression, out var weight);
        Assert.IsTrue(result2);
        Assert.AreEqual(realWeight, result1);
        Assert.AreEqual(weight, result1);
    }

    [Test]
    public void TestInvalidFaceExpressionsCopyCall()
    {
        _faceExpressions.SetFaceState(false, new OVRPlugin.FaceState());

        var floatArray = new float[(int)OVRFaceExpressions.FaceExpression.Max];

        Assert.Throws<InvalidOperationException>(() => { _faceExpressions.CopyTo(floatArray); });
    }

    [Test]
    public void TestNullArrayCopyCall()
    {
        _faceExpressions.SetFaceState(false, new OVRPlugin.FaceState());
        Assert.Throws<ArgumentNullException>(() => { _faceExpressions.CopyTo(null); });
    }

    [Test]
    public void TestValidCopyCall()
    {
        const int maxFaceExpressions = (int)OVRPlugin.FaceConstants.MaxFaceExpressions2;
        var expressionWeights = new float[maxFaceExpressions];

        for (var i = 0; i < maxFaceExpressions; i++)
        {
            expressionWeights[i] = i;
        }

        var faceState = new OVRPlugin.FaceState
        {
            ExpressionWeights = expressionWeights
        };

        _faceExpressions.SetFaceState(true, faceState);

        var floatArray1 = new float[maxFaceExpressions];

        _faceExpressions.CopyTo(floatArray1);
        Assert.AreEqual(floatArray1, expressionWeights);
    }
}

#endif // OVRPLUGIN_TESTING
