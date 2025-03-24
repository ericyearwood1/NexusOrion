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

using System.Collections.Generic;
using System.Text;
using NUnit.Framework;

[TestFixture]
public class OVRCustomFaceAutomappingTest
{
    public static (string, string[])[] tokenizedStrings =
    {
        ("browLowerer_L", new[] { "", "brow", "browlowerer", "l", "L", "lowerer" }),
        ("browLowerer_R", new[] { "", "brow", "browlowerer", "lowerer", "r", "R" }),
        ("cheekPuff_L", new[] { "", "cheek", "cheekpuff", "l", "L", "puff" }),
        ("cheekPuff_R", new[] { "", "cheek", "cheekpuff", "puff", "r", "R" }),
        ("cheekRaiser_L", new[] { "", "cheek", "cheekraiser", "l", "L", "raiser" }),
        ("cheekRaiser_R", new[] { "", "cheek", "cheekraiser", "r", "R", "raiser" }),
        ("cheekSuck_L", new[] { "", "cheek", "cheeksuck", "l", "L", "suck" }),
        ("cheekSuck_R", new[] { "", "cheek", "cheeksuck", "r", "R", "suck" }),
        ("chinRaiser_B", new[] { "", "b", "chin", "chinraiser", "raiser" }),
        ("chinRaiser_T", new[] { "", "chin", "chinraiser", "raiser", "t" }),
        ("dimpler_L", new[] { "", "dimpler", "l", "L" }),
        ("dimpler_R", new[] { "", "dimpler", "r", "R" }),
        ("eyesClosed_L", new[] { "", "closed", "eyes", "eyesclosed", "l", "L" }),
        ("eyesClosed_R", new[] { "", "closed", "eyes", "eyesclosed", "r", "R" }),
        ("eyesLookDown_L", new[] { "", "down", "eyes", "eyeslookdown", "l", "L", "look" }),
        ("eyesLookDown_R", new[] { "", "down", "eyes", "eyeslookdown", "look", "r", "R" }),
        ("eyesLookLeft_L", new[] { "", "eyes", "eyeslookleft", "l", "L", "left", "look" }),
        ("eyesLookLeft_R", new[] { "", "eyes", "eyeslookleft", "L", "left", "look", "r", "R" }),
        ("eyesLookRight_L", new[] { "", "eyes", "eyeslookright", "l", "L", "look", "R", "right" }),
        ("eyesLookRight_R", new[] { "", "eyes", "eyeslookright", "look", "r", "R", "right" }),
        ("eyesLookUp_L", new[] { "", "eyes", "eyeslookup", "l", "L", "look", "up" }),
        ("eyesLookUp_R", new[] { "", "eyes", "eyeslookup", "look", "r", "R", "up" }),
        ("innerBrowRaiser_L", new[] { "", "brow", "inner", "innerbrowraiser", "l", "L", "raiser" }),
        ("innerBrowRaiser_R", new[] { "", "brow", "inner", "innerbrowraiser", "r", "R", "raiser" }),
        ("jawDrop", new[] { "drop", "jaw", "jawdrop" }),
        ("jawSidewaysLeft", new[] { "jaw", "jawsidewaysleft", "L", "left", "sideways" }),
        ("jawSidewaysRight", new[] { "jaw", "jawsidewaysright", "R", "right", "sideways" }),
        ("jawThrust", new[] { "jaw", "jawthrust", "thrust" }),
        ("lidTightener_L", new[] { "", "l", "L", "lid", "lidtightener", "tightener" }),
        ("lidTightener_R", new[] { "", "lid", "lidtightener", "r", "R", "tightener" }),
        ("lipCornerDepressor_L", new[] { "", "corner", "depressor", "l", "L", "lip", "lipcornerdepressor" }),
        ("lipCornerDepressor_R", new[] { "", "corner", "depressor", "lip", "lipcornerdepressor", "r", "R" }),
        ("lipCornerPuller_L", new[] { "", "corner", "l", "L", "lip", "lipcornerpuller", "puller" }),
        ("lipCornerPuller_R", new[] { "", "corner", "lip", "lipcornerpuller", "puller", "r", "R" }),
        ("lipFunneler_LB", new[] { "", "b", "funneler", "l", "L", "lb", "lip", "lipfunneler" }),
        ("lipFunneler_LT", new[] { "", "funneler", "l", "L", "lip", "lipfunneler", "lt", "t" }),
        ("lipFunneler_RB", new[] { "", "b", "funneler", "lip", "lipfunneler", "r", "R", "rb" }),
        ("lipFunneler_RT", new[] { "", "funneler", "lip", "lipfunneler", "r", "R", "rt", "t" }),
        ("lipPressor_L", new[] { "", "l", "L", "lip", "lippressor", "pressor" }),
        ("lipPressor_R", new[] { "", "lip", "lippressor", "pressor", "r", "R" }),
        ("lipPucker_L", new[] { "", "l", "L", "lip", "lippucker", "pucker" }),
        ("lipPucker_R", new[] { "", "lip", "lippucker", "pucker", "r", "R" }),
        ("lipsToward_LB", new[] { "", "b", "l", "L", "lb", "lips", "lipstoward", "toward" }),
        ("lipsToward_LT", new[] { "", "l", "L", "lips", "lipstoward", "lt", "t", "toward" }),
        ("lipsToward_RB", new[] { "", "b", "lips", "lipstoward", "r", "R", "rb", "toward" }),
        ("lipsToward_RT", new[] { "", "lips", "lipstoward", "r", "R", "rt", "t", "toward" }),
        ("lipStretcher_L", new[] { "", "l", "L", "lip", "lipstretcher", "stretcher" }),
        ("lipStretcher_R", new[] { "", "lip", "lipstretcher", "r", "R", "stretcher" }),
        ("lipSuck_LB", new[] { "", "b", "l", "L", "lb", "lip", "lipsuck", "suck" }),
        ("lipSuck_LT", new[] { "", "l", "L", "lip", "lipsuck", "lt", "suck", "t" }),
        ("lipSuck_RB", new[] { "", "b", "lip", "lipsuck", "r", "R", "rb", "suck" }),
        ("lipSuck_RT", new[] { "", "lip", "lipsuck", "r", "R", "rt", "suck", "t" }),
        ("lipTightener_L", new[] { "", "l", "L", "lip", "liptightener", "tightener" }),
        ("lipTightener_R", new[] { "", "lip", "liptightener", "r", "R", "tightener" }),
        ("lowerLipDepressor_L", new[] { "", "depressor", "l", "L", "lip", "lower", "lowerlipdepressor" }),
        ("lowerLipDepressor_R", new[] { "", "depressor", "lip", "lower", "lowerlipdepressor", "r", "R" }),
        ("mouthLeft", new[] { "L", "left", "mouth", "mouthleft" }),
        ("mouthRight", new[] { "mouth", "mouthright", "R", "right" }),
        ("nasolabialFurrow_L", new[] { "", "furrow", "l", "L", "nasolabial", "nasolabialfurrow" }),
        ("nasolabialFurrow_R", new[] { "", "furrow", "nasolabial", "nasolabialfurrow", "r", "R" }),
        ("noseWrinkler_L", new[] { "", "l", "L", "nose", "nosewrinkler", "wrinkler" }),
        ("noseWrinkler_R", new[] { "", "nose", "nosewrinkler", "r", "R", "wrinkler" }),
        ("nostrilCompressor_L", new[] { "", "compressor", "l", "L", "nostril", "nostrilcompressor" }),
        ("nostrilCompressor_R", new[] { "", "compressor", "nostril", "nostrilcompressor", "r", "R" }),
        ("nostrilDilator_L", new[] { "", "dilator", "l", "L", "nostril", "nostrildilator" }),
        ("nostrilDilator_R", new[] { "", "dilator", "nostril", "nostrildilator", "r", "R" }),
        ("outerBrowRaiser_L", new[] { "", "brow", "l", "L", "outer", "outerbrowraiser", "raiser" }),
        ("outerBrowRaiser_R", new[] { "", "brow", "outer", "outerbrowraiser", "r", "R", "raiser" }),
        ("upperLidRaiser_L", new[] { "", "l", "L", "lid", "raiser", "upper", "upperlidraiser" }),
        ("upperLidRaiser_R", new[] { "", "lid", "r", "R", "raiser", "upper", "upperlidraiser" }),
        ("upperLipRaiser_L", new[] { "", "l", "L", "lip", "raiser", "upper", "upperlipraiser" }),
        ("upperLipRaiser_R", new[] { "", "lip", "r", "R", "raiser", "upper", "upperlipraiser" })
    };

    [Test]
    [TestCaseSource(nameof(tokenizedStrings))]
    public void TestTokenizeString((string, string[]) testItem)
    {
        var (test, expected) = testItem;
        HashSet<string> tokenized = OVRCustomFaceExtensions.TokenizeString(test);
        HashSet<string> expectedSet = new HashSet<string>(expected);
        Assert.IsTrue(expectedSet.SetEquals(tokenized),
            "For the string \"{0}\" was expected: {1} but received: {2}",
            test, PrintHashSet(expectedSet), PrintHashSet(tokenized));
    }

    private string PrintHashSet(HashSet<string> src)
    {
        SortedSet<string> sorted = new SortedSet<string>(src);
        bool yetAnotherOne = false;
        StringBuilder sb = new StringBuilder().Append("{ ");
        foreach (var s in sorted)
        {
            if (yetAnotherOne)
            {
                sb.Append(", ");
            }

            sb.Append("\"").Append(s).Append("\"");
            yetAnotherOne = true;
        }

        sb.Append(" }");

        return sb.ToString();
    }

    public (string, OVRFaceExpressions.FaceExpression)[] oculusFaceMapping = new[]
    {
        ("bls_head.browLowerer_L", OVRFaceExpressions.FaceExpression.BrowLowererL),
        ("bls_head.browLowerer_R", OVRFaceExpressions.FaceExpression.BrowLowererR),
        ("bls_head.cheekPuff_L", OVRFaceExpressions.FaceExpression.CheekPuffL),
        ("bls_head.cheekPuff_R", OVRFaceExpressions.FaceExpression.CheekPuffR),
        ("bls_head.cheekRaiser_L", OVRFaceExpressions.FaceExpression.CheekRaiserL),
        ("bls_head.cheekRaiser_R", OVRFaceExpressions.FaceExpression.CheekRaiserR),
        ("bls_head.cheekSuck_L", OVRFaceExpressions.FaceExpression.CheekSuckL),
        ("bls_head.cheekSuck_R", OVRFaceExpressions.FaceExpression.CheekSuckR),
        ("bls_head.chinRaiser_B", OVRFaceExpressions.FaceExpression.ChinRaiserB),
        ("bls_head.chinRaiser_T", OVRFaceExpressions.FaceExpression.ChinRaiserT),
        ("bls_head.dimpler_L", OVRFaceExpressions.FaceExpression.DimplerL),
        ("bls_head.dimpler_R", OVRFaceExpressions.FaceExpression.DimplerR),
        ("bls_head.eyesClosed_L", OVRFaceExpressions.FaceExpression.EyesClosedL),
        ("bls_head.eyesClosed_R", OVRFaceExpressions.FaceExpression.EyesClosedR),
        ("bls_head.eyesLookDown_L", OVRFaceExpressions.FaceExpression.EyesLookDownL),
        ("bls_head.eyesLookDown_R", OVRFaceExpressions.FaceExpression.EyesLookDownR),
        ("bls_head.eyesLookLeft_L", OVRFaceExpressions.FaceExpression.EyesLookLeftL),
        ("bls_head.eyesLookLeft_R", OVRFaceExpressions.FaceExpression.EyesLookLeftR),
        ("bls_head.eyesLookRight_L", OVRFaceExpressions.FaceExpression.EyesLookRightL),
        ("bls_head.eyesLookRight_R", OVRFaceExpressions.FaceExpression.EyesLookRightR),
        ("bls_head.eyesLookUp_L", OVRFaceExpressions.FaceExpression.EyesLookUpL),
        ("bls_head.eyesLookUp_R", OVRFaceExpressions.FaceExpression.EyesLookUpR),
        ("bls_head.innerBrowRaiser_L", OVRFaceExpressions.FaceExpression.InnerBrowRaiserL),
        ("bls_head.innerBrowRaiser_R", OVRFaceExpressions.FaceExpression.InnerBrowRaiserR),
        ("bls_head.jawDrop", OVRFaceExpressions.FaceExpression.JawDrop),
        ("bls_head.jawSidewaysLeft", OVRFaceExpressions.FaceExpression.JawSidewaysLeft),
        ("bls_head.jawSidewaysRight", OVRFaceExpressions.FaceExpression.JawSidewaysRight),
        ("bls_head.jawThrust", OVRFaceExpressions.FaceExpression.JawThrust),
        ("bls_head.lidTightener_L", OVRFaceExpressions.FaceExpression.LidTightenerL),
        ("bls_head.lidTightener_R", OVRFaceExpressions.FaceExpression.LidTightenerR),
        ("bls_head.lipCornerDepressor_L", OVRFaceExpressions.FaceExpression.LipCornerDepressorL),
        ("bls_head.lipCornerDepressor_R", OVRFaceExpressions.FaceExpression.LipCornerDepressorR),
        ("bls_head.lipCornerPuller_L", OVRFaceExpressions.FaceExpression.LipCornerPullerL),
        ("bls_head.lipCornerPuller_R", OVRFaceExpressions.FaceExpression.LipCornerPullerR),
        ("bls_head.lipFunneler_LB", OVRFaceExpressions.FaceExpression.LipFunnelerLB),
        ("bls_head.lipFunneler_LT", OVRFaceExpressions.FaceExpression.LipFunnelerLT),
        ("bls_head.lipFunneler_RB", OVRFaceExpressions.FaceExpression.LipFunnelerRB),
        ("bls_head.lipFunneler_RT", OVRFaceExpressions.FaceExpression.LipFunnelerRT),
        ("bls_head.lipPressor_L", OVRFaceExpressions.FaceExpression.LipPressorL),
        ("bls_head.lipPressor_R", OVRFaceExpressions.FaceExpression.LipPressorR),
        ("bls_head.lipPucker_L", OVRFaceExpressions.FaceExpression.LipPuckerL),
        ("bls_head.lipPucker_R", OVRFaceExpressions.FaceExpression.LipPuckerR),
        ("bls_head.lipsToward_LB", OVRFaceExpressions.FaceExpression.LipsToward),
        ("bls_head.lipsToward_LT", OVRFaceExpressions.FaceExpression.LipsToward),
        ("bls_head.lipsToward_RB", OVRFaceExpressions.FaceExpression.LipsToward),
        ("bls_head.lipsToward_RT", OVRFaceExpressions.FaceExpression.LipsToward),
        ("bls_head.lipStretcher_L", OVRFaceExpressions.FaceExpression.LipStretcherL),
        ("bls_head.lipStretcher_R", OVRFaceExpressions.FaceExpression.LipStretcherR),
        ("bls_head.lipSuck_LB", OVRFaceExpressions.FaceExpression.LipSuckLB),
        ("bls_head.lipSuck_LT", OVRFaceExpressions.FaceExpression.LipSuckLT),
        ("bls_head.lipSuck_RB", OVRFaceExpressions.FaceExpression.LipSuckRB),
        ("bls_head.lipSuck_RT", OVRFaceExpressions.FaceExpression.LipSuckRT),
        ("bls_head.lipTightener_L", OVRFaceExpressions.FaceExpression.LipTightenerL),
        ("bls_head.lipTightener_R", OVRFaceExpressions.FaceExpression.LipTightenerR),
        ("bls_head.lowerLipDepressor_L", OVRFaceExpressions.FaceExpression.LowerLipDepressorL),
        ("bls_head.lowerLipDepressor_R", OVRFaceExpressions.FaceExpression.LowerLipDepressorR),
        ("bls_head.mouthLeft", OVRFaceExpressions.FaceExpression.MouthLeft),
        ("bls_head.mouthRight", OVRFaceExpressions.FaceExpression.MouthRight),
        ("bls_head.nasolabialFurrow_L", OVRFaceExpressions.FaceExpression.Invalid),
        ("bls_head.nasolabialFurrow_R", OVRFaceExpressions.FaceExpression.Invalid),
        ("bls_head.noseWrinkler_L", OVRFaceExpressions.FaceExpression.NoseWrinklerL),
        ("bls_head.noseWrinkler_R", OVRFaceExpressions.FaceExpression.NoseWrinklerR),
        ("bls_head.nostrilCompressor_L", OVRFaceExpressions.FaceExpression.Invalid),
        ("bls_head.nostrilCompressor_R", OVRFaceExpressions.FaceExpression.Invalid),
        ("bls_head.nostrilDilator_L", OVRFaceExpressions.FaceExpression.Invalid),
        ("bls_head.nostrilDilator_R", OVRFaceExpressions.FaceExpression.Invalid),
        ("bls_head.outerBrowRaiser_L", OVRFaceExpressions.FaceExpression.OuterBrowRaiserL),
        ("bls_head.outerBrowRaiser_R", OVRFaceExpressions.FaceExpression.OuterBrowRaiserR),
        ("bls_head.upperLidRaiser_L", OVRFaceExpressions.FaceExpression.UpperLidRaiserL),
        ("bls_head.upperLidRaiser_R", OVRFaceExpressions.FaceExpression.UpperLidRaiserR),
        ("bls_head.upperLipRaiser_L", OVRFaceExpressions.FaceExpression.UpperLipRaiserL),
        ("bls_head.upperLipRaiser_R", OVRFaceExpressions.FaceExpression.UpperLipRaiserR)
    };

    [Test]
    public void TestOculusAutomapping()
    {
        var blendShapeNames = new string[oculusFaceMapping.Length];
        for (var i = 0; i < oculusFaceMapping.Length; i++) blendShapeNames[i] = oculusFaceMapping[i].Item1;

        var mesh = OVRMockMeshCreator.GenerateMockMesh(blendShapeNames);
        Assert.AreEqual(blendShapeNames.Length, mesh.blendShapeCount);

        var generatedMapping =
            OVRCustomFaceExtensions.OculusFaceAutoGenerateMapping(mesh, true);
        Assert.AreEqual(oculusFaceMapping.Length, generatedMapping.Length);

        for (var i = 0; i < oculusFaceMapping.Length; i++)
            Assert.AreEqual(oculusFaceMapping[i].Item2, generatedMapping[i], "for the blend shape {0}",
                oculusFaceMapping[i].Item1);
    }

    public static (string, OVRFaceExpressions.FaceExpression)[] arKitMapping = new[]
    {
        ("blendShape2.eyeBlink_L", OVRFaceExpressions.FaceExpression.EyesClosedL),
        ("blendShape2.eyeLookDown_L", OVRFaceExpressions.FaceExpression.EyesLookDownL),
        ("blendShape2.eyeLookIn_L", OVRFaceExpressions.FaceExpression.EyesLookRightL),
        ("blendShape2.eyeLookOut_L", OVRFaceExpressions.FaceExpression.EyesLookLeftL),
        ("blendShape2.eyeLookUp_L", OVRFaceExpressions.FaceExpression.EyesLookUpL),
        ("blendShape2.eyeSquint_L", OVRFaceExpressions.FaceExpression.LidTightenerL),
        ("blendShape2.eyeWide_L", OVRFaceExpressions.FaceExpression.UpperLidRaiserL),
        ("blendShape2.eyeBlink_R", OVRFaceExpressions.FaceExpression.EyesClosedR),
        ("blendShape2.eyeLookDown_R", OVRFaceExpressions.FaceExpression.EyesLookDownR),
        ("blendShape2.eyeLookIn_R", OVRFaceExpressions.FaceExpression.EyesLookLeftR),
        ("blendShape2.eyeLookOut_R", OVRFaceExpressions.FaceExpression.EyesLookRightR),
        ("blendShape2.eyeLookUp_R", OVRFaceExpressions.FaceExpression.EyesLookUpR),
        ("blendShape2.eyeWide_R", OVRFaceExpressions.FaceExpression.UpperLidRaiserR),
        ("blendShape2.jawForward", OVRFaceExpressions.FaceExpression.JawThrust),
        ("blendShape2.jawLeft", OVRFaceExpressions.FaceExpression.JawSidewaysLeft),
        ("blendShape2.jawOpen", OVRFaceExpressions.FaceExpression.JawDrop),
        ("blendShape2.jawRight", OVRFaceExpressions.FaceExpression.JawSidewaysRight),
        ("blendShape2.mouthPucker", OVRFaceExpressions.FaceExpression.LipPuckerL),
        ("blendShape2.mouthFunnel", OVRFaceExpressions.FaceExpression.LipFunnelerLB),
        ("blendShape2.mouthClose", OVRFaceExpressions.FaceExpression.LipsToward),
        ("blendShape2.mouthSmile_R", OVRFaceExpressions.FaceExpression.LipCornerPullerR),
        ("blendShape2.mouthLeft", OVRFaceExpressions.FaceExpression.MouthLeft),
        ("blendShape2.mouthRight", OVRFaceExpressions.FaceExpression.MouthRight),
        ("blendShape2.mouthSmile_L", OVRFaceExpressions.FaceExpression.LipCornerPullerL),
        ("blendShape2.mouthFrown_R", OVRFaceExpressions.FaceExpression.LipCornerDepressorR),
        ("blendShape2.mouthFrown_L", OVRFaceExpressions.FaceExpression.LipCornerDepressorL),
        ("blendShape2.mouthDimple_L", OVRFaceExpressions.FaceExpression.DimplerL),
        ("blendShape2.mouthDimple_R", OVRFaceExpressions.FaceExpression.DimplerR),
        ("blendShape2.mouthStretch_R", OVRFaceExpressions.FaceExpression.LipStretcherR),
        ("blendShape2.mouthStretch_L", OVRFaceExpressions.FaceExpression.LipStretcherL),
        ("blendShape2.mouthRollUpper", OVRFaceExpressions.FaceExpression.LipSuckLT),
        ("blendShape2.mouthRollLower", OVRFaceExpressions.FaceExpression.LipSuckLB),
        ("blendShape2.mouthShrugUpper", OVRFaceExpressions.FaceExpression.ChinRaiserT),
        ("blendShape2.mouthShrugLower", OVRFaceExpressions.FaceExpression.ChinRaiserB),
        ("blendShape2.mouthPress_L", OVRFaceExpressions.FaceExpression.LipPressorL),
        ("blendShape2.mouthPress_R", OVRFaceExpressions.FaceExpression.LipPressorR),
        ("blendShape2.mouthLowerDown_L", OVRFaceExpressions.FaceExpression.LowerLipDepressorL),
        ("blendShape2.mouthLowerDown_R", OVRFaceExpressions.FaceExpression.LowerLipDepressorR),
        ("blendShape2.mouthUpperUp_R", OVRFaceExpressions.FaceExpression.UpperLipRaiserR),
        ("blendShape2.mouthUpperUp_L", OVRFaceExpressions.FaceExpression.UpperLipRaiserL),
        ("blendShape2.browDown_L", OVRFaceExpressions.FaceExpression.BrowLowererL),
        ("blendShape2.browDown_R", OVRFaceExpressions.FaceExpression.BrowLowererR),
        ("blendShape2.browInnerUp", OVRFaceExpressions.FaceExpression.InnerBrowRaiserL),
        ("blendShape2.browOuterUp_R", OVRFaceExpressions.FaceExpression.OuterBrowRaiserR),
        ("blendShape2.browOuterUp_L", OVRFaceExpressions.FaceExpression.OuterBrowRaiserL),
        ("blendShape2.cheekPuff", OVRFaceExpressions.FaceExpression.CheekPuffL),
        ("blendShape2.cheekSquint_L", OVRFaceExpressions.FaceExpression.CheekRaiserL),
        ("blendShape2.cheekSquint_R", OVRFaceExpressions.FaceExpression.CheekRaiserR),
        ("blendShape2.noseSneer_L", OVRFaceExpressions.FaceExpression.NoseWrinklerL),
        ("blendShape2.noseSneer_R", OVRFaceExpressions.FaceExpression.NoseWrinklerR)
    };

#if OVR_INTERNAL_CODE
    [Test]
    public void TestARKitAutomapping()
    {
        var blendShapeNames = new string[arKitMapping.Length];
        for (var i = 0; i < arKitMapping.Length; i++) blendShapeNames[i] = arKitMapping[i].Item1;

        var mesh = OVRMockMeshCreator.GenerateMockMesh(blendShapeNames);
        Assert.AreEqual(blendShapeNames.Length, mesh.blendShapeCount);

        var generatedMapping =
            OVRCustomFaceExtensions.ARKitAutoGeneratedMapping(mesh, true);
        Assert.AreEqual(arKitMapping.Length, generatedMapping.Length);

        for (var i = 0; i < arKitMapping.Length; i++)
            Assert.AreEqual(arKitMapping[i].Item2, generatedMapping[i], "for the blend shape {0}",
                arKitMapping[i].Item1);
    }
#endif //OVR_INTERNAL_CODE
}

#endif // OVRPLUGIN_TESTING
