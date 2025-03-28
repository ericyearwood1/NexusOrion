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

#ifndef META_DEPTH_ENVIRONMENT_OCCLUSION_BIRP_INCLUDED
#define META_DEPTH_ENVIRONMENT_OCCLUSION_BIRP_INCLUDED

uniform UNITY_DECLARE_TEX2DARRAY(_EnvironmentDepthTexture);

#include "UnityCG.cginc"

float SampleEnvironmentDepth(float2 reprojectedUV) {
  return UNITY_SAMPLE_TEX2DARRAY(_EnvironmentDepthTexture,
           float3(reprojectedUV, (float)unity_StereoEyeIndex)).r;
}

#define META_DEPTH_CONVERT_OBJECT_TO_WORLD(objectPos) mul(unity_ObjectToWorld, objectPos).xyz;

float3 DepthConvertDepthToLinear(float zspace) {
  return LinearEyeDepth(zspace);
}

#include "../EnvironmentOcclusion.cginc"

#endif
