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


#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE

#if USING_XR_SDK_OCULUS
using Unity.XR.Oculus;
#endif
using UnityEngine;

namespace Meta.XR.EnvironmentDepth
{
    internal static class EnvironmentDepthUtils
    {
#if DEPTH_API_SUPPORTED && DEPTH_API_UNITY_SUPPORTED
        internal static bool IsDepthRenderingRequestEnabled;
        private static readonly Vector3 _scalingVector3 = new(1, 1, -1);

        internal static Vector4 ComputeNdcToLinearDepthParameters(float near, float far)
        {
            float invDepthFactor;
            float depthOffset;
            if (far < near || float.IsInfinity(far))
            {
                // Inf far plane:
                invDepthFactor = -2.0f * near;
                depthOffset = -1.0f;
            }
            else
            {
                // Finite far plane:
                invDepthFactor = -2.0f * far * near / (far - near);
                depthOffset = -(far + near) / (far - near);
            }

            return new Vector4(invDepthFactor, depthOffset, 0, 0);
        }

        internal static Matrix4x4 Calculate3DOFReprojection(Utils.EnvironmentDepthFrameDesc frameDesc, OVRPlugin.Fovf fov)
        {
            // Screen To Depth represents the transformation matrix used to map normalised screen UV coordinates to
            // normalised environment depth texture UV coordinates. This needs to account for 2 things:
            // 1. The field of view of the two textures may be different, Unreal typically renders using a symmetric fov.
            //    That is to say the FOV of the left and right eyes is the same. The environment depth on the other hand
            //    has a different FOV for the left and right eyes. So we need to scale and offset accordingly to account
            //    for this difference.
            var screenCameraToScreenNormCoord = MakeUnprojectionMatrix(
                fov.RightTan, fov.LeftTan,
                fov.UpTan, fov.DownTan);

            var depthNormCoordToDepthCamera = MakeProjectionMatrix(
                frameDesc.fovRightAngle, frameDesc.fovLeftAngle,
                frameDesc.fovTopAngle, frameDesc.fovDownAngle);

            // 2. The headset may have moved in between capturing the environment depth and rendering the frame. We
            //    can only account for rotation of the headset, not translation.
            var depthCameraToScreenCamera = MakeScreenToDepthMatrix(frameDesc);

            var screenToDepth = depthNormCoordToDepthCamera * depthCameraToScreenCamera *
                                screenCameraToScreenNormCoord;

            return screenToDepth;
        }

        public static Matrix4x4 CalculateReprojection(Utils.EnvironmentDepthFrameDesc frameDesc, OVRPlugin.Fovf fov)
        {
            float left = frameDesc.fovLeftAngle;
            float right = frameDesc.fovRightAngle;
            float bottom = frameDesc.fovDownAngle;
            float top = frameDesc.fovTopAngle;
            float near = frameDesc.nearZ;
            float far = frameDesc.farZ;

            float x = 2.0F / (right + left);
            float y = 2.0F / (top + bottom);
            float a = (right - left) / (right + left);
            float b = (top - bottom) / (top + bottom);
            float c;
            float d;
            if (float.IsInfinity(far))
            {
                c = -1.0F;
                d = -2.0f * near;
            }
            else
            {
                c = -(far + near) / (far - near);
                d = -(2.0F * far * near) / (far - near);
            }
            float e = -1.0F;
            Matrix4x4 m = new Matrix4x4
            {	
                m00 = x, m01 = 0, m02 = a, m03 = 0,
                m10 = 0, m11 = y, m12 = b, m13 = 0,
                m20 = 0, m21 = 0, m22 = c, m23 = d,
                m30 = 0, m31 = 0, m32 = e, m33 = 0
            };

            var createRotation = frameDesc.createPoseRotation;
            var depthOrientation = new Quaternion(
                createRotation.x,
                createRotation.y,
                createRotation.z,
                createRotation.w
            );

            var viewMatrix = Matrix4x4.TRS(frameDesc.createPoseLocation, depthOrientation,
                _scalingVector3).inverse;

            return m * viewMatrix;
        }

        private static Matrix4x4 MakeScreenToDepthMatrix(Utils.EnvironmentDepthFrameDesc frameDesc)
        {
            // The pose extrapolated to the predicted display time of the current frame
            // assuming left eye rotation == right eye
            var screenOrientation =
                OVRPlugin.GetNodePose(OVRPlugin.Node.EyeLeft, OVRPlugin.Step.Render).Orientation.FromQuatf();

            var depthOrientation = new Quaternion(
                -frameDesc.createPoseRotation.x,
                -frameDesc.createPoseRotation.y,
                frameDesc.createPoseRotation.z,
                frameDesc.createPoseRotation.w
            );

            var screenToDepthQuat = (Quaternion.Inverse(screenOrientation) * depthOrientation).eulerAngles;
            screenToDepthQuat.z = -screenToDepthQuat.z;

            return Matrix4x4.Rotate(Quaternion.Euler(screenToDepthQuat));
        }

        private static Matrix4x4 MakeProjectionMatrix(float rightTan, float leftTan, float upTan, float downTan)
        {
            var matrix = Matrix4x4.identity;
            float tanAngleWidth = rightTan + leftTan;
            float tanAngleHeight = upTan + downTan;

            // Scale
            matrix.m00 = 1.0f / tanAngleWidth;
            matrix.m11 = 1.0f / tanAngleHeight;

            // Offset
            matrix.m03 = leftTan / tanAngleWidth;
            matrix.m13 = downTan / tanAngleHeight;
            matrix.m23 = -1.0f;

            return matrix;
        }

        private static Matrix4x4 MakeUnprojectionMatrix(float rightTan, float leftTan, float upTan, float downTan)
        {
            var matrix = Matrix4x4.identity;

            // Scale
            matrix.m00 = rightTan + leftTan;
            matrix.m11 = upTan + downTan;

            // Offset
            matrix.m03 = -leftTan;
            matrix.m13 = -downTan;
            matrix.m23 = 1.0f;

            return matrix;
        }
#endif // DEPTH_API_SUPPORTED && DEPTH_API_UNITY_SUPPORTED

    }

}
#endif //OVR_PARTNER_CODE || OVR_INTERNAL_CODE
