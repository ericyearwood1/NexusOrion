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
using System.ComponentModel;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

[CustomEditor(typeof(OVRBody))]
public class OVRBodyEditor : OVRSkeletonEditor
{
    public override void OnInspectorGUI()
    {
#if OVR_INTERNAL_CODE
        var trackingJointSet = OVRRuntimeSettings.GetRuntimeSettings().BodyTrackingJointSet;
        var usingJointSet = ((OVRBody)target).ProvidedSkeletonType;

        if (usingJointSet == OVRPlugin.BodyJointSet.FullBody && trackingJointSet == OVRPlugin.BodyJointSet.UpperBody)
        {
            EditorGUILayout.HelpBox("\"Full Body\" is used, but Full Body tracking is NOT enabled for the application. Check the \"Movement\" section of OVRManager (usually located in OVRCameraRig prefab).", MessageType.Warning);
        }
#endif // OVR_INTERNAL_CODE


        DrawDefaultInspector();
    }
}
