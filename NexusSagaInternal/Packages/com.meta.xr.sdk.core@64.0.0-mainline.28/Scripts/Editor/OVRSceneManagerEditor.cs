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

using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(OVRSceneManager))]
internal class OVRSceneManagerEditor : Editor
{
    private SerializedProperty _planePrefab;
    private SerializedProperty _volumePrefab;
    private SerializedProperty _prefabOverrides;
    private SerializedProperty _verboseLogging;
    private SerializedProperty _maxSceneAnchorUpdatesPerFrame;
    private SerializedProperty _initialAnchorParent;
    private SerializedProperty _activeRoomsOnly;
#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE // XR_META_return_to_room
    private SerializedProperty _roomEntered;
    private SerializedProperty _roomExited;
    private SerializedProperty _keepUserInRooms;
#endif
#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE // XR_META_spatial_entity_discovery
    private SerializedProperty _useExperimentalAnchorApi;
#endif
    private bool _showAdvanced;

    private void OnEnable()
    {
        _planePrefab = serializedObject.FindProperty(nameof(OVRSceneManager.PlanePrefab));
        _volumePrefab = serializedObject.FindProperty(nameof(OVRSceneManager.VolumePrefab));
        _prefabOverrides = serializedObject.FindProperty(nameof(OVRSceneManager.PrefabOverrides));
        _verboseLogging = serializedObject.FindProperty(nameof(OVRSceneManager.VerboseLogging));
        _maxSceneAnchorUpdatesPerFrame =
            serializedObject.FindProperty(nameof(OVRSceneManager.MaxSceneAnchorUpdatesPerFrame));
        _activeRoomsOnly = serializedObject.FindProperty(nameof(OVRSceneManager.ActiveRoomsOnly));
        _initialAnchorParent = serializedObject.FindProperty(nameof(OVRSceneManager._initialAnchorParent));
#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE // XR_META_return_to_room
        _roomEntered = serializedObject.FindProperty(nameof(OVRSceneManager.RoomEntered));
        _roomExited = serializedObject.FindProperty(nameof(OVRSceneManager.RoomExited));
        _keepUserInRooms = serializedObject.FindProperty(nameof(OVRSceneManager._keepUserInRooms));
#endif
#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE // XR_META_spatial_entity_discovery
        _useExperimentalAnchorApi = serializedObject.FindProperty(nameof(OVRSceneManager._useExperimentalAnchorApi));
#endif
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        EditorGUILayout.PropertyField(_planePrefab);
        EditorGUILayout.PropertyField(_volumePrefab);
        EditorGUILayout.PropertyField(_prefabOverrides);
        EditorGUILayout.PropertyField(_activeRoomsOnly);
#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE // XR_META_return_to_room
        EditorGUILayout.PropertyField(_keepUserInRooms, new GUIContent("Keep User in Room(s)",
            "When enabled, the system will attempt to keep the user rooms spawned by the Scene Manager by displaying a system prompt if the user moves out of the room."));
        EditorGUILayout.PropertyField(_roomEntered);
        EditorGUILayout.PropertyField(_roomExited);
#endif
        _showAdvanced = EditorGUILayout.Foldout(_showAdvanced, "Advanced");
        if (_showAdvanced)
        {
            EditorGUILayout.PropertyField(_verboseLogging);
            EditorGUILayout.PropertyField(_maxSceneAnchorUpdatesPerFrame);
            EditorGUILayout.PropertyField(_initialAnchorParent);
#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE // XR_META_spatial_entity_discovery
            EditorGUILayout.PropertyField(_useExperimentalAnchorApi);
            var projectConfig = OVRProjectConfig.GetProjectConfig();
            if (_useExperimentalAnchorApi.boolValue && !projectConfig.experimentalFeaturesEnabled)
            {
                EditorGUILayout.HelpBox("To use this feature, you must enable experimental features in the OculusProjectConfig.", MessageType.Error);
                if (GUILayout.Button("Open OculusProjectConfig"))
                {
                    AssetDatabase.OpenAsset(projectConfig);
                    OVRProjectConfigEditor.selectedTab = OVRProjectConfigEditor.eProjectConfigTab.Experimental;
                }
            }
#endif
        }

        serializedObject.ApplyModifiedProperties();
    }
}
