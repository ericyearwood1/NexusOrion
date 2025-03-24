using System;
using System.Collections.Generic;
using System.Linq;
using OSIG.Tools.StateMachines;
using UnityEditor;
using UnityEngine;
using UnityEditorInternal;
using StateMachine = OSIG.Tools.StateMachines.StateMachine;

namespace ARGlasses.Components
{
    [CustomEditor(typeof(ViewStateVisualsOverride))]
    public class ViewStateVisualsOverrideEditor : Editor
    {
        private SerializedProperty _colorMachineOverrideProperty;
        private SerializedProperty _motionMachineOverrideProperty;

        private void OnEnable()
        {
            _motionMachineOverrideProperty = serializedObject.FindProperty("MotionMachineOverride");
            _colorMachineOverrideProperty = serializedObject.FindProperty("ColorMachineOverride");
        }

        public override void OnInspectorGUI()
        {
            ViewStateVisualsOverride viewStateVisualsOverride = (ViewStateVisualsOverride)target;

            var viewStateVisuals = viewStateVisualsOverride.GetComponentInChildren<ViewStateVisuals>();
            if (viewStateVisuals == null)
            {
                GUILayout.Label("No ViewStateVisuals component found in children.");
                return;
            }

            GUIStyle missingNamesLabelStyle = new GUIStyle(GUI.skin.label);
            missingNamesLabelStyle.richText = true;

            if (viewStateVisuals.ColorMachine != null && viewStateVisuals.ColorMachine.Definition != null)
            {
                bool colorMachineOverrideIsValid = ValidateStateMachine(viewStateVisuals.ColorMachine.Definition,
                    viewStateVisualsOverride.ColorMachineOverride.Definition, "Color Machine Override",
                    missingNamesLabelStyle);

                if (viewStateVisualsOverride.ColorMachineOverride.Definition == null || !colorMachineOverrideIsValid)
                {
                    DrawCopyOriginalToOverride(viewStateVisuals.ColorMachine,
                        viewStateVisualsOverride.ColorMachineOverride, "Copy Color Machine");
                }

                EditorGUILayout.PropertyField(_colorMachineOverrideProperty);
            }

            if (viewStateVisuals.MotionMachine != null && viewStateVisuals.MotionMachine.Definition != null)
            {
                bool motionMachineOverrideIsValid = ValidateStateMachine(viewStateVisuals.MotionMachine.Definition,
                    viewStateVisualsOverride.MotionMachineOverride.Definition, "Motion Machine Override",
                    missingNamesLabelStyle);

                if (viewStateVisualsOverride.MotionMachineOverride.Definition == null || !motionMachineOverrideIsValid)
                {
                    DrawCopyOriginalToOverride(viewStateVisuals.MotionMachine,
                        viewStateVisualsOverride.MotionMachineOverride, "Copy Motion Machine");
                }

                EditorGUILayout.PropertyField(_motionMachineOverrideProperty);
            }

            serializedObject.ApplyModifiedProperties();
            viewStateVisualsOverride.ForceUpdate();
        }

        private bool ValidateStateMachine(StateMachineDefinition originalSMD, StateMachineDefinition overrideSMD,
            string machineName, GUIStyle missingNamesLabelStyle)
        {
            if (originalSMD != null && overrideSMD != null)
            {
                var missingComponentNames = CheckRequiredComponentNames(originalSMD, overrideSMD);
                var missingStateNames = CheckRequiredStateNames(originalSMD, overrideSMD);

                if (missingComponentNames.Count > 0)
                {
                    string missingComponentNamesString = string.Join(", ", missingComponentNames);
                    EditorGUILayout.LabelField(
                        $"{machineName} is missing components for: <color=red>{missingComponentNamesString}</color>",
                        missingNamesLabelStyle);
                    return false;
                }

                if (missingStateNames.Count > 0)
                {
                    string missingStateNamesString = string.Join(", ", missingStateNames);
                    EditorGUILayout.LabelField(
                        $"{machineName} is missing states for: <color=red>{missingStateNamesString}</color>",
                        missingNamesLabelStyle);
                    return false;
                }
            }

            return true;
        }

        private void DrawCopyOriginalToOverride(StateMachine originalSm, StateMachine overrideSm,
            string copyButtonLabel)
        {
            if (originalSm?.Definition == null) return;
            if (GUILayout.Button(copyButtonLabel))
            {
                CopyAsset(originalSm.Definition, overrideSm);
            }
        }

        private void CopyAsset(StateMachineDefinition originalSmDef, StateMachine overrideSm)
        {
            string sourcePath = AssetDatabase.GetAssetPath(originalSmDef);
            string destinationPath = EditorUtility.SaveFilePanelInProject("Save copy of state machine definition",
                originalSmDef.name + "-override", "asset", "Please enter a name");

            if (!string.IsNullOrEmpty(destinationPath))
            {
                AssetDatabase.CopyAsset(sourcePath, destinationPath);
                AssetDatabase.Refresh();
                var newAsset = AssetDatabase.LoadAssetAtPath<StateMachineDefinition>(destinationPath);
                overrideSm.Definition = newAsset;
            }
        }

        private List<String> CheckRequiredComponentNames(StateMachineDefinition originalSMD,
            StateMachineDefinition overrideSMD)
        {
            var originalNames = originalSMD.Components.OfType<IWillCreateVariable>().Select(x => x.VariableName)
                .ToList();
            var overrideNames = overrideSMD.Components.OfType<IWillCreateVariable>().Select(x => x.VariableName)
                .ToList();

            return originalNames.Except(overrideNames).ToList();
        }

        private List<String> CheckRequiredStateNames(StateMachineDefinition originalSMD,
            StateMachineDefinition overrideSMD)
        {
            var originalNames = originalSMD.States.Select(x => x.Name).ToList();
            var overrideNames = overrideSMD.States.Select(x => x.Name).ToList();

            return originalNames.Except(overrideNames).ToList();
        }
    }
}
