// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace ARDS.Utils.Editor
{
    public class FigmaSpringSolverEditor : EditorWindow
    {
        [SerializeField] private FigmaSpringSolverPresetName presetName;
        [SerializeField] private float _mass = 1;
        [SerializeField] private float _stiffness = 100;
        [SerializeField] private float _damping = 15;
        [SerializeField] private float _resolutionInterval = .002f;

        [SerializeField] private AnimationCurve _animationCurve = new AnimationCurve();

        private FigmaSpringSolver _solver;

        [MenuItem("ARDS/FigmaSpringSolverEditor")]
        private static void ShowWindow()
        {
            var window = GetWindow<FigmaSpringSolverEditor>();
            window.titleContent = new GUIContent("Figma Spring Solver");
            window.Show();
            window._solver = new FigmaSpringSolver(window._mass, window._stiffness, window._damping, 0);
        }

        private void OnGUI()
        {
            EditorGUILayout.BeginVertical();
            EditorGUI.BeginChangeCheck();
            presetName = (FigmaSpringSolverPresetName)EditorGUILayout.EnumPopup("Preset", presetName);
            //check if preset changed and modify values here
            if (EditorGUI.EndChangeCheck())
                PresetToValues();


            EditorGUI.BeginChangeCheck();
            _mass = EditorGUILayout.FloatField("Mass", _mass);
            _stiffness = EditorGUILayout.FloatField("Stiffness", _stiffness);
            _damping = EditorGUILayout.FloatField("Damping", _damping);
            _resolutionInterval = EditorGUILayout.Slider("Resolution Interval", _resolutionInterval, .002f, .1f);
            //check if settings conform to preset, or custom
            if (EditorGUI.EndChangeCheck())
                presetName = ValuesToPresetName();

            //curve field doesn't allow copy/paste. use serializedproperty and propertyfield
            var so = new SerializedObject(this);
            var curveProp = so.FindProperty(nameof(_animationCurve));
            EditorGUILayout.PropertyField(curveProp, new GUIContent("Curve"));
            so.ApplyModifiedProperties();

            EditorGUILayout.EndVertical();

            if (!GUI.changed) return;
            //calculate curve below
            _solver = new FigmaSpringSolver(_mass, _stiffness, _damping, 0);
            var keyframeList = new List<Keyframe>();
            for (float i = 0; i < 4f; i += _resolutionInterval)
            {
                var keyframe = new Keyframe(i, _solver.Solve(i));
                keyframeList.Add(keyframe);

                if (keyframeList.Count <= 2) continue;
                //if two values in a row are close to 1, clamp curve
                if (Mathf.Approximately(keyframeList[^2].value, 1) &&
                    Mathf.Approximately(keyframeList[^1].value, 1))
                {
                    break;
                }
            }

            _animationCurve = new AnimationCurve
            {
                keys = keyframeList.ToArray()
            };

            for (var i = 0; i < _animationCurve.keys.Length; i++)
            {
                // AnimationUtility.SetKeyLeftTangentMode(_animationCurve, i, AnimationUtility.TangentMode.Linear);
                // AnimationUtility.SetKeyRightTangentMode(_animationCurve, i, AnimationUtility.TangentMode.Linear);
                _animationCurve.SmoothTangents(i, 0);
            }
        }

        private void PresetToValues()
        {
            switch (presetName)
            {
                case FigmaSpringSolverPresetName.Gentle:
                    _stiffness = 100;
                    _damping = 15;
                    _mass = 1;
                    _resolutionInterval = .002f;
                    break;
                case FigmaSpringSolverPresetName.Quick:
                    _stiffness = 300;
                    _damping = 20;
                    _mass = 1;
                    _resolutionInterval = .002f;
                    break;
                case FigmaSpringSolverPresetName.Bouncy:
                    _stiffness = 600;
                    _damping = 15;
                    _mass = 1;
                    _resolutionInterval = .002f;
                    break;
                case FigmaSpringSolverPresetName.Slow:
                    _stiffness = 80;
                    _damping = 20;
                    _mass = 1;
                    _resolutionInterval = .002f;
                    break;
                case FigmaSpringSolverPresetName.Custom:
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        private FigmaSpringSolverPresetName ValuesToPresetName()
        {
            if (Mathf.Approximately(_mass, 1f) &&
                Mathf.Approximately(_stiffness, 100f) &&
                Mathf.Approximately(_damping, 15f))
            {
                return FigmaSpringSolverPresetName.Gentle;
            }

            if (Mathf.Approximately(_mass, 1f) &&
                Mathf.Approximately(_stiffness, 300f) &&
                Mathf.Approximately(_damping, 20f))
            {
                return FigmaSpringSolverPresetName.Quick;
            }

            if (Mathf.Approximately(_mass, 1f) &&
                Mathf.Approximately(_stiffness, 600f) &&
                Mathf.Approximately(_damping, 15f))
            {
                return FigmaSpringSolverPresetName.Bouncy;
            }

            if (Mathf.Approximately(_mass, 1f) &&
                Mathf.Approximately(_stiffness, 80f) &&
                Mathf.Approximately(_damping, 20f))
            {
                return FigmaSpringSolverPresetName.Slow;
            }


            return FigmaSpringSolverPresetName.Custom;
        }


        public enum FigmaSpringSolverPresetName
        {
            Gentle,
            Quick,
            Bouncy,
            Slow,
            Custom
        }
    }
}
