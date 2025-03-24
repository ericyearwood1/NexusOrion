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

#if OVR_INTERNAL_CODE

using System.Collections.Generic;
using Meta.XR.Editor.StatusMenu;
using Meta.XR.ImmersiveDebugger.Utils;
using UnityEditor;
using UnityEngine;
using static Meta.XR.Editor.UserInterface.Styles;
using static Meta.XR.Editor.UserInterface.Styles.Constants;
using static Meta.XR.Editor.UserInterface.Styles.Contents;
using static Meta.XR.Editor.UserInterface.Utils;

namespace Meta.XR.ImmersiveDebugger.Editor
{
    [CustomEditor(typeof(DebugInspector), true)]
    public class DebugInspectorInspector : UnityEditor.Editor
    {
        private readonly HashSet<object> _foldouts = new HashSet<object>();
        private bool _hasChanged;
        private float _dialogNoticeHorizontalPadding;
        private float _dialogNoticeVerticalPadding;
        private GUIContent _dialogNotice;

        private void Awake()
        {
            _dialogNotice = new GUIContent($"<b>{target.GetType().Name}</b> is part of the <b>{Utils.PublicName}</b>." +
                                           $"\nUse it to toggle tracking of any methods, properties or fields" +
                                           $" in any components of your game object.");

            _dialogNoticeHorizontalPadding = 3
                                             + GUIStyles.DialogIconStyle.fixedWidth
                                             + GUIStyles.DialogBox.padding.left
                                             + GUIStyles.DialogBox.padding.right;

            _dialogNoticeVerticalPadding = GUIStyles.DialogBox.padding.bottom + GUIStyles.DialogBox.padding.top;
        }

        private bool Foldout(object handle)
        {
            var rect = GUILayoutUtility.GetRect(12, EditorGUIUtility.singleLineHeight + 4);
            rect.x += 12;

            var foldout = _foldouts.Contains(handle);
            var newFoldout = EditorGUI.Foldout(rect, foldout, "", Styles.GUIStyles.FoldoutLeft);
            if (foldout != newFoldout)
            {
                foldout = newFoldout;
                if (newFoldout)
                {
                    _foldouts.Add(handle);
                }
                else
                {
                    _foldouts.Remove(handle);
                }
            }

            return newFoldout;
        }

        private void ShowHeaderGUI()
        {
            var currentWidth = EditorGUIUtility.currentViewWidth;

            var expectedButtonHeight = ItemHeight;
            GUILayout.BeginArea(new Rect(0, 0, currentWidth, expectedButtonHeight));
            EditorGUILayout.BeginHorizontal();
            Utils.Item.Show(null, true, Item.Origins.Component);
            EditorGUILayout.EndVertical();
            GUILayout.EndArea();
            GUILayoutUtility.GetRect(currentWidth, expectedButtonHeight);

            var infoWidth = currentWidth - _dialogNoticeHorizontalPadding;
            var expectedInfoHeight = GUIStyles.DialogTextStyle.CalcHeight(_dialogNotice,  infoWidth);
            expectedInfoHeight += _dialogNoticeVerticalPadding;
            GUILayout.BeginArea(new Rect(0, expectedButtonHeight, currentWidth, expectedInfoHeight));
            var rect = EditorGUILayout.BeginHorizontal(GUIStyles.DialogBox);
            EditorGUILayout.LabelField(DialogIcon, GUIStyles.DialogIconStyle, GUILayout.Width(GUIStyles.DialogIconStyle.fixedWidth));
            EditorGUILayout.BeginVertical();
            EditorGUILayout.LabelField(_dialogNotice, GUIStyles.DialogTextStyle);
            EditorGUILayout.EndVertical();
            EditorGUILayout.EndHorizontal();
            GUILayout.EndArea();
            GUILayoutUtility.GetRect(currentWidth, expectedInfoHeight);
        }

        private void DrawToggle(DebugInspector.InspectedItemBase item)
        {
            EditorGUI.EndDisabledGroup();
            using (new ColorScope(ColorScope.Scope.All,
                       item.enabled ? Styles.Colors.AccentColorBrighter : Color.white))
            {
                using (var changedCheckScope = new EditorGUI.ChangeCheckScope())
                {
                    item.enabled = EditorGUILayout.Toggle(item.enabled, GUILayout.Width(16.0f));
                    _hasChanged |= changedCheckScope.changed;
                }
            }
            EditorGUI.BeginDisabledGroup(!item.enabled);
        }

        private void ShowMemberGUI(DebugInspector.InspectedMember member)
        {
            if (!member.Valid) return;

            using (var disabledScope = new EditorGUI.DisabledScope(!member.enabled))
            {
                EditorGUILayout.BeginVertical(Styles.GUIStyles.ContentBox);

                using (new IndentScope(0))
                {
                    EditorGUILayout.BeginHorizontal();
                    var foldout = Foldout(member);
                    EditorGUILayout.LabelField(member.MemberInfo.BuildSignatureForDebugInspector(), Styles.GUIStyles.Label);
                    GUILayout.FlexibleSpace();
                    DrawToggle(member);
                    EditorGUILayout.EndHorizontal();

                    if (foldout)
                    {
                        var attribute = member.attribute;

                        using (var changedCheckScope = new EditorGUI.ChangeCheckScope())
                        {
                            if (member.MemberInfo.CanBeChanged())
                            {
                                attribute.Tweakable = EditorGUILayout.Toggle("Tweakable", attribute.Tweakable);
                                if (attribute.Tweakable)
                                {
                                    attribute.Min = EditorGUILayout.FloatField("Min", attribute.Min);
                                    attribute.Max = EditorGUILayout.FloatField("Max", attribute.Max);
                                }
                            }

                            attribute.GizmoType =
                                (DebugGizmoType)EditorGUILayout.EnumPopup("Gizmo Type", attribute.GizmoType);
                            EditorGUI.BeginChangeCheck();
                            var color = EditorGUILayout.ColorField(new GUIContent("Color"),
                                attribute.Color,
                                false, false, false);
                            if (EditorGUI.EndChangeCheck())
                            {
                                attribute.Color = color;
                            }

                            _hasChanged |= changedCheckScope.changed;
                        }
                    }
                }

                EditorGUILayout.EndVertical();
            }
        }

        private string ComputeComponentName(DebugInspector.InspectedHandle handle)
        {
            var prefix = handle.InstanceHandle.Type.IsPublic
                ? "public"
                : (handle.InstanceHandle.Type.IsNestedPrivate ? "private" : "internal");
            return $"<i>{prefix} class</i> <b>{handle.InstanceHandle.Type.Name}</b>";
        }

        private void ShowHandleGUI(DebugInspector.InspectedHandle handle)
        {
            if (!handle.Valid) return;

            using (new EditorGUI.DisabledScope(!handle.enabled))
            {
                EditorGUILayout.BeginVertical(Styles.GUIStyles.ContentBox);
                EditorGUILayout.BeginHorizontal();
                var foldout = Foldout(handle);
                EditorGUILayout.LabelField(ComputeComponentName(handle), Styles.GUIStyles.Label);
                using (new EditorGUI.DisabledScope(true))
                {
                    EditorGUILayout.ObjectField(handle.InstanceHandle.Instance, typeof(MonoBehaviour), true);
                }
                DrawToggle(handle);
                EditorGUILayout.EndHorizontal();

                if (foldout)
                {
                    EditorGUILayout.BeginVertical();
                    EditorGUILayout.Space();
                    EditorGUILayout.LabelField("Members", GUIStyles.BoldLabel);
                    using (new IndentScope(EditorGUI.indentLevel + 2))
                    {
                        foreach (var member in handle.inspectedMembers)
                        {
                            ShowMemberGUI(member);
                        }
                    }
                    EditorGUILayout.EndVertical();
                }

                EditorGUILayout.EndVertical();
            }
        }

        public override void OnInspectorGUI()
        {
            var inspector = target as DebugInspector;

            ShowHeaderGUI();

            using (new EditorGUI.DisabledScope(true))
            {
                EditorGUILayout.LabelField(Utils.PublicName, Utils.ComputeInfoText().Item1);
                EditorGUILayout.ObjectField("Tracked instance", inspector.gameObject, typeof(GameObject), true);
            }

            EditorGUILayout.Space();

            EditorGUILayout.LabelField("Components", GUIStyles.BoldLabel);

            var registry = inspector.Registry;
            foreach (var handle in registry.Handles)
            {
                ShowHandleGUI(handle);
            }

            if (_hasChanged)
            {
                EditorUtility.SetDirty(target);
                _hasChanged = false;
            }
        }
    }
}

#endif
