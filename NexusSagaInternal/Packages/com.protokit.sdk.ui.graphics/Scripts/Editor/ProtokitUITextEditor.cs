using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using OSIG.Tools.Layout;
using TMPro;
using TMPro.EditorUtilities;
using UnityEditor;
using UnityEngine;
using FontFeatureLookupFlags = UnityEngine.TextCore.LowLevel.FontFeatureLookupFlags;

namespace ProtoKit.UI.Graphics.Editor {
    #if UNITY_EDITOR
    [CanEditMultipleObjects]
    [CustomEditor(typeof(PKUIText))]
    public class PKUIEditor :
#if TMP_2_1_0_OR_NEWER
        TMP_EditorPanelUI {
#else
        TMP_UiEditorPanel {
#endif

        private static readonly GUIContent k_RtlToggleLabel = new GUIContent("Enable RTL Editor",
            "Reverses text direction and allows right to left editing.");

        private static readonly GUIContent k_MainSettingsLabel = new GUIContent("Main Settings");

        private static readonly GUIContent k_FontAssetLabel = new GUIContent("Font Asset",
            "The Font Asset containing the glyphs that can be rendered for this text.");

        private static readonly GUIContent k_MaterialPresetLabel = new GUIContent("Material Preset",
            "The material used for rendering. Only materials created from the Font Asset can be used.");

        private static readonly GUIContent k_AutoSizeLabel =
            new GUIContent("Auto Size", "Auto sizes the text to fit the available space.");

        private static readonly GUIContent k_FontSizeLabel =
            new GUIContent("Font Size", "The size the text will be rendered at in points.");

        private static readonly GUIContent k_AutoSizeOptionsLabel = new GUIContent("Auto Size Options");
        private static readonly GUIContent k_MinLabel = new GUIContent("Min", "The minimum font size.");
        private static readonly GUIContent k_MaxLabel = new GUIContent("Max", "The maximum font size.");

        private static readonly GUIContent k_WdLabel = new GUIContent("WD%",
            "Compresses character width up to this value before reducing font size.");

        private static readonly GUIContent k_LineLabel = new GUIContent("Line",
            "Negative value only. Compresses line height down to this value before reducing font size.");

        private static readonly GUIContent k_FontStyleLabel =
            new GUIContent("Font Style", "Styles to apply to the text such as Bold or Italic.");

        private static readonly GUIContent k_BoldLabel = new GUIContent("B", "Bold");
        private static readonly GUIContent k_ItalicLabel = new GUIContent("I", "Italic");
        private static readonly GUIContent k_UnderlineLabel = new GUIContent("U", "Underline");
        private static readonly GUIContent k_StrikethroughLabel = new GUIContent("S", "Strikethrough");
        private static readonly GUIContent k_LowercaseLabel = new GUIContent("ab", "Lowercase");
        private static readonly GUIContent k_UppercaseLabel = new GUIContent("AB", "Uppercase");
        private static readonly GUIContent k_SmallcapsLabel = new GUIContent("SC", "Smallcaps");

        private MethodInfo DrawColorMethod;
        private MethodInfo DrawSpacingMethod;
        private MethodInfo DrawAlignmentMethod;
        private MethodInfo DrawWrappingOverflowMethod;

        private SerializedProperty _widthProp;
        private SerializedProperty _heightProp;
        private SerializedProperty _fontSizePropOCValue;

        private static bool _showFontFoldout;
        private static bool _showColorsFoldout;
        private static bool _showAlignmentFoldout;
        private static bool _showExtraSettingsFoldout;

        protected override void OnEnable() {
            base.OnEnable();

            MethodInfo GetMethod(string name) {
                return typeof(TMP_BaseEditorPanel).GetMethod(name, BindingFlags.NonPublic | BindingFlags.Instance);
            }

            DrawColorMethod = GetMethod("DrawColor");
            DrawSpacingMethod = GetMethod("DrawSpacing");
            DrawAlignmentMethod = GetMethod("DrawAlignment");
            DrawWrappingOverflowMethod = GetMethod("DrawWrappingOverflow");

            _widthProp = serializedObject.FindProperty("_width");
            _heightProp = serializedObject.FindProperty("_height");
            _fontSizePropOCValue = serializedObject.FindProperty("_fontSizeOCValue");
        }

        public override void OnInspectorGUI() {
            if (IsMixSelectionTypes()) return;

            using (new EditorGUI.DisabledGroupScope(true)) {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("m_Script"));
            }

            string extraTMPComponentName = null;
            foreach (var target in targets)
            foreach (var tmp in (target as Component).GetComponents<TMP_Text>())
                if (!(tmp is PKUIText)) {
                    extraTMPComponentName = tmp.GetType().Name;
                    break;
                }

            if (extraTMPComponentName != null)
                EditorGUILayout.HelpBox(
                    $"OCText handles all text rendering functionality, the attached component {extraTMPComponentName} " +
                    "is not required and will interfere with the rendering and behavior of OCText!", MessageType.Error);

#if !TMP_2_1_0_OR_NEWER
            EditorGUILayout.LabelField("APPEARANCE", EditorStyles.miniBoldLabel);
#endif

            serializedObject.Update();

#if !TMP_2_1_0_OR_NEWER
            using (new GUILayout.VerticalScope(GUI.skin.box))
#endif
            {
                if (targets.Cast<PKUIText>().All(t => t.GetComponent<OCLayoutComponentBase>() == null))
                    using (var check = new EditorGUI.ChangeCheckScope()) {
                        EditorGUILayout.PropertyField(_widthProp);
                        EditorGUILayout.PropertyField(_heightProp);

                        if (check.changed) {
                            serializedObject.ApplyModifiedProperties();
                            foreach (var target in targets.Cast<PKUIText>()) {
                                target.GetComponent<RectTransform>().SetSizeWithCurrentAnchors(
                                    RectTransform.Axis.Horizontal, target.Width.GetMeters(target.UnitsContext));
                                target.GetComponent<RectTransform>().SetSizeWithCurrentAnchors(
                                    RectTransform.Axis.Vertical, target.Height.GetMeters(target.UnitsContext));
                            }
                        }
                    }

                DrawTextInput();

                if (targets.Length > 0) {
                    serializedObject.ApplyModifiedProperties();
                    EditorGUI.showMixedValue = false;
                }
            }

#if !TMP_2_1_0_OR_NEWER
            using (new GUILayout.VerticalScope(GUI.skin.box))
#endif
            {
#if TMP_2_1_0_OR_NEWER
                EditorGUIUtility.hierarchyMode = true;
#else
                EditorGUIUtility.hierarchyMode = false;
#endif

                _showFontFoldout = EditorGUILayout.Foldout(_showFontFoldout, "Font", true);
                if (_showFontFoldout)
                    using (new GUILayout.VerticalScope(GUI.skin.box)) {
                        DrawFontCustom();
                    }

                _showColorsFoldout = EditorGUILayout.Foldout(_showColorsFoldout, "Color", true);
                if (_showColorsFoldout)
                    using (new GUILayout.HorizontalScope(GUI.skin.box)) {
                        using (new GUILayout.VerticalScope()) {
                            DrawColorMethod.Invoke(this, null);
                        }

                    }

                _showAlignmentFoldout = EditorGUILayout.Foldout(_showAlignmentFoldout, "Alignment", true);
                if (_showAlignmentFoldout)
                    using (new GUILayout.VerticalScope(GUI.skin.box)) {
                        EditorGUIUtility.hierarchyMode = true;
                        DrawAlignmentMethod.Invoke(this, null);

                        DrawWrappingOverflowMethod.Invoke(this, null);

                        DrawSpacingMethod.Invoke(this, null);
                        EditorGUIUtility.hierarchyMode = false;
                    }

                _showExtraSettingsFoldout = EditorGUILayout.Foldout(_showExtraSettingsFoldout, "Extra Settings", true);
                if (_showExtraSettingsFoldout)
                    using (new GUILayout.VerticalScope(GUI.skin.box)) {
                        EditorGUIUtility.hierarchyMode = true;
                        DrawTextureMapping();

                        DrawMargins();

                        DrawGeometrySorting();

                        DrawRichText();

                        DrawRaycastTarget();

                        DrawParsing();

                        DrawKerning();

                        DrawPadding();
                        EditorGUIUtility.hierarchyMode = false;
                    }
            }

            if (m_HavePropertiesChanged) {
                m_HavePropertiesChanged = false;
                m_TextComponent.havePropertiesChanged = true;
                m_TextComponent.ComputeMarginSize();
                EditorUtility.SetDirty(target);
            }

            serializedObject.ApplyModifiedProperties();
        }

        public new void OnSceneGUI() {
            //Override to prevent default scene handles from working, should use layout instead
        }

        protected void DrawFontCustom() {
            // Update list of material presets if needed.
            if (m_IsPresetListDirty)
                m_MaterialPresetNames = GetMaterialPresets();

            // FONT ASSET
            EditorGUI.BeginChangeCheck();
            EditorGUILayout.PropertyField(m_FontAssetProp, k_FontAssetLabel);
            if (EditorGUI.EndChangeCheck()) {
                m_HavePropertiesChanged = true;
                m_HasFontAssetChangedProp.boolValue = true;

                m_IsPresetListDirty = true;
                m_MaterialPresetSelectionIndex = 0;
            }

            Rect rect;

            // MATERIAL PRESET
            if (m_MaterialPresetNames != null) {
                EditorGUI.BeginChangeCheck();
                rect = EditorGUILayout.GetControlRect(false, 17);

                var oldHeight = EditorStyles.popup.fixedHeight;
                EditorStyles.popup.fixedHeight = rect.height;

                var oldSize = EditorStyles.popup.fontSize;
                EditorStyles.popup.fontSize = 11;

                m_MaterialPresetSelectionIndex = EditorGUI.Popup(rect, k_MaterialPresetLabel,
                    m_MaterialPresetSelectionIndex, m_MaterialPresetNames);
                if (EditorGUI.EndChangeCheck()) {
                    m_FontSharedMaterialProp.objectReferenceValue = m_MaterialPresets[m_MaterialPresetSelectionIndex];
                    m_HavePropertiesChanged = true;
                }

                //Make sure material preset selection index matches the selection
                if (m_MaterialPresetSelectionIndex < m_MaterialPresetNames.Length &&
                    m_TargetMaterial != m_MaterialPresets[m_MaterialPresetSelectionIndex] && !m_HavePropertiesChanged)
                    m_IsPresetListDirty = true;

                EditorStyles.popup.fixedHeight = oldHeight;
                EditorStyles.popup.fontSize = oldSize;
            }

            // FONT STYLE
            EditorGUI.BeginChangeCheck();

            int v1, v2, v3, v4, v5, v6, v7;

            if (EditorGUIUtility.wideMode) {
                rect = EditorGUILayout.GetControlRect(true, EditorGUIUtility.singleLineHeight + 2f);

                EditorGUI.PrefixLabel(rect, k_FontStyleLabel);

                var styleValue = m_FontStyleProp.intValue;

                rect.x += EditorGUIUtility.labelWidth;
                rect.width -= EditorGUIUtility.labelWidth;

                rect.width = Mathf.Max(25f, rect.width / 7f);

                v1 = TMP_EditorUtility.EditorToggle(rect, (styleValue & 1) == 1, k_BoldLabel,
                    TMP_UIStyleManager.alignmentButtonLeft)
                    ? 1
                    : 0; // Bold
                rect.x += rect.width;
                v2 = TMP_EditorUtility.EditorToggle(rect, (styleValue & 2) == 2, k_ItalicLabel,
                    TMP_UIStyleManager.alignmentButtonMid)
                    ? 2
                    : 0; // Italics
                rect.x += rect.width;
                v3 = TMP_EditorUtility.EditorToggle(rect, (styleValue & 4) == 4, k_UnderlineLabel,
                    TMP_UIStyleManager.alignmentButtonMid)
                    ? 4
                    : 0; // Underline
                rect.x += rect.width;
                v7 = TMP_EditorUtility.EditorToggle(rect, (styleValue & 64) == 64, k_StrikethroughLabel,
                    TMP_UIStyleManager.alignmentButtonRight)
                    ? 64
                    : 0; // Strikethrough
                rect.x += rect.width;

                var selected = 0;

                EditorGUI.BeginChangeCheck();
                v4 = TMP_EditorUtility.EditorToggle(rect, (styleValue & 8) == 8, k_LowercaseLabel,
                    TMP_UIStyleManager.alignmentButtonLeft)
                    ? 8
                    : 0; // Lowercase
                if (EditorGUI.EndChangeCheck() && v4 > 0) selected = v4;
                rect.x += rect.width;
                EditorGUI.BeginChangeCheck();
                v5 = TMP_EditorUtility.EditorToggle(rect, (styleValue & 16) == 16, k_UppercaseLabel,
                    TMP_UIStyleManager.alignmentButtonMid)
                    ? 16
                    : 0; // Uppercase
                if (EditorGUI.EndChangeCheck() && v5 > 0) selected = v5;
                rect.x += rect.width;
                EditorGUI.BeginChangeCheck();
                v6 = TMP_EditorUtility.EditorToggle(rect, (styleValue & 32) == 32, k_SmallcapsLabel,
                    TMP_UIStyleManager.alignmentButtonRight)
                    ? 32
                    : 0; // Smallcaps
                if (EditorGUI.EndChangeCheck() && v6 > 0) selected = v6;

                if (selected > 0) {
                    v4 = selected == 8 ? 8 : 0;
                    v5 = selected == 16 ? 16 : 0;
                    v6 = selected == 32 ? 32 : 0;
                }
            }
            else {
                rect = EditorGUILayout.GetControlRect(true, EditorGUIUtility.singleLineHeight + 2f);

                EditorGUI.PrefixLabel(rect, k_FontStyleLabel);

                var styleValue = m_FontStyleProp.intValue;

                rect.x += EditorGUIUtility.labelWidth;
                rect.width -= EditorGUIUtility.labelWidth;
                rect.width = Mathf.Max(25f, rect.width / 4f);

                v1 = TMP_EditorUtility.EditorToggle(rect, (styleValue & 1) == 1, k_BoldLabel,
                    TMP_UIStyleManager.alignmentButtonLeft)
                    ? 1
                    : 0; // Bold
                rect.x += rect.width;
                v2 = TMP_EditorUtility.EditorToggle(rect, (styleValue & 2) == 2, k_ItalicLabel,
                    TMP_UIStyleManager.alignmentButtonMid)
                    ? 2
                    : 0; // Italics
                rect.x += rect.width;
                v3 = TMP_EditorUtility.EditorToggle(rect, (styleValue & 4) == 4, k_UnderlineLabel,
                    TMP_UIStyleManager.alignmentButtonMid)
                    ? 4
                    : 0; // Underline
                rect.x += rect.width;
                v7 = TMP_EditorUtility.EditorToggle(rect, (styleValue & 64) == 64, k_StrikethroughLabel,
                    TMP_UIStyleManager.alignmentButtonRight)
                    ? 64
                    : 0; // Strikethrough

                rect = EditorGUILayout.GetControlRect(true, EditorGUIUtility.singleLineHeight + 2f);

                rect.x += EditorGUIUtility.labelWidth;
                rect.width -= EditorGUIUtility.labelWidth;

                rect.width = Mathf.Max(25f, rect.width / 4f);

                var selected = 0;

                EditorGUI.BeginChangeCheck();
                v4 = TMP_EditorUtility.EditorToggle(rect, (styleValue & 8) == 8, k_LowercaseLabel,
                    TMP_UIStyleManager.alignmentButtonLeft)
                    ? 8
                    : 0; // Lowercase
                if (EditorGUI.EndChangeCheck() && v4 > 0) selected = v4;
                rect.x += rect.width;
                EditorGUI.BeginChangeCheck();
                v5 = TMP_EditorUtility.EditorToggle(rect, (styleValue & 16) == 16, k_UppercaseLabel,
                    TMP_UIStyleManager.alignmentButtonMid)
                    ? 16
                    : 0; // Uppercase
                if (EditorGUI.EndChangeCheck() && v5 > 0) selected = v5;
                rect.x += rect.width;
                EditorGUI.BeginChangeCheck();
                v6 = TMP_EditorUtility.EditorToggle(rect, (styleValue & 32) == 32, k_SmallcapsLabel,
                    TMP_UIStyleManager.alignmentButtonRight)
                    ? 32
                    : 0; // Smallcaps
                if (EditorGUI.EndChangeCheck() && v6 > 0) selected = v6;

                if (selected > 0) {
                    v4 = selected == 8 ? 8 : 0;
                    v5 = selected == 16 ? 16 : 0;
                    v6 = selected == 32 ? 32 : 0;
                }
            }

            if (EditorGUI.EndChangeCheck()) {
                m_FontStyleProp.intValue = v1 + v2 + v3 + v4 + v5 + v6 + v7;
                m_HavePropertiesChanged = true;
            }

            // FONT SIZE
            EditorGUI.BeginChangeCheck();

            EditorGUI.BeginDisabledGroup(m_AutoSizingProp.boolValue);
            EditorGUILayout.PropertyField(_fontSizePropOCValue, k_FontSizeLabel,
                GUILayout.MaxWidth(EditorGUIUtility.labelWidth + 70f));
            EditorGUI.EndDisabledGroup();

            if (EditorGUI.EndChangeCheck()) {
                serializedObject.ApplyModifiedProperties();
                foreach (var target in targets) (target as PKUIText).UpdateOCValues();
                serializedObject.Update();

                m_FontSizeBaseProp.floatValue = m_FontSizeProp.floatValue;
                m_HavePropertiesChanged = true;
            }

            EditorGUILayout.Space();
        }

    }
#endif


}
