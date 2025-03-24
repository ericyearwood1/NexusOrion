using System;
using System.Collections.Generic;
using System.Reflection;
using OSIG.Tools.Layout;
using OSIG.Tools.Units;
using UnityEngine.Serialization;

namespace ProtoKit.UI {
    using TMPro;
    using UnityEngine;

    public class PKUIText : TextMeshProUGUI, IOCUnitsContextCache {

        private static readonly string TextSentinelValue = "<<<<UNSERIALIZED_TEXT>>>>";
    private static readonly Color ColorSentinelValue = new Color(1.234f, 2.345f, 3.456f, 4.567f);


    [FormerlySerializedAs("Width")] [SerializeField]
    protected OCValue _width = 100.AsPixels();

    [FormerlySerializedAs("Height")] [SerializeField]
    protected OCValue _height = 100.AsPixels();

    [SerializeField] protected OCValue _fontSizeOCValue = 36.AsPixels();

    [FormerlySerializedAs("Text")] [SerializeField] [HideInInspector]
    protected string _prevSerializedText = TextSentinelValue;

    [FormerlySerializedAs("ColorA")] [SerializeField] [HideInInspector]
    protected Color _prevSerializedColor = ColorSentinelValue;



    protected bool _dirtyWaitingForLayout;
    protected bool _ignoreNextGenerateMesh;
    public OCUnitsContext.Native UnitsContext;

    public OCValue Width {
        get => _width;
        set {
            _width = value;
            GetComponent<RectTransform>()
                .SetSizeWithCurrentAnchors(RectTransform.Axis.Horizontal, value.GetMeters(UnitsContext));
        }
    }

    public OCValue Height {
        get => _height;
        set {
            _height = value;
            GetComponent<RectTransform>()
                .SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, value.GetMeters(UnitsContext));
        }
    }

    public new OCValue fontSize {
        get => _fontSizeOCValue;
        set {
            _fontSizeOCValue = value;
            UpdateOCValues();
        }
    }

    public string Text {
        get => m_text;
        set => text = value;
    }

    public void SetTextFromFloat(float value) {
        Text = value.ToString();
    }

    public void SetTextFromInt(int value) {
        SetTextFromFloat(value);
    }

    public void AppendText(string value) {
        Text += value;
    }

    public void AppendTextAsFloat(float value) {
        AppendText(value.ToString());
    }

    public void AppendTextAsInt(int value) {
        AppendTextAsFloat(value);
    }


    public bool IsCurrentlyDirty => m_havePropertiesChanged || m_isLayoutDirty || _dirtyWaitingForLayout;

    protected override void Awake() {
        base.Awake();

        UpdateOCValues();
    }

#if UNITY_EDITOR
    protected override void Reset() {
        base.Reset();

        Width = 100.AsPixels();
        Height = 100.AsPixels();
        UpdateOCValues();
    }
#endif

    protected override void OnEnable() {
        base.OnEnable();
        UpdateOCValues();
    }

    protected override void OnRectTransformDimensionsChange() {
        base.OnRectTransformDimensionsChange();

        var rt = GetComponent<RectTransform>();

        _width.SetFromMeters(UnitsContext, rt.rect.width);
        _height.SetFromMeters(UnitsContext, rt.rect.height);
    }

    protected override void OnTransformParentChanged() {
        UnitsContext.Dirty();
        UpdateOCValues();

        base.OnTransformParentChanged();
    }

    public virtual void OnApplyLayout(IOCLayoutComponent layoutComponent) {
        _dirtyWaitingForLayout = false;
        _ignoreNextGenerateMesh = true;
    }

    public virtual void OnUnitsContextDirtied() {
        UnitsContext.Dirty();
        UpdateOCValues();
    }

    public virtual void OnBeforeSerialize() {
    }

    public virtual void OnAfterDeserialize() {
        if (_prevSerializedText != TextSentinelValue) {
            m_text = _prevSerializedText;
            _prevSerializedText = TextSentinelValue;
        }

        if (_prevSerializedColor != ColorSentinelValue) {
            m_colorMode = ColorMode.Single;
            m_fontColor = _prevSerializedColor;
            _prevSerializedColor = ColorSentinelValue;
        }
    }

    public void UpdateOCValues() {
        UnitsContext.UpdateIfDirty(gameObject);
        base.fontSize = _fontSizeOCValue.GetMeters(UnitsContext);
    }

    protected override void GenerateTextMesh() {
        if (_ignoreNextGenerateMesh)
            _ignoreNextGenerateMesh = false;
        else
            _dirtyWaitingForLayout = true;

        base.GenerateTextMesh();
    }


    #region CALCULATE PREFERRED VALUES REIMPLEMENTATION

        //We want to specifically modify the rounding behavior of CalculatePreferredValues, but unfortunately the API of
        //TPM does not give us this power.  This code has been copy-pasted from the TMP source and glued back together
        //with reflection until it worked!

        private static readonly BindingFlags m_AllInstance =
            BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;

        private static readonly FieldInfo m_internalCharacterInfoField =
            typeof(TMP_Text).GetField("m_internalCharacterInfo", m_AllInstance);

        private static readonly FieldInfo m_TextProcessingArrayField =
            typeof(TMP_Text).GetField("m_TextProcessingArray", m_AllInstance);

        private static readonly FieldInfo m_unicodeCharField =
            m_TextProcessingArrayField.FieldType.GetElementType().GetField("unicode", m_AllInstance);

        private static readonly FieldInfo m_glyphPairAdjustmentRecordLookupDictionaryField =
            typeof(TMP_FontFeatureTable).GetField("m_GlyphPairAdjustmentRecordLookupDictionary", m_AllInstance);

        private static readonly FieldInfo m_adjustedAscenderField =
            typeof(TMP_CharacterInfo).GetField("adjustedAscender", m_AllInstance);

        private static readonly FieldInfo m_adjustedDescenderField =
            typeof(TMP_CharacterInfo).GetField("adjustedDescender", m_AllInstance);

        private static readonly MethodInfo m_validateHtmlTag =
            typeof(TMP_Text).GetMethod("ValidateHtmlTag", BindingFlags.NonPublic | BindingFlags.Instance);

        private TMP_CharacterInfo[] m_internalCharacterInfo {
            get => m_internalCharacterInfoField.GetValue(this) as TMP_CharacterInfo[];
            set => m_internalCharacterInfoField.SetValue(this, value);
        }

        private Array m_TextProcessingArray {
            get => m_TextProcessingArrayField.GetValue(this) as Array;
            set => m_TextProcessingArrayField.SetValue(this, value);
        }

        private int GetUnicode(object unicodeChar) {
            return (int)m_unicodeCharField.GetValue(unicodeChar);
        }

        private Dictionary<uint, TMP_GlyphPairAdjustmentRecord> GetGlyphPairAdjustmentRecordLookupDictionary(
            TMP_FontFeatureTable featureTable) {
            return m_glyphPairAdjustmentRecordLookupDictionaryField.GetValue(featureTable) as
                Dictionary<uint, TMP_GlyphPairAdjustmentRecord>;
        }

        private float GetAdjustedAscender(TMP_CharacterInfo charInfo) {
            return (float)m_adjustedAscenderField.GetValue(charInfo);
        }

        private void SetAdjustedAscender(TMP_CharacterInfo charInfo, float value) {
            m_adjustedAscenderField.SetValue(charInfo, value);
        }

        private float GetAdjustedDescender(TMP_CharacterInfo charInfo) {
            return (float)m_adjustedDescenderField.GetValue(charInfo);
        }

        private void SetAdjustedDescender(TMP_CharacterInfo charInfo, float value) {
            m_adjustedDescenderField.SetValue(charInfo, value);
        }

        private bool ValidateHtmlTagReflection(Array array, int i, out int index) {
            //TODO: can this be cleaned up with MethodInfo.CreateDelegate to reduce allocations?
            var args = new object[3];
            args[0] = array;
            args[1] = i;
            var result = (bool)m_validateHtmlTag.Invoke(this, args);
            index = (int)args[2];
            return result;
        }


        protected override Vector2 CalculatePreferredValues(ref float fontSize, Vector2 marginSize,
            bool isTextAutoSizingEnabled, bool isWordWrappingEnabled) {

            // Early exit if no font asset was assigned. This should not be needed since LiberationSans SDF will be assigned by default.
            if (m_fontAsset == null || m_fontAsset.characterLookupTable == null) {
                Debug.LogWarning(
                    "Can't Generate Mesh! No Font Asset has been assigned to Object ID: " + GetInstanceID());

                m_IsAutoSizePointSizeSet = true;
                return Vector2.zero;
            }

            // Early exit if we don't have any Text to generate.

            if (m_TextProcessingArray == null || m_TextProcessingArray.Length == 0 ||
                GetUnicode(m_TextProcessingArray.GetValue(0)) == (char)0) {
                m_IsAutoSizePointSizeSet = true;
                return Vector2.zero;
            }

            m_currentFontAsset = m_fontAsset;
            m_currentMaterial = m_sharedMaterial;
            m_currentMaterialIndex = 0;
            m_materialReferenceStack.SetDefault(new MaterialReference(0, m_currentFontAsset, null, m_currentMaterial,
                m_padding));

            // Total character count is computed when the text is parsed.
            var totalCharacterCount = m_totalCharacterCount; // m_VisibleCharacters.Count;

            if (m_internalCharacterInfo == null || totalCharacterCount > m_internalCharacterInfo.Length)
                m_internalCharacterInfo = new TMP_CharacterInfo[totalCharacterCount > 1024
                    ? totalCharacterCount + 256
                    : Mathf.NextPowerOfTwo(totalCharacterCount)];

            // Calculate the scale of the font based on selected font size and sampling point size.
            // baseScale is calculated using the font asset assigned to the text object.
            var baseScale = fontSize / m_fontAsset.faceInfo.pointSize * m_fontAsset.faceInfo.scale *
                            (m_isOrthographic ? 1 : 0.1f);
            var currentElementScale = baseScale;
            var currentEmScale = fontSize * 0.01f * (m_isOrthographic ? 1 : 0.1f);
            m_fontScaleMultiplier = 1;

            m_currentFontSize = fontSize;
            m_sizeStack.SetDefault(m_currentFontSize);
            float fontSizeDelta = 0;

            m_FontStyleInternal = m_fontStyle; // Set the default style.

            m_lineJustification =
                m_HorizontalAlignment; // m_textAlignment; // Sets the line justification mode to match editor alignment.
            m_lineJustificationStack.SetDefault(m_lineJustification);

            m_baselineOffset = 0; // Used by subscript characters.
            m_baselineOffsetStack.Clear();

            m_lineOffset = 0; // Amount of space between lines (font line spacing + m_linespacing).
            m_lineHeight = TMP_Math.FLOAT_UNSET;
            var lineGap = m_currentFontAsset.faceInfo.lineHeight -
                          (m_currentFontAsset.faceInfo.ascentLine - m_currentFontAsset.faceInfo.descentLine);

            m_cSpacing = 0; // Amount of space added between characters as a result of the use of the <cspace> tag.
            m_monoSpacing = 0;
            //float lineOffsetDelta = 0;
            m_xAdvance = 0; // Used to track the position of each character.
            float maxXAdvance = 0; // Used to determine Preferred Width.

            tag_LineIndent = 0; // Used for indentation of text.
            tag_Indent = 0;
            m_indentStack.SetDefault(0);
            tag_NoParsing = false;
            //m_isIgnoringAlignment = false;

            m_characterCount = 0; // Total characters in the char[]


            // Tracking of line information
            m_firstCharacterOfLine = 0;
            m_maxLineAscender = k_LargeNegativeFloat;
            m_maxLineDescender = k_LargePositiveFloat;
            m_lineNumber = 0;
            m_startOfLineAscender = 0;
            m_IsDrivenLineSpacing = false;

            var marginWidth = marginSize.x;
            var marginHeight = marginSize.y;
            m_marginLeft = 0;
            m_marginRight = 0;

            float lineMarginLeft = 0;
            float lineMarginRight = 0;

            m_width = -1;
            var widthOfTextArea = marginWidth + 0.0001f - m_marginLeft - m_marginRight;

            // Used by Unity's Auto Layout system.
            float renderedWidth = 0;
            float renderedHeight = 0;
            float textWidth = 0;
            m_isCalculatingPreferredValues = true;

            // Tracking of the highest Ascender
            m_maxCapHeight = 0;
            m_maxTextAscender = 0;
            m_ElementDescender = 0;
            float maxVisibleDescender = 0;
            var isMaxVisibleDescenderSet = false;

            // Initialize struct to track states of word wrapping
            var isFirstWordOfLine = true;
            m_isNonBreakingSpace = false;
            //bool isLastBreakingChar = false;
            var isLastCharacterCJK = false;
            //int lastSoftLineBreak = 0;

            var characterToSubstitute = new CharacterSubstitution(-1, 0);
            var isSoftHyphenIgnored = false;

            var internalWordWrapState = new WordWrapState();
            var internalLineState = new WordWrapState();
            var internalSoftLineBreak = new WordWrapState();

            // Counter to prevent recursive lockup when computing preferred values.
            m_AutoSizeIterationCount += 1;

            // Parse through Character buffer to read HTML tags and begin creating mesh.
            for (var i = 0;
                 i < m_TextProcessingArray.Length && GetUnicode(m_TextProcessingArray.GetValue(i)) != 0;
                 i++) {
                var charCode = GetUnicode(m_TextProcessingArray.GetValue(i));

                // Parse Rich Text Tag

                #region Parse Rich Text Tag

                if (m_isRichText && charCode == 60) // '<'
                {
                    m_isParsingText = true;
                    m_textElementType = TMP_TextElementType.Character;
                    int endTagIndex;

                    // Check if Tag is valid. If valid, skip to the end of the validated tag.
                    if (ValidateHtmlTagReflection(m_TextProcessingArray, i + 1, out endTagIndex)) {
                        i = endTagIndex;

                        // Continue to next character or handle the sprite element
                        if (m_textElementType == TMP_TextElementType.Character)
                            continue;
                    }
                }
                else {
                    m_textElementType = m_textInfo.characterInfo[m_characterCount].elementType;
                    m_currentMaterialIndex = m_textInfo.characterInfo[m_characterCount].materialReferenceIndex;
                    m_currentFontAsset = m_textInfo.characterInfo[m_characterCount].fontAsset;
                }

                #endregion End Parse Rich Text Tag

                var prev_MaterialIndex = m_currentMaterialIndex;
                var isUsingAltTypeface = m_textInfo.characterInfo[m_characterCount].isUsingAlternateTypeface;

                m_isParsingText = false;

                // Handle potential character substitutions

                #region Character Substitutions

                var isInjectingCharacter = false;

                if (characterToSubstitute.index == m_characterCount) {
                    charCode = (int)characterToSubstitute.unicode;
                    m_textElementType = TMP_TextElementType.Character;
                    isInjectingCharacter = true;

                    switch (charCode) {
                        case 0x03:
                            m_internalCharacterInfo[m_characterCount].textElement =
                                m_currentFontAsset.characterLookupTable[0x03];
                            m_isTextTruncated = true;
                            break;
                        case 0x2D:
                            //
                            break;
                        case 0x2026:
                            m_internalCharacterInfo[m_characterCount].textElement = m_Ellipsis.character;
                            ;
                            m_internalCharacterInfo[m_characterCount].elementType = TMP_TextElementType.Character;
                            m_internalCharacterInfo[m_characterCount].fontAsset = m_Ellipsis.fontAsset;
                            m_internalCharacterInfo[m_characterCount].material = m_Ellipsis.material;
                            m_internalCharacterInfo[m_characterCount].materialReferenceIndex = m_Ellipsis.materialIndex;

                            // Indicates the source parsing data has been modified.
                            m_isTextTruncated = true;

                            // End Of Text
                            characterToSubstitute.index = m_characterCount + 1;
                            characterToSubstitute.unicode = 0x03;
                            break;
                    }
                }

                #endregion


                // When using Linked text, mark character as ignored and skip to next character.

                #region Linked Text

                if (m_characterCount < m_firstVisibleCharacter && charCode != 0x03) {
                    m_internalCharacterInfo[m_characterCount].isVisible = false;
                    m_internalCharacterInfo[m_characterCount].character = (char)0x200B;
                    m_internalCharacterInfo[m_characterCount].lineNumber = 0;
                    m_characterCount += 1;
                    continue;
                }

                #endregion


                // Handle Font Styles like LowerCase, UpperCase and SmallCaps.

                #region Handling of LowerCase, UpperCase and SmallCaps Font Styles

                var smallCapsMultiplier = 1.0f;

                if (m_textElementType == TMP_TextElementType.Character) {
                    if ( /*(m_fontStyle & FontStyles.UpperCase) == FontStyles.UpperCase ||*/
                        (m_FontStyleInternal & FontStyles.UpperCase) == FontStyles.UpperCase) {
                        // If this character is lowercase, switch to uppercase.
                        if (char.IsLower((char)charCode))
                            charCode = char.ToUpper((char)charCode);
                    }
                    else if ( /*(m_fontStyle & FontStyles.LowerCase) == FontStyles.LowerCase ||*/
                             (m_FontStyleInternal & FontStyles.LowerCase) == FontStyles.LowerCase) {
                        // If this character is uppercase, switch to lowercase.
                        if (char.IsUpper((char)charCode))
                            charCode = char.ToLower((char)charCode);
                    }
                    else if ( /*(m_fontStyle & FontStyles.SmallCaps) == FontStyles.SmallCaps ||*/
                             (m_FontStyleInternal & FontStyles.SmallCaps) == FontStyles.SmallCaps) {
                        if (char.IsLower((char)charCode)) {
                            smallCapsMultiplier = 0.8f;
                            charCode = char.ToUpper((char)charCode);
                        }
                    }
                }

                #endregion


                // Look up Character Data from Dictionary and cache it.

                #region Look up Character Data

                //float baselineOffset = 0;
                float elementAscentLine = 0;
                float elementDescentLine = 0;
                if (m_textElementType == TMP_TextElementType.Sprite) {
                    // If a sprite is used as a fallback then get a reference to it and set the color to white.
                    m_currentSpriteAsset = m_textInfo.characterInfo[m_characterCount].spriteAsset;
                    m_spriteIndex = m_textInfo.characterInfo[m_characterCount].spriteIndex;

                    var sprite = m_currentSpriteAsset.spriteCharacterTable[m_spriteIndex];
                    if (sprite == null) continue;

                    // Sprites are assigned in the E000 Private Area + sprite Index
                    if (charCode == 60)
                        charCode = 57344 + m_spriteIndex;

                    // The sprite scale calculations are based on the font asset assigned to the text object.
                    if (m_currentSpriteAsset.faceInfo.pointSize > 0) {
                        var spriteScale = m_currentFontSize / m_currentSpriteAsset.faceInfo.pointSize *
                                          m_currentSpriteAsset.faceInfo.scale * (m_isOrthographic ? 1 : 0.1f);
                        currentElementScale = sprite.scale * sprite.glyph.scale * spriteScale;
                        elementAscentLine = m_currentSpriteAsset.faceInfo.ascentLine;
                        //baselineOffset = m_currentSpriteAsset.faceInfo.baseline * m_fontScale * m_fontScaleMultiplier * m_currentSpriteAsset.faceInfo.scale;
                        elementDescentLine = m_currentSpriteAsset.faceInfo.descentLine;
                    }
                    else {
                        var spriteScale = m_currentFontSize / m_currentFontAsset.faceInfo.pointSize *
                                          m_currentFontAsset.faceInfo.scale * (m_isOrthographic ? 1 : 0.1f);
                        currentElementScale = m_currentFontAsset.faceInfo.ascentLine / sprite.glyph.metrics.height *
                                              sprite.scale * sprite.glyph.scale * spriteScale;
                        var scaleDelta = spriteScale / currentElementScale;
                        elementAscentLine = m_currentFontAsset.faceInfo.ascentLine * scaleDelta;
                        //baselineOffset = m_currentFontAsset.faceInfo.baseline * m_fontScale * m_fontScaleMultiplier * m_currentFontAsset.faceInfo.scale;
                        elementDescentLine = m_currentFontAsset.faceInfo.descentLine * scaleDelta;
                    }

                    m_cached_TextElement = sprite;

                    m_internalCharacterInfo[m_characterCount].elementType = TMP_TextElementType.Sprite;
                    m_internalCharacterInfo[m_characterCount].scale = currentElementScale;

                    m_currentMaterialIndex = prev_MaterialIndex;
                }
                else if (m_textElementType == TMP_TextElementType.Character) {
                    m_cached_TextElement = m_textInfo.characterInfo[m_characterCount].textElement;
                    if (m_cached_TextElement == null) continue;

                    m_currentMaterialIndex = m_textInfo.characterInfo[m_characterCount].materialReferenceIndex;

                    float adjustedScale;
                    if (isInjectingCharacter && GetUnicode(m_TextProcessingArray.GetValue(i)) == 0x0A &&
                        m_characterCount != m_firstCharacterOfLine)
                        adjustedScale = m_textInfo.characterInfo[m_characterCount - 1].pointSize * smallCapsMultiplier /
                                        m_currentFontAsset.faceInfo.pointSize * m_currentFontAsset.faceInfo.scale *
                                        (m_isOrthographic ? 1 : 0.1f);
                    else
                        adjustedScale = m_currentFontSize * smallCapsMultiplier /
                                        m_currentFontAsset.faceInfo.pointSize *
                                        m_currentFontAsset.faceInfo.scale * (m_isOrthographic ? 1 : 0.1f);

                    // Special handling for injected Ellipsis
                    if (isInjectingCharacter && charCode == 0x2026) {
                        elementAscentLine = 0;
                        elementDescentLine = 0;
                    }
                    else {
                        elementAscentLine = m_currentFontAsset.faceInfo.ascentLine;
                        elementDescentLine = m_currentFontAsset.faceInfo.descentLine;
                    }

                    currentElementScale = adjustedScale * m_fontScaleMultiplier * m_cached_TextElement.scale;
                    //baselineOffset = m_currentFontAsset.faceInfo.baseline * m_fontScale * m_fontScaleMultiplier * m_currentFontAsset.faceInfo.scale;

                    m_internalCharacterInfo[m_characterCount].elementType = TMP_TextElementType.Character;
                }

                #endregion


                // Handle Soft Hyphen

                #region Handle Soft Hyphen

                var currentElementUnmodifiedScale = currentElementScale;
                if (charCode == 0xAD || charCode == 0x03)
                    currentElementScale = 0;

                #endregion


                // Store some of the text object's information
                m_internalCharacterInfo[m_characterCount].character = (char)charCode;

                // Cache glyph metrics
                var currentGlyphMetrics = m_cached_TextElement.glyph.metrics;

                // Optimization to avoid calling this more than once per character.
                var isWhiteSpace = charCode <= 0xFFFF && char.IsWhiteSpace((char)charCode);


                // Handle Kerning if Enabled.

                #region Handle Kerning

                var glyphAdjustments = new TMP_GlyphValueRecord();
                var characterSpacingAdjustment = m_characterSpacing;
                m_GlyphHorizontalAdvanceAdjustment = 0;
                if (m_enableKerning) {
                    TMP_GlyphPairAdjustmentRecord adjustmentPair;
                    var baseGlyphIndex = m_cached_TextElement.glyphIndex;

                    if (m_characterCount < totalCharacterCount - 1) {
                        var nextGlyphIndex = m_textInfo.characterInfo[m_characterCount + 1].textElement.glyphIndex;
                        var key = (nextGlyphIndex << 16) | baseGlyphIndex;

                        if (GetGlyphPairAdjustmentRecordLookupDictionary(m_currentFontAsset.fontFeatureTable)
                            .TryGetValue(key, out adjustmentPair)) {
                            glyphAdjustments = adjustmentPair.firstAdjustmentRecord.glyphValueRecord;
                            characterSpacingAdjustment =
                                (adjustmentPair.featureLookupFlags & FontFeatureLookupFlags.IgnoreSpacingAdjustments) ==
                                FontFeatureLookupFlags.IgnoreSpacingAdjustments
                                    ? 0
                                    : characterSpacingAdjustment;
                        }
                    }

                    if (m_characterCount >= 1) {
                        var previousGlyphIndex = m_textInfo.characterInfo[m_characterCount - 1].textElement.glyphIndex;
                        var key = (baseGlyphIndex << 16) | previousGlyphIndex;

                        if (GetGlyphPairAdjustmentRecordLookupDictionary(m_currentFontAsset.fontFeatureTable)
                            .TryGetValue(key, out adjustmentPair)) {
                            glyphAdjustments += adjustmentPair.secondAdjustmentRecord.glyphValueRecord;
                            characterSpacingAdjustment =
                                (adjustmentPair.featureLookupFlags & FontFeatureLookupFlags.IgnoreSpacingAdjustments) ==
                                FontFeatureLookupFlags.IgnoreSpacingAdjustments
                                    ? 0
                                    : characterSpacingAdjustment;
                        }
                    }

                    m_GlyphHorizontalAdvanceAdjustment = glyphAdjustments.xAdvance;
                }

                #endregion


                // Initial Implementation for RTL support.

                #region Handle Right-to-Left

                //if (m_isRightToLeft)
                //{
                //    m_xAdvance -= ((m_cached_TextElement.xAdvance * bold_xAdvance_multiplier + m_characterSpacing + m_wordSpacing + m_currentFontAsset.normalSpacingOffset) * currentElementScale + m_cSpacing) * (1 - m_charWidthAdjDelta);

                //    if (char.IsWhiteSpace((char)charCode) || charCode == 0x200B)
                //        m_xAdvance -= m_wordSpacing * currentElementScale;
                //}

                #endregion


                // Handle Mono Spacing

                #region Handle Mono Spacing

                float monoAdvance = 0;
                if (m_monoSpacing != 0) {
                    monoAdvance =
                        (m_monoSpacing / 2 -
                         (m_cached_TextElement.glyph.metrics.width / 2 +
                          m_cached_TextElement.glyph.metrics.horizontalBearingX) * currentElementScale) *
                        (1 - m_charWidthAdjDelta);
                    m_xAdvance += monoAdvance;
                }

                #endregion


                // Set Padding based on selected font style

                #region Handle Style Padding

                float boldSpacingAdjustment = 0;
                if (m_textElementType == TMP_TextElementType.Character && !isUsingAltTypeface &&
                    (m_FontStyleInternal & FontStyles.Bold) ==
                    FontStyles.Bold) // Checks for any combination of Bold Style.
                    boldSpacingAdjustment = m_currentFontAsset.boldSpacing;

                #endregion Handle Style Padding

                m_internalCharacterInfo[m_characterCount].baseLine = 0 - m_lineOffset + m_baselineOffset;

                // Compute text metrics

                #region Compute Ascender & Descender values

                // Element Ascender in line space
                var elementAscender = m_textElementType == TMP_TextElementType.Character
                    ? elementAscentLine * currentElementScale / smallCapsMultiplier + m_baselineOffset
                    : elementAscentLine * currentElementScale + m_baselineOffset;

                // Element Descender in line space
                var elementDescender = m_textElementType == TMP_TextElementType.Character
                    ? elementDescentLine * currentElementScale / smallCapsMultiplier + m_baselineOffset
                    : elementDescentLine * currentElementScale + m_baselineOffset;

                var adjustedAscender = elementAscender;
                var adjustedDescender = elementDescender;

                var isFirstCharacterOfLine = m_characterCount == m_firstCharacterOfLine;
                // Max line ascender and descender in line space
                if (isFirstCharacterOfLine || isWhiteSpace == false) {
                    // Special handling for Superscript and Subscript where we use the unadjusted line ascender and descender
                    if (m_baselineOffset != 0) {
                        adjustedAscender = Mathf.Max((elementAscender - m_baselineOffset) / m_fontScaleMultiplier,
                            adjustedAscender);
                        adjustedDescender = Mathf.Min((elementDescender - m_baselineOffset) / m_fontScaleMultiplier,
                            adjustedDescender);
                    }

                    m_maxLineAscender = Mathf.Max(adjustedAscender, m_maxLineAscender);
                    m_maxLineDescender = Mathf.Min(adjustedDescender, m_maxLineDescender);
                }

                // Element Ascender and Descender in object space
                if (isFirstCharacterOfLine || isWhiteSpace == false) {
                    SetAdjustedAscender(m_internalCharacterInfo[m_characterCount], adjustedAscender);
                    SetAdjustedDescender(m_internalCharacterInfo[m_characterCount], adjustedDescender);

                    m_ElementAscender = m_internalCharacterInfo[m_characterCount].ascender =
                        elementAscender - m_lineOffset;
                    m_ElementDescender = m_internalCharacterInfo[m_characterCount].descender =
                        elementDescender - m_lineOffset;
                }
                else {
                    SetAdjustedAscender(m_internalCharacterInfo[m_characterCount], m_maxLineAscender);
                    SetAdjustedDescender(m_internalCharacterInfo[m_characterCount], m_maxLineDescender);

                    m_ElementAscender = m_internalCharacterInfo[m_characterCount].ascender =
                        m_maxLineAscender - m_lineOffset;
                    m_ElementDescender = m_internalCharacterInfo[m_characterCount].descender =
                        m_maxLineDescender - m_lineOffset;
                }

                // Max text object ascender and cap height
                if (m_lineNumber == 0 || m_isNewPage)
                    if (isFirstCharacterOfLine || isWhiteSpace == false) {
                        m_maxTextAscender = m_maxLineAscender;
                        m_maxCapHeight = Mathf.Max(m_maxCapHeight,
                            m_currentFontAsset.faceInfo.capLine * currentElementScale / smallCapsMultiplier);
                    }

                // Page ascender
                if (m_lineOffset == 0)
                    if (!isWhiteSpace || m_characterCount == m_firstCharacterOfLine)
                        m_PageAscender = m_PageAscender > elementAscender ? m_PageAscender : elementAscender;

                #endregion

                var isJustifiedOrFlush =
                    (m_lineJustification & HorizontalAlignmentOptions.Flush) == HorizontalAlignmentOptions.Flush ||
                    (m_lineJustification & HorizontalAlignmentOptions.Justified) ==
                    HorizontalAlignmentOptions.Justified;

                // Setup Mesh for visible text elements. ie. not a SPACE / LINEFEED / CARRIAGE RETURN.

                #region Handle Visible Characters

                if (charCode == 9 ||
                    (isWhiteSpace == false && charCode != 0x200B && charCode != 0xAD && charCode != 0x03) ||
                    (charCode == 0xAD && isSoftHyphenIgnored == false) ||
                    m_textElementType == TMP_TextElementType.Sprite) {
                    //float marginLeft = m_marginLeft;
                    //float marginRight = m_marginRight;

                    // Injected characters do not override margins
                    //if (isInjectingCharacter)
                    //{
                    //    marginLeft = m_textInfo.lineInfo[m_lineNumber].marginLeft;
                    //    marginRight = m_textInfo.lineInfo[m_lineNumber].marginRight;
                    //}

                    widthOfTextArea = m_width != -1
                        ? Mathf.Min(marginWidth + 0.0001f - m_marginLeft - m_marginRight, m_width)
                        : marginWidth + 0.0001f - m_marginLeft - m_marginRight;

                    // Calculate the line breaking width of the text.
                    textWidth = Mathf.Abs(m_xAdvance) + currentGlyphMetrics.horizontalAdvance *
                        (1 - m_charWidthAdjDelta) *
                        (charCode == 0xAD ? currentElementUnmodifiedScale : currentElementScale);

                    var testedCharacterCount = m_characterCount;

                    // Handling of Horizontal Bounds

                    #region Current Line Horizontal Bounds Check

                    if (textWidth >
                        widthOfTextArea *
                        (isJustifiedOrFlush ? 1.05f : 1.0f)) // Handle Line Breaking (if still possible)
                        if (isWordWrappingEnabled &&
                            m_characterCount != m_firstCharacterOfLine) // && isFirstWord == false)
                        {
                            // Restore state to previous safe line breaking
                            i = RestoreWordWrappingState(ref internalWordWrapState);

                            // Replace Soft Hyphen by Hyphen Minus 0x2D

                            #region Handle Soft Hyphenation

                            if (m_internalCharacterInfo[m_characterCount - 1].character == 0xAD &&
                                isSoftHyphenIgnored == false && m_overflowMode == TextOverflowModes.Overflow) {
                                characterToSubstitute.index = m_characterCount - 1;
                                characterToSubstitute.unicode = 0x2D;

                                i -= 1;
                                m_characterCount -= 1;
                                continue;
                            }

                            isSoftHyphenIgnored = false;

                            // Ignore Soft Hyphen to prevent it from wrapping
                            if (m_internalCharacterInfo[m_characterCount].character == 0xAD) {
                                isSoftHyphenIgnored = true;
                                continue;
                            }

                            #endregion

                            // Adjust character spacing before breaking up word if auto size is enabled

                            #region Handle Text Auto Size (if word wrapping is no longer possible)

                            if (isTextAutoSizingEnabled && isFirstWordOfLine) {
                                // Handle Character Width Adjustments

                                #region Character Width Adjustments

                                if (m_charWidthAdjDelta < m_charWidthMaxAdj / 100 &&
                                    m_AutoSizeIterationCount < m_AutoSizeMaxIterationCount) {
                                    var adjustedTextWidth = textWidth;

                                    // Determine full width of the text
                                    if (m_charWidthAdjDelta > 0)
                                        adjustedTextWidth /= 1f - m_charWidthAdjDelta;

                                    var adjustmentDelta = textWidth -
                                                          (widthOfTextArea - 0.0001f) *
                                                          (isJustifiedOrFlush ? 1.05f : 1.0f);
                                    m_charWidthAdjDelta += adjustmentDelta / adjustedTextWidth;
                                    m_charWidthAdjDelta = Mathf.Min(m_charWidthAdjDelta, m_charWidthMaxAdj / 100);

                                    //Debug.Log("[" + m_AutoSizeIterationCount + "] Reducing Character Width by " + (m_charWidthAdjDelta * 100) + "%");
                                    return Vector2.zero;
                                }

                                #endregion

                                // Handle Text Auto-sizing resulting from text exceeding vertical bounds.

                                #region Text Auto-Sizing (Text greater than vertical bounds)

                                if (fontSize > m_fontSizeMin &&
                                    m_AutoSizeIterationCount < m_AutoSizeMaxIterationCount) {
                                    m_maxFontSize = fontSize;

                                    var sizeDelta = Mathf.Max((fontSize - m_minFontSize) / 2, 0.05f);
                                    fontSize -= sizeDelta;
                                    fontSize = Mathf.Max((int)(fontSize * 20 + 0.5f) / 20f, m_fontSizeMin);

                                    //Debug.Log("[" + m_AutoSizeIterationCount + "] Reducing Point Size from [" + m_maxFontSize.ToString("f3") + "] to [" + m_fontSize.ToString("f3") + "] with delta of [" + sizeDelta.ToString("f3") + "].");
                                    return Vector2.zero;
                                }

                                #endregion Text Auto-Sizing
                            }

                            #endregion

                            // Adjust line spacing if necessary
                            var baselineAdjustmentDelta = m_maxLineAscender - m_startOfLineAscender;
                            if (m_lineOffset > 0 && Math.Abs(baselineAdjustmentDelta) > 0.01f &&
                                m_IsDrivenLineSpacing == false && !m_isNewPage) {
                                //AdjustLineOffset(m_firstCharacterOfLine, m_characterCount, baselineAdjustmentDelta);
                                m_ElementDescender -= baselineAdjustmentDelta;
                                m_lineOffset += baselineAdjustmentDelta;
                            }

                            // Calculate line ascender and make sure if last character is superscript or subscript that we check that as well.
                            var lineAscender = m_maxLineAscender - m_lineOffset;
                            var lineDescender = m_maxLineDescender - m_lineOffset;

                            // Update maxDescender and maxVisibleDescender
                            m_ElementDescender =
                                m_ElementDescender < lineDescender ? m_ElementDescender : lineDescender;
                            if (!isMaxVisibleDescenderSet)
                                maxVisibleDescender = m_ElementDescender;

                            if (m_useMaxVisibleDescender && (m_characterCount >= m_maxVisibleCharacters ||
                                                             m_lineNumber >= m_maxVisibleLines))
                                isMaxVisibleDescenderSet = true;

                            // Store first character of the next line.
                            m_firstCharacterOfLine = m_characterCount;
                            m_lineVisibleCharacterCount = 0;

                            // Compute Preferred Width & Height
                            renderedWidth += m_xAdvance;

                            if (isWordWrappingEnabled)
                                renderedHeight = m_maxTextAscender - m_ElementDescender;
                            else
                                renderedHeight = Mathf.Max(renderedHeight, lineAscender - lineDescender);

                            // Store the state of the line before starting on the new line.
                            SaveWordWrappingState(ref internalLineState, i, m_characterCount - 1);

                            m_lineNumber += 1;

                            var ascender = GetAdjustedAscender(m_internalCharacterInfo[m_characterCount]);

                            // Compute potential new line offset in the event a line break is needed.
                            if (m_lineHeight == TMP_Math.FLOAT_UNSET) {
                                m_lineOffset += 0 - m_maxLineDescender + ascender +
                                                (lineGap + m_lineSpacingDelta) * baseScale +
                                                m_lineSpacing * currentEmScale;
                                m_IsDrivenLineSpacing = false;
                            }
                            else {
                                m_lineOffset += m_lineHeight + m_lineSpacing * currentEmScale;
                                m_IsDrivenLineSpacing = true;
                            }

                            m_maxLineAscender = k_LargeNegativeFloat;
                            m_maxLineDescender = k_LargePositiveFloat;
                            m_startOfLineAscender = ascender;

                            m_xAdvance = 0 + tag_Indent;
                            //isStartOfNewLine = true;
                            isFirstWordOfLine = true;
                            continue;
                        }

                    #endregion

                    lineMarginLeft = m_marginLeft;
                    lineMarginRight = m_marginRight;
                }

                #endregion Handle Visible Characters


                // Check if Line Spacing of previous line needs to be adjusted.

                #region Adjust Line Spacing

                /*if (m_lineOffset > 0 && !TMP_Math.Approximately(m_maxLineAscender, m_startOfLineAscender) && m_IsDrivenLineSpacing == false && !m_isNewPage)
                {
                    float offsetDelta = m_maxLineAscender - m_startOfLineAscender;
                    //AdjustLineOffset(m_firstCharacterOfLine, m_characterCount, offsetDelta);
                    m_ElementDescender -= offsetDelta;
                    m_lineOffset += offsetDelta;

                    m_startOfLineAscender += offsetDelta;
                    internalWordWrapState.lineOffset = m_lineOffset;
                    internalWordWrapState.startOfLineAscender = m_startOfLineAscender;
                }*/

                #endregion


                // Handle xAdvance & Tabulation Stops. Tab stops at every 25% of Font Size.

                #region XAdvance, Tabulation & Stops

                if (charCode == 9) {
                    var tabSize = m_currentFontAsset.faceInfo.tabWidth * m_currentFontAsset.tabSize *
                                  currentElementScale;
                    var tabs = Mathf.Ceil(m_xAdvance / tabSize) * tabSize;
                    m_xAdvance = tabs > m_xAdvance ? tabs : m_xAdvance + tabSize;
                }
                else if (m_monoSpacing != 0) {
                    m_xAdvance +=
                        (m_monoSpacing - monoAdvance +
                         (m_currentFontAsset.normalSpacingOffset + characterSpacingAdjustment) * currentEmScale +
                         m_cSpacing) * (1 - m_charWidthAdjDelta);

                    if (isWhiteSpace || charCode == 0x200B)
                        m_xAdvance += m_wordSpacing * currentEmScale;
                }
                else {
                    m_xAdvance +=
                        ((currentGlyphMetrics.horizontalAdvance + glyphAdjustments.xAdvance) * currentElementScale +
                         (m_currentFontAsset.normalSpacingOffset + characterSpacingAdjustment + boldSpacingAdjustment) *
                         currentEmScale + m_cSpacing) * (1 - m_charWidthAdjDelta);

                    if (isWhiteSpace || charCode == 0x200B)
                        m_xAdvance += m_wordSpacing * currentEmScale;
                }

                #endregion Tabulation & Stops


                // Handle Carriage Return

                #region Carriage Return

                if (charCode == 13) {
                    maxXAdvance = Mathf.Max(maxXAdvance, renderedWidth + m_xAdvance);
                    renderedWidth = 0;
                    m_xAdvance = 0 + tag_Indent;
                }

                #endregion Carriage Return


                // Handle Line Spacing Adjustments + Word Wrapping & special case for last line.

                #region Check for Line Feed and Last Character

                if (charCode == 10 || charCode == 11 || charCode == 0x03 || charCode == 0x2028 || charCode == 0x2029 ||
                    m_characterCount == totalCharacterCount - 1) {
                    // Check if Line Spacing of previous line needs to be adjusted.
                    var baselineAdjustmentDelta = m_maxLineAscender - m_startOfLineAscender;
                    if (m_lineOffset > 0 && Math.Abs(baselineAdjustmentDelta) > 0.01f &&
                        m_IsDrivenLineSpacing == false &&
                        !m_isNewPage) {
                        m_ElementDescender -= baselineAdjustmentDelta;
                        m_lineOffset += baselineAdjustmentDelta;
                    }

                    m_isNewPage = false;

                    // Calculate lineAscender & make sure if last character is superscript or subscript that we check that as well.
                    //float lineAscender = m_maxLineAscender - m_lineOffset;
                    var lineDescender = m_maxLineDescender - m_lineOffset;

                    // Update maxDescender and maxVisibleDescender
                    m_ElementDescender = m_ElementDescender < lineDescender ? m_ElementDescender : lineDescender;

                    // Store PreferredWidth paying attention to linefeed and last character of text.
                    if (m_characterCount == totalCharacterCount - 1) {
                        renderedWidth = Mathf.Max(maxXAdvance,
                            renderedWidth + textWidth + lineMarginLeft + lineMarginRight);
                    }
                    else {
                        maxXAdvance = Mathf.Max(maxXAdvance,
                            renderedWidth + textWidth + lineMarginLeft + lineMarginRight);
                        renderedWidth = 0;
                    }

                    renderedHeight = m_maxTextAscender - m_ElementDescender;

                    // Add new line if not last lines or character.
                    if (charCode == 10 || charCode == 11 || charCode == 0x2D || charCode == 0x2028 ||
                        charCode == 0x2029) {
                        // Store the state of the line before starting on the new line.
                        SaveWordWrappingState(ref internalLineState, i, m_characterCount);
                        // Store the state of the last Character before the new line.
                        SaveWordWrappingState(ref internalWordWrapState, i, m_characterCount);

                        m_lineNumber += 1;
                        m_firstCharacterOfLine = m_characterCount + 1;

                        var ascender = GetAdjustedAscender(m_internalCharacterInfo[m_characterCount]);

                        // Apply Line Spacing with special handling for VT char(11)
                        if (m_lineHeight == TMP_Math.FLOAT_UNSET) {
                            var lineOffsetDelta = 0 - m_maxLineDescender + ascender +
                                                  (lineGap + m_lineSpacingDelta) * baseScale +
                                                  (m_lineSpacing + (charCode == 10 || charCode == 0x2029
                                                      ? m_paragraphSpacing
                                                      : 0)) * currentEmScale;
                            m_lineOffset += lineOffsetDelta;
                            m_IsDrivenLineSpacing = false;
                        }
                        else {
                            m_lineOffset += m_lineHeight +
                                            (m_lineSpacing + (charCode == 10 || charCode == 0x2029
                                                ? m_paragraphSpacing
                                                : 0)) * currentEmScale;
                            m_IsDrivenLineSpacing = true;
                        }

                        m_maxLineAscender = k_LargeNegativeFloat;
                        m_maxLineDescender = k_LargePositiveFloat;
                        m_startOfLineAscender = ascender;

                        m_xAdvance = 0 + tag_LineIndent + tag_Indent;

                        m_characterCount += 1;
                        continue;
                    }

                    // If End of Text
                    if (charCode == 0x03)
                        i = m_TextProcessingArray.Length;
                }

                #endregion Check for Linefeed or Last Character


                // Save State of Mesh Creation for handling of Word Wrapping

                #region Save Word Wrapping State

                if (isWordWrappingEnabled || m_overflowMode == TextOverflowModes.Truncate ||
                    m_overflowMode == TextOverflowModes.Ellipsis) {
                    if ((isWhiteSpace || charCode == 0x200B || charCode == 0x2D || charCode == 0xAD) &&
                        !m_isNonBreakingSpace && charCode != 0xA0 && charCode != 0x2007 && charCode != 0x2011 &&
                        charCode != 0x202F && charCode != 0x2060) {
                        // We store the state of numerous variables for the most recent Space, LineFeed or Carriage Return to enable them to be restored
                        // for Word Wrapping.
                        SaveWordWrappingState(ref internalWordWrapState, i, m_characterCount);
                        isFirstWordOfLine = false;
                        isLastCharacterCJK = false;

                        // Reset soft line breaking point since we now have a valid hard break point.
                        internalSoftLineBreak.previous_WordBreak = -1;
                    }
                    // Handling for East Asian languages
                    else if (m_isNonBreakingSpace == false &&
                             ((((charCode > 0x1100 && charCode < 0x11ff) || /* Hangul Jamo */
                                (charCode > 0xA960 && charCode < 0xA97F) || /* Hangul Jamo Extended-A */
                                (charCode > 0xAC00 && charCode < 0xD7FF)) && /* Hangul Syllables */
                               TMP_Settings.useModernHangulLineBreakingRules == false) ||
                              (charCode > 0x2E80 && charCode < 0x9FFF) || (charCode > 0xF900 && charCode < 0xFAFF) ||
                              (charCode > 0xFE30 && charCode < 0xFE4F) ||
                              (charCode > 0xFF00 && charCode < 0xFFEF))) /* CJK Halfwidth */ {
                        var isLeadingCharacter = TMP_Settings.linebreakingRules.leadingCharacters.ContainsKey(charCode);
                        var isFollowingCharacter = m_characterCount < totalCharacterCount - 1 &&
                                                   TMP_Settings.linebreakingRules.followingCharacters.ContainsKey(
                                                       m_internalCharacterInfo[m_characterCount + 1].character);

                        if (isFirstWordOfLine || isLeadingCharacter == false) {
                            if (isFollowingCharacter == false) {
                                SaveWordWrappingState(ref internalWordWrapState, i, m_characterCount);
                                isFirstWordOfLine = false;
                            }

                            if (isFirstWordOfLine) {
                                // Special handling for non-breaking space and soft line breaks
                                if (isWhiteSpace)
                                    SaveWordWrappingState(ref internalSoftLineBreak, i, m_characterCount);

                                SaveWordWrappingState(ref internalWordWrapState, i, m_characterCount);
                            }
                        }

                        isLastCharacterCJK = true;
                    }
                    else if (isLastCharacterCJK) {
                        var isLeadingCharacter = TMP_Settings.linebreakingRules.leadingCharacters.ContainsKey(charCode);

                        if (isLeadingCharacter == false)
                            SaveWordWrappingState(ref internalWordWrapState, i, m_characterCount);

                        isLastCharacterCJK = false;
                    }
                    else if (isFirstWordOfLine) {
                        // Special handling for non-breaking space and soft line breaks
                        if (isWhiteSpace || (charCode == 0xAD && isSoftHyphenIgnored == false))
                            SaveWordWrappingState(ref internalSoftLineBreak, i, m_characterCount);

                        SaveWordWrappingState(ref internalWordWrapState, i, m_characterCount);
                        isLastCharacterCJK = false;
                    }
                }

                #endregion Save Word Wrapping State

                m_characterCount += 1;
            }

            // Check Auto Sizing and increase font size to fill text container.

            #region Check Auto-Sizing (Upper Font Size Bounds)

            fontSizeDelta = m_maxFontSize - m_minFontSize;
            if (isTextAutoSizingEnabled && fontSizeDelta > 0.051f && fontSize < m_fontSizeMax &&
                m_AutoSizeIterationCount < m_AutoSizeMaxIterationCount) {
                // Reset character width adjustment delta
                if (m_charWidthAdjDelta < m_charWidthMaxAdj / 100)
                    m_charWidthAdjDelta = 0;

                m_minFontSize = fontSize;

                var sizeDelta = Mathf.Max((m_maxFontSize - fontSize) / 2, 0.05f);
                fontSize += sizeDelta;
                fontSize = Mathf.Min((int)(fontSize * 20 + 0.5f) / 20f, m_fontSizeMax);

                //Debug.Log("[" + m_AutoSizeIterationCount + "] Increasing Point Size from [" + m_minFontSize.ToString("f3") + "] to [" + m_fontSize.ToString("f3") + "] with delta of [" + sizeDelta.ToString("f3") + "].");
                return Vector2.zero;
            }

            #endregion End Auto-sizing Check

            m_IsAutoSizePointSizeSet = true;

            m_isCalculatingPreferredValues = false;

            // Adjust Preferred Width and Height to account for Margins.
            renderedWidth += m_margin.x > 0 ? m_margin.x : 0;
            renderedWidth += m_margin.z > 0 ? m_margin.z : 0;

            renderedHeight += m_margin.y > 0 ? m_margin.y : 0;
            renderedHeight += m_margin.w > 0 ? m_margin.w : 0;

            // Round Preferred Values to nearest 5/100.
            // DISABLED by OSIG TOOLS:amarcolina because it causes issues with very small real-world font sizes
            //renderedWidth = (int)(renderedWidth * 100 + 1f) / 100f;
            //renderedHeight = (int)(renderedHeight * 100 + 1f) / 100f;

            //Debug.Log("Preferred Values: (" + renderedWidth + ", " + renderedHeight + ") with Recursive count of " + m_recursiveCount);

            return new Vector2(renderedWidth, renderedHeight);
        }

        #endregion
    }
}
