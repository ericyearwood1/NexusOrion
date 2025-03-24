using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using UnityEngine;
using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine.UIElements;
using Vector2 = UnityEngine.Vector2;
using Vector3 = UnityEngine.Vector3;

namespace ARGlasses.Interaction.Motion
{
    /// <summary>
    /// Custom editor that draws a graph to represent the
    /// normalized curve based on the frequency, damping, response
    /// supplied by MotionParams.
    /// </summary>
    [CustomPropertyDrawer(typeof(MotionParams))]
    public class MotionParamsEditor : PropertyDrawer
    {
        // Object
        SerializedProperty _motionParamsProperty;
        
        private const string FREQ_PROP = "_frequency";
        private const string DAMP_PROP = "_damping";
        private const string RESP_PROP = "_response";

        private float _fCache, _dCache, _rCache;
        private SerializedProperty _fProp, _dProp, _rProp;
        private float _f, _d, _r;
        private Vector2[] _graphDataUnscaledCache;

        // Rect attributes
        float _graphHeight = 3 * EditorGUIUtility.singleLineHeight;
        float _graphPadding = 4f;
        float _totalGraphHeight => _graphHeight + 2 * _graphPadding;
        Rect _graphRectPrev;
        Rect _graphRectCurr;
        
        // Property layout attributes
        private int _prependLinesExpanded = 1;
        private int _appendLinesExpanded = 3;
        private float _propertyElementsHeightExpanded => (_prependLinesExpanded + _appendLinesExpanded) * EditorGUIUtility.singleLineHeight;
        private float _graphToFloatRatioCollapsed = .5f;

        // Visualization attributes
        private float _graphLineThickness => _motionParamsProperty.isExpanded ? 3f : 2f;
        private float _guideLineThickness => _motionParamsProperty.isExpanded ? 2f : 1f;
        private const float MAX_X_DEFAULT = 1f;
        private const float MIN_Y_DEFAULT = -.5f;
        private const float MAX_Y_DEFAULT = 2f;
        private const float Y_CLAMP = 3f;
        private Vector2 _min = new Vector2(0, MIN_Y_DEFAULT);
        private Vector2 _max = new Vector2(MAX_X_DEFAULT, MAX_Y_DEFAULT);
        private float _minYFromData = 0f;
        private float _maxYFromData = 1f;
        private Vector3 _range => _max - _min;
        private readonly float _guideLineLabelSize = 12f;
        private readonly Color _lineZeroColor = new Color(0.39f, 0.47f, 0.54f);
        private readonly Color _lineOneColor = new Color(0.52f, 0.58f, 0.64f);
        private readonly Color _graphLineColor = new Color(0.33f, 1f, 0.62f);

        public override void OnGUI(Rect rect, SerializedProperty property, GUIContent label)
        {
            EditorGUI.BeginProperty(rect, label, property);
            GetPropertyValues(property);
            DrawFoldoutBox(rect, label);
            
            DrawGraphRect(rect);
            DrawGraphData();
            DrawProperties(rect);
            
            EditorGUI.EndProperty();
        }

        private void DrawFoldoutBox(Rect rect, GUIContent label)
        {
            Rect foldOutBox = new Rect(rect.position, new Vector2(rect.size.x, EditorGUIUtility.singleLineHeight));
            _motionParamsProperty.isExpanded = EditorGUI.Foldout(foldOutBox, _motionParamsProperty.isExpanded, label);
        }

        private void DrawProperties(Rect rect)
        {
            if (_motionParamsProperty.isExpanded)
            {
                EditorGUI.indentLevel = 0;
                EditorGUI.PropertyField(GetExpandedPropertyLineRect(rect, 1), _fProp);
                EditorGUI.PropertyField(GetExpandedPropertyLineRect(rect, 2), _dProp);
                EditorGUI.PropertyField(GetExpandedPropertyLineRect(rect, 3), _rProp);    
            }
            else
            {
                float labelWidthCache = EditorGUIUtility.labelWidth;
                float fieldWidthCache = EditorGUIUtility.fieldWidth;

                Rect fRect = GetCollapsedPropertyLineRects(rect, 0);
                Rect dRect = GetCollapsedPropertyLineRects(rect, 1);
                Rect rRect = GetCollapsedPropertyLineRects(rect, 2);
                
                EditorGUIUtility.labelWidth = 12f;
                EditorGUIUtility.fieldWidth = 36f;
                
                EditorGUI.PropertyField(fRect, _fProp, new GUIContent("F"));
                EditorGUI.PropertyField(dRect, _dProp, new GUIContent("D"));
                EditorGUI.PropertyField(rRect, _rProp, new GUIContent("R"));

                EditorGUIUtility.labelWidth = labelWidthCache;
                EditorGUIUtility.fieldWidth = fieldWidthCache;
            }
        }
        
        private void GetPropertyValues(SerializedProperty property)
        {
            _motionParamsProperty = property;

            _fCache = _f;
            _dCache = _d;
            _rCache = _r;

            _fProp = property.FindPropertyRelative(FREQ_PROP);
            _f = _fProp.floatValue;

            _dProp = property.FindPropertyRelative(DAMP_PROP);
            _d = _dProp.floatValue;

            _rProp = property.FindPropertyRelative(RESP_PROP);
            _r = _rProp.floatValue;
        }

        private bool ShouldResimulate()
        {
            return !(Mathf.Approximately(_f,_fCache) &&
                     Mathf.Approximately(_d,_dCache) &&
                     Mathf.Approximately(_r,_rCache) &&
                     _graphRectCurr == _graphRectPrev
                     );
        }

        private void DrawGraphRect(Rect defaultRect)
        {
            _graphRectPrev = _graphRectCurr;
            
            float yOffset;
            Vector2 position;
            Vector2 size;

            if (_motionParamsProperty.isExpanded)
            {
                // Position underneath the foldout label
                yOffset = (_prependLinesExpanded * EditorGUIUtility.singleLineHeight) + _graphPadding;
                position = defaultRect.position + (Vector2.up * yOffset);
                size = new Vector2(defaultRect.width, _graphHeight);
            }
            else
            {
                float xOffset = (defaultRect.width - EditorGUIUtility.labelWidth) * _graphToFloatRatioCollapsed;
                position = defaultRect.position + (Vector2.right * EditorGUIUtility.labelWidth) + (Vector2.right * xOffset);
                size = new Vector2(xOffset, EditorGUIUtility.singleLineHeight);
            }
            
            _graphRectCurr = new Rect(position, size);
            EditorGUI.DrawRect(_graphRectCurr, Color.clear);
        }

        private void DrawGraphData()
        {
            int pixelWidth = Mathf.FloorToInt(_graphRectCurr.width);
            
            if (pixelWidth <= 0) return;

            Vector2[] graphDataUnscaled = new Vector2[pixelWidth];
            Vector3[] graphDataScaled = new Vector3[pixelWidth];

            if (!ShouldResimulate())
            {
                graphDataUnscaled = _graphDataUnscaledCache;
            }
            else
            {
                // Reset Y bounds
                _minYFromData = MIN_Y_DEFAULT;
                _maxYFromData = MAX_Y_DEFAULT;

                float deltaTime = 1f / pixelWidth; // i.e timestep per pixel
                MotionParams secondOrder = new MotionParams(_f, _d, _r);
                Motion<Vector3> motion = new Motion<Vector3>(Vector3.zero, Vector3.one, secondOrder);
                motion._id = $"EDITOR_{motion._id}";

                for (int i = 0; i < pixelWidth; i++)
                {
                    if (i == 0)
                    {
                        graphDataUnscaled[i] = Vector2.zero;
                        continue;
                    }

                    secondOrder.UpdateConstants();
                    motion.Step(deltaTime);
                    float yVal = float.IsNaN(motion.Current.x) ? 0 : motion.Current.x;

                    _minYFromData = Mathf.Min(yVal, _minYFromData);
                    _maxYFromData = Mathf.Max(yVal, _maxYFromData);

                    graphDataUnscaled[i] = new Vector2(deltaTime * i, yVal);
                }
            }

            // Reset graph scaling
            _min.y = _minYFromData;
            _max.y = _maxYFromData;

            // Scale and write graph
            for (int i = 0; i < pixelWidth; i++)
            {
                Vector3 inGraphCoords = new Vector2(UnitToGraphX(graphDataUnscaled[i].x), UnitToGraphY(graphDataUnscaled[i].y));
                graphDataScaled[i] = inGraphCoords;
            }

            // Draw reference lines

            // y=0
            Handles.color = _lineZeroColor;
            Handles.DrawAAPolyLine(_guideLineThickness, UnitToGraph(Vector2.zero), UnitToGraph(Vector2.right));
            Rect yZeroLabel = new Rect(UnitToGraph(Vector2.zero), Vector2.one * _guideLineLabelSize);
            var yZeroStyle = new GUIStyle();
            yZeroStyle.normal.textColor = _lineZeroColor;
            if(_motionParamsProperty.isExpanded) GUI.Label(yZeroLabel, "0", yZeroStyle);

            // y=1
            Handles.color = _lineOneColor;
            Handles.DrawAAPolyLine(_guideLineThickness, UnitToGraph(Vector2.up), UnitToGraph(Vector2.one));
            Rect yOneLabel = new Rect(UnitToGraph(Vector2.up), Vector2.one * _guideLineLabelSize);
            var yOneStyle = new GUIStyle();
            yOneStyle.normal.textColor = _lineOneColor;
            if(_motionParamsProperty.isExpanded) GUI.Label(yOneLabel, "1", yOneStyle);

            // Draw graph
            Handles.color = _graphLineColor;
            Handles.DrawAAPolyLine(_graphLineThickness, graphDataScaled);

            _graphDataUnscaledCache = graphDataUnscaled;
        }

        private Rect GetExpandedPropertyLineRect(Rect defaultRect, int lineIncrement)
        {
            float yOffset = _totalGraphHeight + _prependLinesExpanded + (lineIncrement * EditorGUIUtility.singleLineHeight);
            Vector2 position = new Vector2(defaultRect.position.x, defaultRect.position.y + yOffset);
            Vector2 size = new Vector2(defaultRect.size.x, EditorGUIUtility.singleLineHeight);
            Rect adjRect = new Rect(position, size);

            return adjRect;
        }

        private (Rect, Rect) GetCollapsedPropertyLineRectsWithLabels(Rect defaultRect, int propertyIndex)
        {
            Rect controlsRect = new Rect(defaultRect.position + Vector2.right * EditorGUIUtility.labelWidth, new Vector2(defaultRect.size.x - EditorGUIUtility.labelWidth, EditorGUIUtility.singleLineHeight));
            Rect floatsRect = new Rect(controlsRect.position, new Vector2(controlsRect.size.x * (1-_graphToFloatRatioCollapsed), EditorGUIUtility.singleLineHeight));
            
            float widthOfOneCombo = floatsRect.size.x / _appendLinesExpanded;
            Rect thisComboRect = new Rect(floatsRect.position + (Vector2.right * widthOfOneCombo * propertyIndex), new Vector2(widthOfOneCombo, EditorGUIUtility.singleLineHeight));

            float paddingLeft = 2f;
            float innerWidthOfOneCombo = widthOfOneCombo - paddingLeft;
            float labelToFloatRatio = .3f;
            float widthOfOneLabel = innerWidthOfOneCombo * labelToFloatRatio;
            float widthOfOneFloat = innerWidthOfOneCombo * (1 - labelToFloatRatio);

            Rect thisLabelRect = new Rect(thisComboRect.position + (Vector2.right * paddingLeft), new Vector2(innerWidthOfOneCombo, EditorGUIUtility.singleLineHeight));
            Rect thisFloatRect = new Rect(thisComboRect.position + (Vector2.right * widthOfOneLabel), new Vector2(widthOfOneFloat, EditorGUIUtility.singleLineHeight));
      
            return (thisLabelRect, thisFloatRect);
        }
        
        private Rect GetCollapsedPropertyLineRects(Rect defaultRect, int propertyIndex)
        {
            float paddingLeft = 4;
            
            Rect controlsRect = new Rect(defaultRect.position + Vector2.right * EditorGUIUtility.labelWidth, new Vector2(defaultRect.size.x - EditorGUIUtility.labelWidth, EditorGUIUtility.singleLineHeight));
            Rect floatsRect = new Rect(controlsRect.position, new Vector2(controlsRect.size.x * (1-_graphToFloatRatioCollapsed), EditorGUIUtility.singleLineHeight));
            
            float widthOfOneCombo = (floatsRect.size.x / _appendLinesExpanded) - (paddingLeft * (_appendLinesExpanded-1));
            Rect thisComboRect = new Rect(floatsRect.position + (Vector2.right * widthOfOneCombo * propertyIndex) + (Vector2.right * paddingLeft * propertyIndex), new Vector2(widthOfOneCombo, EditorGUIUtility.singleLineHeight));
            return thisComboRect;
        }
        
        public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
        {
            if (property.isExpanded)
            {
                return _propertyElementsHeightExpanded + _totalGraphHeight;
            }
            else
            {
                return EditorGUIUtility.singleLineHeight;
            }
        }

        #region Helper Functions

        Vector2 UnitToGraph(Vector2 unit)
        {
            unit.x = Mathf.Lerp(_graphRectCurr.x, _graphRectCurr.xMax, (unit.x - _min.x) / _range.x);
            unit.y = Mathf.Lerp(_graphRectCurr.yMax, _graphRectCurr.y, (unit.y - _min.y) / _range.y);

            return unit;
        }

        float UnitToGraphX(float x)
        {
            return Mathf.Lerp(_graphRectCurr.x, _graphRectCurr.xMax, (x - _min.x) / _range.x);
        }

        float UnitToGraphY(float y)
        {
            return Mathf.Lerp(_graphRectCurr.yMax, _graphRectCurr.y, (y - _min.y) / _range.y);
        }

        #endregion

    }
}
