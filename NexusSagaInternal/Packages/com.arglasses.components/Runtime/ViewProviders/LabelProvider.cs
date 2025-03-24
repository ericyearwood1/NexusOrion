using OSIG.Tools.Units;
using ProtoKit.UI;
using UnityEditor;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    /// <summary>
    /// tells view controllers where to render text label
    /// </summary>
    public class LabelProvider : MonoBehaviour
    {
        [FormerlySerializedAs("TextRenderer")] [SerializeField]
        private PKUIText _textRenderer;

        public void SetLabel(string text)
        {
            _textRenderer.text = text;
#if UNITY_EDITOR
            EditorUtility.SetDirty(_textRenderer);
#endif
        }

        public void SetSize(float size)
        {
            _textRenderer.fontSize = new OCValue(size, OCUnits.Pixels);
        }
    }
}
