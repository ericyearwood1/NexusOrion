using System;
using OSIG.Tools.Layout;
using ProtoKit.UI;
using UnityEngine;

namespace ARGlasses.Components
{
    /// <summary>
    /// tells controller where to assign icons
    /// </summary>
    public class BackgroundProvider : MonoBehaviour
    {
        public PKUIPanel BackgroundRenderer;
        public OCLayoutSize OcLayoutSize;
        public OCInsetAttachData AttachData;
        public Vector3 CurrentSize => OcLayoutSize.CurrentSize;

        private void Awake()
        {
            if (OcLayoutSize == null)
                OcLayoutSize = GetComponent<OCLayoutSize>();
            if (BackgroundRenderer == null)
                BackgroundRenderer = GetComponent<PKUIPanel>();
            if (AttachData == null)
                AttachData = GetComponent<OCInsetAttachData>();
        }
    }
}
