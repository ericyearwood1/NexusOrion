using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class EditorPersonalSpaceOverride : MonoBehaviour
    {
        [SerializeField]
        private ARGlassesPersonalSpace _personalSpace;

        [SerializeField]
        private float _overrideDistanceInEditor = 0.3f;
        
        void Start()
        {
            if (Application.isEditor && !UnityEngine.XR.XRSettings.isDeviceActive)
            {
                _personalSpace._distanceFromEye = _overrideDistanceInEditor;
            }
        }
    }
}
