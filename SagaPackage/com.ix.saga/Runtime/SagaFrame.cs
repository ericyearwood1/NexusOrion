// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using UnityEngine;
using UnityEngine.Events;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace IX.Saga
{
    [Serializable]
    public enum SagaFrameType : int {
        Color = 0,
        Gray =1,
        Depth =2
    }

    [Serializable]
    public struct NamedMaterial {
        public SagaFrameType name;
        public Material material;
    }

    public class SagaFrame : MonoBehaviour
    {
        public int _frameIndex = 0;
        public List<NamedMaterial> _materials = new List<NamedMaterial>();
        private SagaPlugin _plugin = null;
        private MeshRenderer _meshRenderer;

        void Populate(String caller)
        {
            _plugin = gameObject.GetComponentInParent<SagaPlugin>();
            if (_plugin == null)
            {
                Debug.LogError($"IX: SAGA: IX.Saga.SagaFrame[{_frameIndex}].{caller} could not find SagaPlugin");
            }
            _meshRenderer = gameObject.GetComponent<MeshRenderer>();
            if (_meshRenderer == null)
            {
                Debug.LogError($"IX: SAGA: IX.Saga.SagaFrame[{_frameIndex}].{caller} could not find MeshRenderer");
            }
            if (_materials.Count == 0)
            {
                Debug.LogError($"IX: SAGA: IX.Saga.SagaFrame[{_frameIndex}].{caller} could not find material");
            } else
            {
                _meshRenderer.material = new Material(_materials[0].material);
            }
        }

        // Start is called before the first frame update
        void Start()
        {
            Debug.Log($"IX: SAGA: IX.Saga.SagaFrame[{_frameIndex}].Start BEGIN");
            Populate("Start");
            Debug.Log($"IX: SAGA: IX.Saga.SagaFrame[{_frameIndex}].Start END");
        }

        void OnEnable()
        {
            Debug.Log($"IX: SAGA: IX.Saga.SagaFrame[{_frameIndex}].OnEnable BEGIN");
            // Populate("OnEnable");
            Debug.Log($"IX: SAGA: IX.Saga.SagaFrame[{_frameIndex}].OnEnable END");
        }

        private void Awake()
        {
            Debug.Log($"IX: SAGA: IX.Saga.SagaFrame[{_frameIndex}].Awake BEGIN");
            // Populate("Awake");
            Debug.Log($"IX: SAGA: IX.Saga.SagaFrame[{_frameIndex}].Awake END");
        }

        // Update is called once per frame
        void Update()
        {
            if (_plugin != null && _meshRenderer.material != null)
            {
                if (_plugin.PollFrame(_frameIndex, out var texture))
                {
                    if (texture != null)
                    {
                        Debug.Log($"IX: SAGA: IX.Saga.SagaFrame[{_frameIndex}].Update new Texture");
                        _meshRenderer.material.SetTexture("_MainTex", texture);
                    }
                    Debug.Log($"IX: SAGA: IX.Saga.SagaFrame[{_frameIndex}].Update texture updated");
                }
            }
        }
    }
}
