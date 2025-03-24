using UnityEngine;

namespace Multiplayer.Runtime.View
{
    public class UserEntityView : SyncedEntityView
    {
        private const float Speed = 45f;
        private static readonly int Color = Shader.PropertyToID("_Color");
        [SerializeField] private MeshRenderer _renderer;
        private Vector3 _position;
        private Quaternion _rotation;
        private Vector3 _scale;

        public void UpdateDisplay(Color color)
        {
            if(_renderer == null) return;
            
            var propertyBlock = new MaterialPropertyBlock();
            propertyBlock.SetColor(Color, color);
            _renderer.SetPropertyBlock(propertyBlock);
        }

        public void SetTargets(Vector3 position, Quaternion rotation, Vector3 scale)
        {
            _position = position;
            _rotation = rotation;
            _scale = scale;
            transform.localPosition = _position;
            transform.localRotation = _rotation;
            transform.localScale = _scale;
        }      
    }
}