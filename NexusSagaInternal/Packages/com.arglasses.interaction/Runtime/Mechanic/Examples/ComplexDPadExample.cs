using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class ComplexDPadExample : MonoBehaviour
    {
        [SerializeField] private Target _target;
        [SerializeField] private float _spawnedScale = 0.05f;
        [SerializeField] private float _spawnedOffset = 0.05f;
        [SerializeField] private float _destroyDelay = 1f;

        // notice that unlike the other examples, we don't need a Mechanic here since DPad events are discrete
        // we can subscribe directly to a Target component
        private void Awake() => this.Ensure(ref _target);
        private void OnEnable() => _target.WhenHovering += HandleHovering;
        private void OnDisable() => _target.WhenHovering -= HandleHovering;

        private void HandleHovering(IHover hover)
        {
            var currentDPad = hover.DPad;

            if (currentDPad.IsNone()) return;

            var spawned = GameObject.CreatePrimitive(PrimitiveType.Cube);
            spawned.transform.SetParent(transform, false);
            spawned.transform.localScale = Vector3.one * _spawnedScale;

            // we can convert the dPad enum into a local Vector2 easily
            spawned.transform.localPosition += currentDPad.ToVector3() * _spawnedOffset;
            Destroy(spawned, _destroyDelay);

            // or we can perform logic based on the dPad direction
            if(currentDPad.IsUp()) Debug.Log("Do something Up!");
            if(currentDPad.IsRight()) Debug.Log("Do something Right!");
            if(currentDPad.IsDown()) Debug.Log("Do something Down!");
            if(currentDPad.IsLeft()) Debug.Log("Do something Left!");

            // You can dig into the Selection.Current object for 'source' data from the rig and manipulation
            var gazePose = hover.Eyes;
            var pinchPose = hover.Pinch;
            var gazeHitPose = hover.TargetHit;
            var head = hover.Head;
            var imu = hover.ImuReferenceDelta;
        }
    }
}
