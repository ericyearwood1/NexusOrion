using ARGlasses.Components;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class ComponentProxyReceiver : MonoBehaviour
    {
        [Header("Receiver")] [SerializeField] private bool _suppressReceiverTargeting = true;
        [SerializeField] private Target _receiver;

        [Header("Sources")] [SerializeField] private ViewController[] _sourceViewControllers;
        [SerializeField] private Target[] _sourceTargets;

        private void Start()
        {
            if (_suppressReceiverTargeting) _receiver.AddSuppressor(this);
            foreach (var target in _sourceTargets) _receiver.RegisterProxySource(target);
            foreach (var vc in _sourceViewControllers) vc.SubscribeToInitialization(() => { _receiver.RegisterProxySource(vc.GetComponentInChildren<Target>()); });
        }
    }
}
