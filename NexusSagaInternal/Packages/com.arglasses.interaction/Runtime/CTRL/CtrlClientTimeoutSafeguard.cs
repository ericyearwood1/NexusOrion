using System.Collections;
using CTRL.ClientBehaviors;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class CtrlClientTimeoutSafeguard : MonoBehaviour
    {
        [SerializeField] private ScanForHost _scanForHost;

        [SerializeField] private float _disableTime = 8f;

        private void Start()
        {
            if(!_scanForHost) _scanForHost = FindObjectOfType<ScanForHost>();
            if(_scanForHost) StartCoroutine(WaitAndDisable());
        }

        private IEnumerator WaitAndDisable()
        {
            yield return new WaitForSeconds(_disableTime);
            _scanForHost.enabled = false;
        }
    }
}
