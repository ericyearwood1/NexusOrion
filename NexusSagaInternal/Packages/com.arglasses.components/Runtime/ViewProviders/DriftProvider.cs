using OSIG.Tools.Layout;
using OSIG.Tools.Layout.Internals;
using OSIG.Tools.Units;
using UnityEngine;

namespace ARGlasses.Components
{
    public class DriftProvider : MonoBehaviour, IOCLayoutListener
    {
        private bool _override;
        private Vector3 _overrideLocalPos;
        private Vector3 _initPos;

        public void MoveAbsolute(Vector3 drift)
        {
            if (drift == Vector3.zero) //drag has resolved. let layout engine take over
            {
                _override = false;
                GetComponent<OCLayoutComponentBase>().SetDirty();
            }
            else
            {
                if (!_override) //first drag event

                {
                    _override = true;
                    _initPos = transform.localPosition;
                }

                _overrideLocalPos = _initPos + drift;
                transform.localPosition = _overrideLocalPos;
            }
        }

        public void OnApplyLayout(IOCLayoutComponent layoutComponent)
        {
            if (_override)
            {
                transform.localPosition = _overrideLocalPos;
            }
        }
    }
}
