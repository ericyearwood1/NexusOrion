using System.Collections;
using System.Collections.Generic;
using System.Linq;
using ARGlasses.Interaction.Motion;
using UnityEngine;
using UnityEngine.Events;

namespace ARGlasses.Interaction
{
    public class CanvasGroupFadeView : MonoBehaviour
    {
        public CanvasGroup _canvasGroup;
        public bool _hiddenOnStart = false;
        public bool SetInteractivity = false;
        
        [Space(10)]
        public MotionParamsEasing _fadeIn = new(Easing.Linear, 0.6f);

        [Space(10)]
        public MotionParamsEasing _fadeOut = new(Easing.Linear, 0.8f);
        
        private Coroutine _fadeRoutine;
        
        [HideInInspector]
        public bool _isShown;

        public UnityEvent OnShown;
        public UnityEvent OnHidden;

        [SerializeField]
        private bool _debugging = false;
        
        void Awake()
        {
            if(_hiddenOnStart)
            {
                SetVisibilityImmediate(false);
            }
        }
        
        public void ShowHide(float showDuration = 1.0f)
        {
            if (_fadeRoutine != null)
            {
                StopCoroutine(_fadeRoutine);
            }

            _fadeRoutine = StartCoroutine(ShowHideRoutine(showDuration));
        }

        IEnumerator ShowHideRoutine(float showDuration)
        {
            Show();
            yield return new WaitForSeconds(showDuration + _fadeIn._duration);
            Hide();
        }
        
        public void Show(float delay = 0f)
        {
            SetVisibility(true, delay);
        }

        public void Hide(float delay = 0f)
        {
            SetVisibility(false, delay);
        }
        
        public void SetVisibility(bool visible, float delay = 0)
        {
            if (!_canvasGroup.gameObject.activeInHierarchy) return;
            if (visible == _isShown) return;
            
            float targetAlpha = visible ? 1f : 0f;
            MotionParamsEasing motionParams = visible ? _fadeIn.Copy() : _fadeOut.Copy();
            motionParams._delay = delay;
            
            float vToTargetAlpha = targetAlpha - _canvasGroup.alpha;

            //Reduce the duration if already mid transition
            motionParams._duration *= Mathf.Abs(vToTargetAlpha);
            
            if(_debugging) Debug.Log($"vToTargetAlpha: {vToTargetAlpha} // duration: {motionParams._duration} / {(visible ? _fadeIn._duration : _fadeOut._duration)}", this);

            _canvasGroup.FadeOneShot(targetAlpha, motionParams).OnComplete(() => { HandleVisibilitySet(visible, true); });
        }

        private void HandleVisibilitySet(bool visible, bool fireEvents = false)
        {
            _canvasGroup.alpha = visible ? 1f : 0f;
            
            if (SetInteractivity)
            {
                _canvasGroup.interactable = visible;
                _canvasGroup.blocksRaycasts = visible;

                SetColliders(visible);
            }

            _isShown = visible;
            
            if (!fireEvents) return;
            
            if (visible)
            {
                OnShown?.Invoke();
            }
            else
            {
                OnHidden?.Invoke();
            }
        }

        public void SetVisibilityImmediate(bool visible, bool fireEvents = false)
        {
            HandleVisibilitySet(visible, fireEvents);
        }

        public void SetColliders(bool shouldEnable)
        {
            if(_debugging) Debug.Log($"CanvasGroupFadeView - Setting Colliders enabled: <b>{shouldEnable}</b>");
                
            //Find any canvas group components and respect their Ignore Parent Groups property
            CanvasGroup[] canvasGroups = transform.GetComponentsInChildren<CanvasGroup>();
            
            List<Collider> ignoredColliders = new List<Collider>();
            List<Transform> ignoredTransforms = new List<Transform>();
            
            foreach (var canvasGroup in canvasGroups)
            {
                if (canvasGroup.ignoreParentGroups)
                {
                    if(_debugging) Debug.Log($"Found canvas group with IgnoreParentGroups: <b>{canvasGroup.gameObject.name}</b>");
                    
                    //Add any child colliders/interactables to an ignored list
                    ignoredColliders.AddRange(canvasGroup.transform.GetComponentsInChildren<Collider>());
                    ignoredTransforms.AddRange(FindChildren(canvasGroup.transform, "_interactable"));
                }
            }
            
            Collider[] colliders = transform.GetComponentsInChildren<Collider>(true);
            foreach (Collider c in colliders)
            {
                if (ignoredColliders.Contains(c))
                {
                    if(_debugging) Debug.Log($"Ignoring collider: <b>{c.gameObject.name}</b>");
                    continue;
                }
                
                if(_debugging) Debug.Log($"Setting collider <b>{c.gameObject.name}</b> enabled: {shouldEnable}");
                c.enabled = shouldEnable;
            }
            
            Transform[] interactables = FindChildren(transform, "_interactable");
            foreach (Transform t in interactables)
            {
                if (ignoredTransforms.Contains(t))
                {
                    if(_debugging) Debug.Log($"Ignoring transform: <b>{t.gameObject.name}</b>");
                    continue;
                }
                
                if(_debugging) Debug.Log($"Setting transform <b>{t.gameObject.name}</b> active: {shouldEnable}");
                t.gameObject.SetActive(shouldEnable);
            }
        }

        Transform[] FindChildren(Transform transform, string name)
        {
            return transform.GetComponentsInChildren<Transform>(true).Where(t => t.name == name).ToArray();
        }
    }
}
