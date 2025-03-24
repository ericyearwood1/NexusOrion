using FigmaUnity.FigmaComponents;
using OSIG.Tools.Layout.Internals;
using UnityEngine;
using UnityEditor;

namespace ARGlasses.Components
{
    public abstract class StyleEnforcerBase<TController, TViewModel> : MonoBehaviour
        where TController : ViewController
        where TViewModel : ViewModelBase
    {
        protected TController _controller;
        protected TViewModel _model;

        private void OnValidate()
        {
            if (Application.isPlaying) return;
            PopulateDependencies();
        }
        
        protected abstract GameObject GetPrefab();

        protected abstract void PopulateUse();

        protected void PopulateDependencies()
        {
            if (_controller == null) _controller = GetComponent<TController>();
        }
        
        public void PopulatePrefab()
        {
            PopulateDependencies();
            var prefabToSpawn = GetPrefab();
            if (prefabToSpawn != null)
            {

#if UNITY_EDITOR
                // Get the root of the prefab instance and its asset path before unpacking
                GameObject root = PrefabUtility.GetOutermostPrefabInstanceRoot(_controller.gameObject);
                string assetPath = null;
                if(root != null) //if this is null, there is no prefab to worry about
                {
                    assetPath = AssetDatabase.GetAssetPath(PrefabUtility.GetCorrespondingObjectFromSource(root));

                    if (root != _controller.gameObject) // Check if gameObject is nested inside another prefab instance
                    {
                        // Unpack the prefab instance
                        // PrefabUtility.UnpackPrefabInstance(root, PrefabUnpackMode.OutermostRoot,
                        //     InteractionMode.AutomatedAction);
                    }
                }
#endif
                //modify the hierarchy now that we're unpacked
                var componentViewRoot = _controller.GetComponentInChildren<ComponentViewRootProvider>();
                DestroyAllChildren(componentViewRoot.transform);

#if UNITY_EDITOR
                var instantiatePrefab =
                    (GameObject)PrefabUtility.InstantiatePrefab(prefabToSpawn, componentViewRoot.transform);

                instantiatePrefab.name += $" (USE BASE \"{_controller.gameObject.name}\" TO MODIFY)";

                if (root != null && root != _controller
                        .gameObject) // Check if _argButton.gameObject is nested inside another prefab instance
                {
                    // Repack the prefab instance
                    //PrefabUtility.SaveAsPrefabAssetAndConnect(root, assetPath, InteractionMode.AutomatedAction);
                }

#else
                var instantiatedGo = GameObject.Instantiate(prefabToSpawn, componentViewRoot.transform);
#endif
                PopulateDependencies();
                PopulateUse();
                _controller.ForceUpdateViewNoModel();
                OCLayoutSingletonDriver.ForceUpdateNowForAllLayoutInScene();
            }
        }

        protected void DestroyAllChildren(Transform parent)
        {
            if (transform == null)
                return;
            while (parent.childCount > 0)
            {
                var child = parent.GetChild(0);
                DestroyImmediate(child.gameObject);
            }
        }
    }
}
