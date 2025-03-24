using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ARGlasses.Interaction
{

    public static class ResolveIn
    {
        // Populate 'field' with a single instance from the current scene.  Ensure only 1 possible candidate exists, otherwise error
        public static T Scene<T>(this Component searcher, ref T field, string nameStrict = null, string nameContains = null, bool optional = false, bool allowSibling = true, bool sidedName = false, bool includeInactive = false, SearchOrder searchOrder = SearchOrder.None) where T : Component
        {
            if (field != null && field is Behaviour) return field;
            var findObjectsInactive = includeInactive ? FindObjectsInactive.Include : FindObjectsInactive.Exclude;
            var components = GameObject.FindObjectsByType<T>(sortMode: FindObjectsSortMode.InstanceID, findObjectsInactive: findObjectsInactive);

            if (components.Length == 0 && includeInactive && !optional && GameObject.FindObjectOfType<T>(includeInactive: true))
            {
                Debug.LogError($"Found zero active instances of {typeof(T).Name} in the scene, but did find an inactive instance.  Maybe you want to use `includeInactive=true`?");
            }

            var found = FilterComponents(searcher, components, ref field, nameStrict, nameContains, optional, allowSibling, sidedName, nameof(Scene), searchOrder);
            return found;
        }

        // Populate 'field' with a single instance from any child components, searching deep.  Ensure only 1 possible candidate exists, otherwise error
        public static T Sibling<T>(this Component searcher, ref T field, string nameStrict = null, string nameContains = null, bool optional = false, bool allowSibling = true)
            where T : Component
        {
            if (field != null && field is Behaviour) return field;

            var components = searcher.GetComponents<T>();
            var found = FilterComponents(searcher, components, ref field, nameStrict, nameContains, optional, allowSibling, sidedName: false, method: nameof(Sibling), SearchOrder.None);
            return found;
        }

        // Populate 'field' with a single instance from any child components, searching deep.  Ensure only 1 possible candidate exists, otherwise error
        public static T Descendant<T>(this Component searcher, ref T field, string nameStrict = null, string nameContains = null, bool optional = false, bool allowSibling = true, bool sidedName = false, bool includeInactive = false, SearchOrder searchOrder = SearchOrder.None)
            where T : Component
        {
            if (field != null && field is Behaviour) return field;

            var components = searcher.GetComponentsInChildren<T>(includeInactive);
            var found = FilterComponents(searcher, components, ref field, nameStrict, nameContains, optional, allowSibling, sidedName, method: nameof(Descendant), searchOrder);
            return found;
        }

        // Populate 'field' with a single instance from any parent components, searching deep.  Ensure only 1 possible candidate exists, otherwise error
        public static T Ancestor<T>(this Component searcher, ref T field, string nameStrict = null, string nameContains = null, bool optional = false, bool allowSibling = true, bool includeInactive = false, SearchOrder searchOrder = SearchOrder.None)
            where T : Component
        {
            if (field != null && field is Behaviour) return field;
            var components = searcher.GetComponentsInParent<T>(includeInactive);
            var found = FilterComponents(searcher, components, ref field, nameStrict, nameContains, optional, allowSibling, sidedName: false, method: nameof(Ancestor), searchOrder);
            return found;
        }
        
        // Search within direct siblings of a parent, but not their children
        public static T ParentSiblingGameObject<T>(this Component searcher, ref T field, string nameStrict = null, string nameContains = null, bool optional = false, bool allowSibling = true, bool includeInactive = false, SearchOrder searchOrder = SearchOrder.None)
            where T : Component
        {
            if (field != null && field is Behaviour) return field;

            List<T> componentsList = new List<T>();
            
            Transform parentOfParent = searcher.transform.parent.parent;
            for (int i = 0; i < parentOfParent.childCount; i++)
            {
                var c = parentOfParent.GetChild(i).GetComponents<T>();
                if(c != null) componentsList.AddRange(c);
            }

            var components = componentsList.ToArray();
            var found = FilterComponents(searcher, components, ref field, nameStrict, nameContains, optional, allowSibling, sidedName: false, method: nameof(Ancestor), searchOrder);
            return found;
        }
        
        // Search within direct sibling GameObjects, but not their children
        public static T SiblingGameObject<T>(this Component searcher, ref T field, string nameStrict = null, string nameContains = null, bool optional = false, bool allowSibling = true, bool includeInactive = false, SearchOrder searchOrder = SearchOrder.None)
            where T : Component
        {
            if (field != null && field is Behaviour) return field;

            List<T> componentsList = new List<T>();
            
            Transform parent = searcher.transform.parent;
            for (int i = 0; i < parent.childCount; i++)
            {
                var c = parent.GetChild(i).GetComponents<T>();
                if(c != null) componentsList.AddRange(c);
            }

            var components = componentsList.ToArray();
            var found = FilterComponents(searcher, components, ref field, nameStrict, nameContains, optional, allowSibling, sidedName: false, method: nameof(Ancestor), searchOrder);
            return found;
        }

        public static T FilterComponents<T>(Component searcher, T[] components, ref T fieldValue, string nameStrict, string nameContains, bool optional, bool allowSibling, bool sidedName, string method, SearchOrder searchOrder = SearchOrder.None)
            where T : Component
        {
            if (fieldValue != null && fieldValue is Behaviour) return fieldValue;

            var filtered = components.Where(c => allowSibling || c.gameObject != searcher.gameObject);
            if (!string.IsNullOrEmpty(nameStrict)) filtered = filtered.Where(d => d.name == nameStrict);
            if (!string.IsNullOrEmpty(nameContains)) filtered = filtered.Where(d => d.name.ToLower().Contains(nameContains.ToLower()));
            if (sidedName) filtered = filtered.Where(d => d.name.ToLower().Contains(searcher.SideFromName().ToString().ToLower()));

            string PathString()
            {
                string path = string.Empty;
                if (searcher) path = searcher.GetPath();
                if (!string.IsNullOrEmpty(nameStrict)) path += $", nameStrict: {nameStrict}";
                if (!string.IsNullOrEmpty(nameContains)) path += $", nameContains: {nameContains}";
                if (sidedName) path += $", sideFromName: {searcher.SideFromName()}";
                return path;
            }

            if (searchOrder == SearchOrder.None)
            {
                if (filtered.Count() > 1)
                {
                    var foundPaths = "\nFound:\n" + string.Join("\n", filtered.Select(c => c.GetPath())) + "\n\n";
                    Debug.LogError($"<b>{searcher.GetType().Name}</b>.{method} expected exactly one <b>{typeof(T).Name}</b>, but instead found <b>{filtered.Count()}</b> [{PathString()}\n{foundPaths}]", searcher);
                }

                fieldValue = filtered.FirstOrDefault();
            }
            else
            {
                foreach (var candidate in filtered)
                {
                    if (fieldValue == null) fieldValue = candidate;
                    fieldValue = Hierarchy.Sort(candidate, fieldValue, searchOrder);
                }
            }

            if (!optional && fieldValue == null) Debug.LogError($"<b>{searcher.GetType().Name}</b>.{method} could not find <b>{typeof(T).Name}</b> [{PathString()}]", searcher.gameObject);

            return fieldValue;
        }
    }
}
