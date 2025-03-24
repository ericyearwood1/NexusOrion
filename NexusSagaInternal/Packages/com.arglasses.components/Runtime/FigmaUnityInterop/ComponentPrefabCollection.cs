using UnityEngine;

namespace FigmaUnity.FigmaComponents
{

    public abstract class ComponentPrefabCollection : ScriptableObject
    {
        public abstract void AddComponentStyleFromString(string styleString);
        public abstract bool ContainsKey(string styleString);
        public abstract void AssignPrefabToStyle(string styleString, GameObject prefab);
    }

    public abstract class ComponentPrefabCollection<T> : ComponentPrefabCollection where T : IComponentStyle
    {
        public abstract bool TryGetPrefabForStyle(T style, out GameObject prefab);
    }
}
