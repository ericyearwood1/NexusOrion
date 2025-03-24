using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Component = UnityEngine.Component;

namespace ARGlasses.Interaction
{
    public enum SearchOrder
    {
        None,
        FirstInHierarchy,
        LastInHierarchy,
        RenderFirst,
        RenderLast
    }

    public enum Relation
    {
        None,
        Sibling,
        Parent,
        Child,
        Descendant,
        Ancestor,
        Scene,
    }

    [Flags]
    public enum RelationFlags
    {
        None = 1 << Relation.None,
        Sibling = 1 << Relation.Sibling,
        Parent = 1 << Relation.Parent,
        Child = 1 << Relation.Child,
        Descendant = 1 << Relation.Descendant,
        Ancestor = 1 << Relation.Ancestor,
        Scene = 1 << Relation.Scene,

        SiblingParent = Sibling | Parent,
        SiblingChild = Sibling | Child,
        SiblingDescendant = Sibling | Descendant,
        SiblingAncestor = Sibling | Ancestor,
    }

    public static class Hierarchy
    {
        /// <summary>
        /// Ensure that there are no other instances of this component type in my parents or children
        /// </summary>
        public static bool EnsureIsLeaf<T>(this T c, List<T> results = null, bool logError = true) where T : Component
        {
            var others = GetComponentsInChildrenOrParent(c, true, includeSelf: false, results);
            if (others.Count == 0) return true;

            var othersLines = "";
            foreach (var other in others) othersLines += $"Other: {other.GetPath()}\n";
            string message =
                $"<b>{c.GetType().Name}</b>.{nameof(EnsureIsLeaf)} expected to be a <b>Leaf</b> but found found parents or children.<b>{typeof(T).Name}</b>\nExpected Leaf: {c.GetPath()}\n{othersLines}\n\n";
            if (logError) Debug.LogError(message, c);
            else Debug.LogWarning(message, c);
            return false;
        }

        public static List<T> GetComponentsInChildrenOrParent<T>(this Component c, bool includeInactive = true, bool includeSelf = true, List<T> results = null) where T : Component
        {
            results ??= new List<T>();
            c.GetComponentsInChildren(includeInactive: includeInactive, results);
            c.GetComponentsInParent(includeInactive: includeInactive, results);
            if (!includeSelf && c is T providedComponentIsOfType) results.Remove(providedComponentIsOfType);
            return results;
        }

        public static bool IsNone(this Relation f) => f == Relation.None;
        public static bool IsSibling(this Relation f) => f == Relation.Sibling;
        public static bool IsParent(this Relation f) => f == Relation.Parent;
        public static bool IsChildren(this Relation f) => f == Relation.Child;
        public static bool IsDescendant(this Relation f) => f == Relation.Descendant;
        public static bool IsAncestor(this Relation f) => f == Relation.Ancestor;
        public static bool IsScene(this Relation f) => f == Relation.Scene;

        public static RelationFlags ToFlags(this Relation relation) => (RelationFlags)(1 << (int)relation);

        public static bool IsNone(this RelationFlags f) => f == RelationFlags.None;
        public static bool IncludeSibling(this RelationFlags f) => (f & RelationFlags.Sibling) != 0;
        public static bool IncludeParent(this RelationFlags f) => (f & RelationFlags.Parent) != 0;
        public static bool IncludeChildren(this RelationFlags f) => (f & RelationFlags.Child) != 0;
        public static bool IncludeDescendant(this RelationFlags f) => (f & RelationFlags.Descendant) != 0;
        public static bool IncludeAncestor(this RelationFlags f) => (f & RelationFlags.Ancestor) != 0;
        public static bool IncludeScene(this RelationFlags f) => (f & RelationFlags.Scene) != 0;

        public static bool IsRelated(this Component a, Component b, RelationFlags f)
        {
            if (f.IncludeSibling() && a.gameObject == b.gameObject) return true;
            if (f.IncludeParent() && a.transform.parent == b.gameObject.transform) return true;
            if (f.IncludeChildren() && b.transform.parent == a.gameObject.transform) return true;
            if (f.IncludeDescendant() && b.transform.IsChildOf(a.gameObject.transform)) return true;
            if (f.IncludeAncestor() && a.transform.IsChildOf(b.gameObject.transform)) return true;
            if (f.IncludeScene()) return true;
            return false;
        }

        private static readonly Stack<Transform> _ancestryA = new();
        private static readonly Stack<Transform> _ancestryB = new();

        public static int GetDepth(Component a)
        {
            int depth = 0;
            var t = a.transform;
            while (t != null)
            {
                depth++;
                t = t.parent;
            }

            return depth;
        }

        public static T Sort<T>(T a, T b, SearchOrder searchOrder) where T : Component
        {
            if (searchOrder == SearchOrder.FirstInHierarchy) return CompareBreadthFirst(a, b) < 0 ? a : b;
            if (searchOrder == SearchOrder.LastInHierarchy) return CompareBreadthFirst(a, b) < 0 ? b : a;
            if (searchOrder == SearchOrder.RenderFirst) return CompareInspectorOrder(a, b) < 0 ? a : b;
            if (searchOrder == SearchOrder.RenderLast) return CompareInspectorOrder(a, b) < 0 ? b : a;
            Debug.LogWarning("Hierarchy.Find cannot use SearchOrder.None");
            return a;
        }

        public static int CompareBreadthFirst(Component a, Component b)
        {
            if (a == null || b == null) return 0;
            var aDepth = GetDepth(a);
            var bDepth = GetDepth(b);
            var depthCompare = aDepth.CompareTo(bDepth);
            return depthCompare != 0 ? depthCompare : CompareInspectorOrder(a, b);
        }

        private const bool DebugLog = false;

        // -1: a renders behind b.
        // 1: a renders after b.
        // First is rendered behind
        // Last is rendered ontop
        public static int CompareInspectorOrder(Component a, Component b)
        {
            if (a == null || b == null) return 0;

            void PopulateAncestry(Transform target, Stack<Transform> ancestry)
            {
                ancestry.Clear();
                ancestry.Push(target);
                while (target.parent != null)
                {
                    target = target.parent;
                    ancestry.Push(target);
                }
            }

            PopulateAncestry(a.transform, _ancestryA);
            PopulateAncestry(b.transform, _ancestryB);

            // no common root, cannot compare
            if (_ancestryA.Pop() != _ancestryB.Pop()) return 0;

            var minDepth = Mathf.Min(_ancestryA.Count, _ancestryB.Count);
            for (var i = 0; i < minDepth; i++)
            {
                var ancestorA = _ancestryA.Pop();
                var ancestorB = _ancestryB.Pop();

                // traverse down from shared root until the first branch is found
                if (ancestorA != ancestorB)
                {
                    var siblingIndexA = ancestorA.GetSiblingIndex();
                    var siblingIndexB = ancestorB.GetSiblingIndex();
                    var compareResult = siblingIndexA.CompareTo(siblingIndexB);
                    return compareResult;
                }
            }

            // same ancestry line, choose the deeper one is Last
            return _ancestryA.Count.CompareTo(_ancestryB.Count);
        }

        [System.Diagnostics.Conditional("DEVELOPMENT_BUILD")]
        private static void Log(object message) => Debug.Log(message);
    }
}
