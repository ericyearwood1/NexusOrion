// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using System.Collections;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static class EnsureIn
    {
        public static T Ensure<T>(this Component searcher, ref T field, Relation relation = Relation.Sibling, RelationFlags searchFlags = RelationFlags.None, Type defaultType = null) where T : Component
        {
            if (field != null) return field;
            field = Ensure<T>(searcher, relation, searchFlags, defaultType);
            return field;
        }

        public static T Ensure<T>(this Component searcher, Relation relation = Relation.Sibling, RelationFlags searchFlags = RelationFlags.None, Type defaultType = null) where T : Component
        {
            if (searcher == null)
            {
                Debug.LogError($"Ensure<{typeof(T)}> called with null searcher");
                return null;
            }

            if (searchFlags.IsNone()) searchFlags = relation.ToFlags();

            T result = null;
            if (searcher.TrySearch(searchFlags, out result)) return result;
            if (searcher.TryCreate(relation, defaultType, out result)) return result;
            return result;
        }

        public class SearchResult<T>
        {
            public T Result;
            public bool HasResult => Result != null;
        }

        public static IEnumerator WaitForDependency<T>(Component searcher, RelationFlags search, SearchResult<T> result) where T : Component
        {
            while (!result.HasResult)
            {
                searcher.TrySearch(search, out result.Result);
                if (result.HasResult) yield break;

                // check timeout

                yield return null;
            }
        }

        public static T Search<T>(this Component searcher, RelationFlags search) where T : Component
        {
            return searcher.TrySearch<T>(search, out var result) ? result : null;
        }

        public static bool TrySearch<T>(this Component searcher, RelationFlags search, out T result) where T : Component
        {
            result = null;
            if (search.IncludeSibling()) if (searcher.TryGetComponent(out result)) return result;
            if (search.IncludeParent()) if (searcher.transform.parent.TryGetComponent(out result)) return result;
            if (search.IncludeChildren())
            {
                var childCount = searcher.transform.childCount;
                for (int i = 0; i < childCount; ++i)
                {
                    var child = searcher.transform.GetChild(i);
                    if (child.TryGetComponent(out result)) return result;
                }
            }

            if (search.IncludeAncestor())
            {
                VerifyNoSibling<T>(searcher, Relation.Ancestor);
                if (searcher.Ancestor(ref result, optional: true, allowSibling: false, includeInactive: true)) return result;
            }

            if (search.IncludeDescendant())
            {
                VerifyNoSibling<T>(searcher, Relation.Descendant);
                if (searcher.Descendant(ref result, optional: true, allowSibling: false, includeInactive:true)) return result;
            }

            return false;
        }

        private static void VerifyNoSibling<T>(Component searcher, Relation relation) where T : Component
        {
            if (!searcher.TryGetComponent(out T sibling)) return;
            Debug.LogError(Format(searcher.GetType().Name, "Ensure", relation.ToString(), typeof(T).ToString(), "sibling", sibling.GetPath()));
        }

        private static string Format(string searcherName, string methodName, string relation, string expectedType, string foundType, string foundPath)
        {
            return $"<b>{searcherName}</b>.{methodName} expected {relation} <b>{expectedType}</b> but found {foundType} instead {foundPath}";
        }

        public static bool TryCreate<T>(this Component searcher, Relation relation, Type defaultType, out T result) where T : Component
        {
            if (relation.IsChildren() || relation.IsDescendant())
            {
                result = searcher.CreateChild<T>(defaultType);
                return result;
            }

            Transform host = null;
            if (relation.IsSibling()) host = searcher.transform;
            if (relation.IsParent()) host = searcher.transform.parent;
            if (relation.IsAncestor()) host = searcher.transform.root;
            if (relation.IsScene()) host = null;
            if (relation.IsNone()) host = null;

            if (host == null)
            {
                result = searcher.CreateChild<T>(defaultType);
                result.transform.parent = null;
                return result;
            }

            if (defaultType != null) result = host.gameObject.AddComponent(defaultType) as T;
            else result = host.gameObject.AddComponent<T>();

            return result;
        }
    }
}
