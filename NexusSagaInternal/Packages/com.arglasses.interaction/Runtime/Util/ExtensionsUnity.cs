// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using TMPro;
using UnityEngine;
using Object = UnityEngine.Object;
using Plane = UnityEngine.Plane;
using Quaternion = UnityEngine.Quaternion;
using Vector2 = UnityEngine.Vector2;
using Vector3 = UnityEngine.Vector3;
using Vector4 = UnityEngine.Vector4;

namespace ARGlasses.Interaction
{
    public enum Corner
    {
        BottomLeft,
        TopLeft,
        TopRight,
        BottomRight
    }

    public static class ExtensionsUnity
    {
        public static float Average(float[] arr)
        {
            float result = 0;
            for (int i = 0; i < arr.Length; i++) result += arr[i];
            if (arr.Length > 0) result /= arr.Length;
            return result;
        }

        public static Vector3 Average(this Vector3[] arr)
        {
            Vector3 result = Vector3.zero;
            for (int i = 0; i < arr.Length; i++) result += arr[i];
            if (arr.Length > 0) result /= arr.Length;
            return result;
        }

        public static Vector3 Average(this List<Vector3> arr)
        {
            Vector3 result = Vector3.zero;
            for (int i = 0; i < arr.Count; i++) result += arr[i];
            if (arr.Count > 0) result /= arr.Count;
            return result;
        }

        public static Vector3 AveragePosition(this Component[] arr)
        {
            Vector3 result = Vector3.zero;
            for (int i = 0; i < arr.Length; i++) result += arr[i].transform.position;
            if (arr.Length > 0) result /= arr.Length;
            return result;
        }

        public static float Lerp(this Vector2 vec, float t) => Mathf.Lerp(vec.x, vec.y, t);

        public static float Map(this float x, float inMin, float inMax, float outMin, float outMax, bool clamp = false)
        {
            if (clamp) x = Mathf.Max(inMin, Mathf.Min(x, inMax));
            return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
        }

        public static float Map(this float val, Vector2 inRange, Vector2 outRange, bool clamp = false)
        {
            if (clamp) val = Mathf.Max(inRange.x, Mathf.Min(val, inRange.y));
            return (val - inRange.x) * (outRange.y - outRange.x) / (inRange.y - inRange.x) + outRange.x;
        }

        public static Bounds GetBoundsLocal(this RectTransform rt) => new(rt.rect.center, rt.rect.size);

        public static Vector2 GetRectSize(this Transform t)
        {
            if (t.TryGetComponent<RectTransform>(out var rectTransform)) return rectTransform.rect.size;
            Debug.LogError("No RectTransform to get Size from");
            return Vector2.one;
        }

        public static void SetRectSize(this Transform t, Vector2 size) => t.SetRectSize(size.x, size.y);
        public static void SetRectSize(this Transform t, float x, float y)
        {
            if (!t.TryGetComponent<RectTransform>(out var rectTransform))
            {
                Debug.LogError("No RectTransform to set Size on");
                return;
            }

            rectTransform.SetSizeWithCurrentAnchors(RectTransform.Axis.Horizontal, x);
            rectTransform.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, y);
        }

        public static Vector2 ConvertPointInOuterRectSpaceToInnerRectSpace(this Vector2 pointInOuterRectSpace, Vector2 innerRectPosition, Vector2 innerRectSize)
        {
            // Calculate the relative position of the point to the centroid of inner rectangle
            Vector2 relativePosition = pointInOuterRectSpace - innerRectPosition;
            // Adjust the relative position by the size of inner rectangle
            Vector2 adjustedPosition = new Vector2(relativePosition.x / innerRectSize.x, relativePosition.y / innerRectSize.y);
            // Return the new coordinates
            return adjustedPosition;
        }
        
        public static Vector2 ConvertPointInInnerRectSpaceToOuterRectSpace(this Vector2 pointInInnerRectSpace, Vector2 innerRectPosition, Vector2 innerRectSize)
        {
            // Adjust the point's position by the size of interior rectangle
            Vector2 adjustedPosition = new Vector2(pointInInnerRectSpace.x * innerRectSize.x, pointInInnerRectSpace.y * innerRectSize.y);
            // Calculate the position of the point relative to the centroid of inner rectangle
            Vector2 positionInOuter = adjustedPosition + innerRectPosition;
            // Return the new coordinates
            return positionInOuter;
        }
        
        private static readonly Vector3[] CornersCache = new Vector3[4];
        public static Vector3 GetWorldCorner(this RectTransform rectTransform, Corner corner)
        {
            rectTransform.GetWorldCorners(CornersCache);
            return CornersCache[(int)corner];
        }

        public static (Vector3, Vector3) GetWorldCorner(this RectTransform rectTransform, Corner corner1, Corner corner2)
        {
            rectTransform.GetWorldCorners(CornersCache);
            return (CornersCache[(int)corner1], CornersCache[(int)corner2]);
        }

        public static Bounds LocalToWorld(this Transform transform, Bounds localBounds) => transform.TransformBounds(localBounds);

        public static Bounds TransformBounds(this Transform transform, in Bounds localBounds)
        {
            var p1 = transform.TransformPoint(localBounds.center);
            var p2 = transform.TransformPoint(localBounds.max);
            var radius = (p2 - p1).magnitude;
            return new Bounds(p1, Vector3.one * radius);
        }

        public static Bounds InverseTransformBounds(this Transform transform, in Bounds worldBounds)
        {
            var p1 = transform.InverseTransformPoint(worldBounds.center);
            var p2 = transform.InverseTransformPoint(worldBounds.max);
            var radius = (p2 - p1).magnitude;
            return new Bounds(p1, Vector3.one * radius);
        }

        public static Bounds WorldToLocal(this Transform transform, in Bounds worldBounds)
        {
            var center = transform.InverseTransformPoint(worldBounds.center);
            var inverseMax = transform.InverseTransformPoint(worldBounds.max);
            return new Bounds(center, Vector3.one * (inverseMax - center).magnitude);
        }

        public static Transform ToTransform(this Object o)
        {
            if (o is Component c) return c.transform;
            if (o is GameObject go) return go.transform;
            Debug.LogError($"Cannot find Transform for {o}");
            return null;
        }

        public static GameObject ToGameObject(this Object o)
        {
            if (o is Component c) return c.gameObject;
            if (o is GameObject go) return go;
            Debug.LogError($"Cannot find GameObject for {o}");
            return null;
        }

        public static T CreateChild<T>(this Component creator, ref T field, Type defaultType = null, string name = null) where T : Component
        {
            if (field != null) return field;
            field = CreateChild<T>(creator, defaultType, name);
            return field;
        }

        public static T CreateChild<T>(this GameObject go, Type defaultType = null, string name = null) where T : Component => go.transform.CreateChild<T>(defaultType, name);

        public static T CreateChild<T>(this Component creator, Type defaultType = null, string name = null) where T : Component
        {
            var creatorType = creator.GetType();
            var createType = defaultType ?? typeof(T);
            if (string.IsNullOrEmpty(name)) name = $"-{createType.Name}<={creator.name}:{creatorType.Name}";
            return creator.CreateChild(name).Ensure<T>(defaultType: defaultType);
        }

        public static Transform CreateChild(this GameObject go, string name = null) => go.transform.CreateChild(name);

        public static Transform CreateChild(this Component creator, string name = null)
        {
            var newTransform = new GameObject(string.IsNullOrEmpty(name) ? $"-{creator.name}" : name).transform;
            newTransform.SetParent(creator.transform, false);
            return newTransform;
        }

        public static void SetActive(this MonoBehaviour c, bool v)
        {
            c.enabled = v;
            c.gameObject.SetActive(v);
        }

        public static void ResetToParent(this Transform t)
        {
            t.localPosition = Vector3.zero;
            t.localRotation = Quaternion.identity;
            t.localScale = Vector3.one;
        }

        public static IEnumerable<T> GetChildrenWithName<T>(this Transform transform, string exactName)
            where T : Component
        {
            return transform.GetComponentsInChildren<T>().Where(c => c.name == exactName);
        }

        public static IEnumerable<T> GetChildrenWithNameContains<T>(this Transform transform, string contains)
            where T : Component
        {
            return transform.GetComponentsInChildren<T>().Where(c => c.name.Contains(contains));
        }

        public static void DoChildren<T>(this GameObject gameObject, Action<T> action, bool includeInactive = false)
        {
            foreach (var child in gameObject.GetComponentsInChildren<T>(includeInactive: includeInactive)) action(child);
        }

        public static string Limit(this string text, int maxLines = 1, int maxCharactersPerLine = 120)
        {
            StringBuilder result = new StringBuilder();
            int lineCount = 0;
            int charCount = 0;
            for (var i = 0; i < text.Length; i++)
            {
                var c = text[i];
                if (c is '\r' or '\n')
                {
                    if (c == '\r' && i + 1 < text.Length && text[i + 1] == '\n') // Handle "\r\n"
                        i++;
                    lineCount++;
                    charCount = 0;
                    if (lineCount >= maxLines)
                        break;
                    result.Append(Environment.NewLine);
                }
                else if (charCount < maxCharactersPerLine)
                {
                    result.Append(c);
                    charCount++;
                }
            }
            return result.ToString();
        }

        public static string GetPath(this Component c)
        {
            if (c == null) return string.Empty;
            return $"{c.transform.GetPath()}[{c.transform.GetSiblingIndex()}] ({c.GetType().Name})";
        }

        private static readonly StringBuilder Sb = new();
        public static string GetPath(this Transform t)
        {
            if (t == null) return string.Empty;
            Sb.Length = 0;

            var current = t;
            while (current != null)
            {
                if (Sb.Length > 0) Sb.Insert(0, "/");
                Sb.Insert(0, current.name);
                current = current.parent;
            }

            return Sb.ToString();
        }

        public static List<T> GetComponentsInDirectChildren<T>(this Component searcher, List<T> list = null) where T: MonoBehaviour
        {
            if (list == null) list = new List<T>();
            list.Clear();

            var searcherTransform = searcher.transform;
            foreach (Transform child in searcherTransform)
            {
                if (child.TryGetComponent<T>(out var found))
                {
                    list.Add(found);
                }
            }

            return list;
        }

        public static T Random<T>(this List<T> list, List<T> exclude = null)
        {
            var shuffled = new List<T>(list).Shuffle();
            if(exclude != null) shuffled.RemoveAll(exclude.Contains);
            return shuffled.FirstOrDefault();
        }

        public static List<T> Shuffle<T>(this List<T> list, int shuffleCount = -1)
        {
            if (shuffleCount == -1) shuffleCount = list.Count;
            while (shuffleCount > 1)
            {
                var k = UnityEngine.Random.Range(0, shuffleCount);
                shuffleCount--;
                (list[k], list[shuffleCount]) = (list[shuffleCount], list[k]);
            }

            return list;
        }

        public static Color WithAlpha(this Color color, float a) => new Color(color.r, color.g, color.b, a);

        public static Color Clamp01(this Color c)
        {
            c.r = Mathf.Clamp01(c.r);
            c.g = Mathf.Clamp01(c.g);
            c.b = Mathf.Clamp01(c.b);
            c.a = Mathf.Clamp01(c.a);
            return c;
        }

        public static Vector2 YX(this Vector2 v) => new Vector2(v.y, v.x);

        public static Ray ToRay(this Transform t) => new Ray(t.position, t.forward);

        public static Vector2 Rotate(this Vector2 vec, float degrees)
        {
            var radians = degrees * Mathf.Deg2Rad;
            var sin = Mathf.Sin(radians);
            var cos = Mathf.Cos(radians);
            return new Vector2(vec.x * cos - vec.y * sin, vec.x * sin + vec.y * cos);
        }

        public static Quaternion WithX(this Quaternion q, float x)
        {
            var eulerAngles = q.eulerAngles;
            q = Quaternion.Euler(x, eulerAngles.y, eulerAngles.z);
            return q;
        }

        public static Quaternion WithY(this Quaternion q, float y)
        {
            var eulerAngles = q.eulerAngles;
            q = Quaternion.Euler(eulerAngles.x, y, eulerAngles.z);
            return q;
        }

        public static Quaternion WithZ(this Quaternion q, float z)
        {
            var eulerAngles = q.eulerAngles;
            q = Quaternion.Euler(eulerAngles.x, eulerAngles.y, z);
            return q;
        }

        public static float SignedAngleAlongAxis(this Vector3 fromDir, Vector3 toDir, Vector3 axis,
            bool clockwise = false)
        {
            Vector3 right;
            if (clockwise)
            {
                right = Vector3.Cross(fromDir, axis);
                fromDir = Vector3.Cross(axis, right);
            }
            else
            {
                right = Vector3.Cross(axis, fromDir);
                fromDir = Vector3.Cross(right, axis);
            }

            return Mathf.Atan2(Vector3.Dot(toDir, right), Vector3.Dot(toDir, fromDir)) * Mathf.Rad2Deg;
        }

        public static float Pitch(this Quaternion q)
        {
            var forward = q * Vector3.forward;
            return Mathf.Asin(forward.y) * Mathf.Rad2Deg;
        }

        public static float Yaw(this Quaternion q)
        {
            var projectedXZ = (q * Vector3.forward).WithY(0);
            return Mathf.Atan2(projectedXZ.x, projectedXZ.z) * Mathf.Rad2Deg;
        }

        public static float Roll(this Quaternion q)
        {
            var projectedYZ = (q * Vector3.right).WithX(0);
            return Mathf.Atan2(projectedYZ.z, projectedYZ.y) * Mathf.Rad2Deg;
        }

        public static void ScaleAround(this Transform target, Vector3 pivot, float scaleFactor,
            bool worldSpacePivot = true)
        {
            if (worldSpacePivot && target.parent != null) pivot = target.parent.InverseTransformPoint(pivot);
            target.localScale *= scaleFactor;
            target.localPosition = pivot + ((target.localPosition - pivot) * scaleFactor);
        }

        public static Ray ToRay(this Pose p)
        {
            return new Ray(p.position, p.forward);
        }

        public static Ray DebugDrawRay(this Ray r, Color color)
        {
            Debug.DrawRay(r.origin, r.direction, color);
            return r;
        }

        public static Ray DebugDrawRay(this Ray r)
        {
            Debug.DrawRay(r.origin, r.direction);
            return r;
        }

        public static Pose DebugDrawRay(this Pose p, Color color)
        {
            Debug.DrawRay(p.position, p.forward, color);
            return p;
        }

        public static Pose DebugDrawRay(this Pose p)
        {
            Debug.DrawRay(p.position, p.forward);
            return p;
        }

        public static void Set(this Transform transform, in Transform other) => transform.Set(other.ToPose());

        public static void Set(this Transform transform, in Pose pose, Space space = Space.World)
        {
            if (space == Space.World)
            {
                transform.SetPositionAndRotation(pose.position, pose.rotation);
            }
            else
            {
                transform.localRotation = pose.rotation;
                transform.localPosition = pose.position;
            }
        }

        public static Pose ToPose(this Transform transform, Space space = Space.World)
        {
            return space == Space.World ? new Pose(transform.position, transform.rotation) : new Pose(transform.localPosition, transform.localRotation);
        }

        public static Pose ToPose(this Ray r)
        {
            return new Pose(r.origin, Quaternion.LookRotation(r.direction));
        }

        public static Plane ToPlane(this Ray r)
        {
            return new Plane(r.direction, r.origin);
        }

        public static Plane ToPlane(this Transform t)
        {
            return new Plane(t.forward, t.position);
        }

        public static Plane ToPlaneForward(this Transform t)
        {
            return new Plane(t.forward, t.position);
        }

        public static Plane ToPlaneRight(this Transform t)
        {
            return new Plane(t.right, t.position);
        }

        public static Plane ToPlane(this Pose p)
        {
            return new Plane(p.forward, p.position);
        }

        public static Pose Inverse(this Pose p)
        {
            return new Pose(-p.position, Quaternion.Inverse(p.rotation));
        }

        public static Pose Rotate(this Pose pose, Quaternion rotation)
        {
            pose.rotation *= rotation;
            return pose;
        }

        public static Pose WithPosition(this Pose p, Vector3 pos)
        {
            p.position = pos;
            return p;
        }

        public static Pose AddLocalPosition(this Pose p, Vector3 local)
        {
            p.position += p.rotation * local;
            return p;
        }

        public static Pose AddWorldPosition(this Pose p, Vector3 local)
        {
            p.position += local;
            return p;
        }

        public static Pose WithRotation(this Pose p, Quaternion rot)
        {
            p.rotation = rot;
            return p;
        }

        public static Quaternion Subtract(this Quaternion from, Quaternion minus)
        {
            return Quaternion.Inverse(minus) * from;
        }

        // public static Pose Subtract(this Pose from, Pose minus)
        // {
        //     var inv = minus.Inverse();
        //     return new Pose(inv.rotation * (from.position + inv.position), inv.rotation * from.rotation);
        // }

        public static Pose Subtract(this Pose poseA, Pose poseB)
        {
            var relativePosition = poseA.position - poseB.position;
            var relativeRotation = Quaternion.Inverse(poseB.rotation) * poseA.rotation;
            return new Pose(relativePosition, relativeRotation);
        }

        public static Pose Reflected(this Pose p, Plane plane)
        {
            return new Pose(2f * plane.ClosestPointOnPlane(p.position) - p.position,
                Quaternion.LookRotation(Vector3.Reflect(p.forward, plane.normal), Vector3.Reflect(p.up, plane.normal)));
        }

        public static Vector3 InverseTransformDirection(this Pose pose, Vector3 direction)
        {
            return Quaternion.Inverse(pose.rotation) * direction;
        }

        public static Vector3 TransformPoint(this Pose pose, Vector3 point)
        {
            return pose.position + pose.rotation * point;
        }

        public static Quaternion TransformRotation(this Pose pose, Quaternion rotation)
        {
            return pose.rotation * rotation;
        }

        public static Quaternion InverseTransformRotation(this Pose pose, Quaternion rotation)
        {
            return Quaternion.Inverse(rotation) * pose.rotation;
        }

        public static Vector3 InverseTransformPoint(this Pose pose, Vector3 position)
        {
            return Quaternion.Inverse(pose.rotation) * (position - pose.position);
        }

        public static Quaternion SmoothDamp(this Quaternion current, Quaternion target, ref Quaternion deriv, float time)
        {
            if (Time.deltaTime < Mathf.Epsilon) return current;
            // account for double-cover
            var dot = Quaternion.Dot(current, target);
            var multi = dot > 0f ? 1f : -1f;
            target.x *= multi;
            target.y *= multi;
            target.z *= multi;
            target.w *= multi;

            // smooth damp (nlerp approx)
            var result = new Vector4(
                Mathf.SmoothDamp(current.x, target.x, ref deriv.x, time),
                Mathf.SmoothDamp(current.y, target.y, ref deriv.y, time),
                Mathf.SmoothDamp(current.z, target.z, ref deriv.z, time),
                Mathf.SmoothDamp(current.w, target.w, ref deriv.w, time)
            ).normalized;

            // ensure deriv is tangent
            var derivError = Vector4.Project(new Vector4(deriv.x, deriv.y, deriv.z, deriv.w), result);
            deriv.x -= derivError.x;
            deriv.y -= derivError.y;
            deriv.z -= derivError.z;
            deriv.w -= derivError.w;

            return new Quaternion(result.x, result.y, result.z, result.w);
        }

        public static Pose Transform(this Transform lhs, Pose rhs)
        {
            return new Pose
            {
                position = lhs.TransformPoint(rhs.position),
                rotation = lhs.TransformRotation(rhs.rotation)
            };
        }

        public static Quaternion TransformRotation(this Transform t, Quaternion localRotation)
        {
            return t.rotation * localRotation;
        }

        public static Pose InverseTransform(this Transform lhs, Pose rhs)
        {
            return new Pose
            {
                position = lhs.InverseTransformPoint(rhs.position),
                rotation = lhs.InverseTransformRotation(rhs.rotation)
            };
        }

        public static Quaternion InverseTransformRotation(this Transform t, Quaternion worldRotation)
        {
            return Quaternion.Inverse(t.rotation) * worldRotation;
        }

        public static Pose Transform(this Pose lhs, Pose rhs)
        {
            return new Pose
            {
                position = lhs.TransformPoint(rhs.position),
                rotation = lhs.TransformRotation(rhs.rotation)
            };
        }

        public static Pose InverseTransform(this Pose lhs, Pose rhs)
        {
            return new Pose
            {
                position = lhs.InverseTransformPoint(rhs.position),
                rotation = lhs.InverseTransformRotation(rhs.rotation)
            };
        }

        public static Vector3 GetPoint(this Pose p, float distance) => p.position + p.forward * distance;
        public static Vector3 GetPoint(this Transform p, float distance) => p.position + p.forward * distance;

        private static Plane _plane = default;

        public static bool CastToPlane(this Pose p, Transform t, out Vector3 worldHit) => CastToPlane(new Ray(p.position, p.forward), t.forward, t.position, out worldHit);

        public static bool CastToPlane(this Pose p, Vector3 inNormal, Vector3 inPoint, out Vector3 worldHit) =>
            CastToPlane(new Ray(p.position, p.forward), inNormal, inPoint, out worldHit);

        public static bool CastToPlane(this Transform t, Vector3 inNormal, Vector3 inPoint, out Vector3 worldHit)
        {
            return CastToPlane(new Ray(t.position, t.forward), inNormal, inPoint, out worldHit);
        }

        public static bool CastToPlane(this Ray ray, Transform t, out Vector3 worldHit) => CastToPlane(ray, t.forward, t.position, out worldHit);

        public static bool CastToPlane(this Ray ray, Vector3 inNormal, Vector3 inPoint, out Vector3 worldHit)
        {
            _plane.SetNormalAndPosition(inNormal, inPoint);
            if (!_plane.Raycast(ray, out var enter))
            {
                worldHit = default;
                return false;
            }

            worldHit = ray.origin + ray.direction * enter;
            return true;
        }

        public static float SignedPow(this float val, float pow) => Mathf.Pow(val, pow) * Mathf.Sign(val);
        public static Vector3 SignedPow(this Vector3 val, float pow) => new Vector3(SignedPow(val.x, pow), SignedPow(val.y, pow), SignedPow(val.z, pow));

        public const float FloatCompareThreshold = 0.0001f;
        public static Vector3 ForceNonZero(this Vector3 val, float minComponentSize = FloatCompareThreshold) => Vector3.Max(val, Vector3.one * minComponentSize);
        public static Vector3 ZeroIfNearZero(this Vector3 val, float threshold = FloatCompareThreshold) => val.IsNearZero(threshold) ? Vector3.zero : val;
        public static float ZeroIfNearZero(this float val, float threshold = FloatCompareThreshold) => val.IsNearZero(threshold) ? 0 : val;
        public static bool IsSameSign(this float val, float other) => Mathf.Sign(val) == Mathf.Sign(other);
        public static bool IsNearZero(this Vector3 val, float threshold = FloatCompareThreshold) => val.magnitude.IsNearZero(threshold);
        public static bool IsNearZero(this float val, float threshold = FloatCompareThreshold) => val.IsApproximately(0, threshold);

        public static bool IsApproximately(this float val, float other, float threshold = FloatCompareThreshold) => Mathf.Abs(val - other) < threshold;
        public static bool IsNotApproximately(this float val, float other, float threshold = FloatCompareThreshold) => !val.IsApproximately(other, threshold);
        public static bool IsApproximately(this Vector2 val, Vector2 other, float threshold = FloatCompareThreshold) => val.x.IsApproximately(other.x) && val.y.IsApproximately(other.y);
        public static bool IsApproximately(this Vector3 val, Vector3 other, float threshold = FloatCompareThreshold) => val.x.IsApproximately(other.x) && val.y.IsApproximately(other.y) && val.z.IsApproximately(other.z);

        public static float WithDeadzone(this float value, float threshold)
        {
            return value > 0 ? Mathf.Max(0, value - threshold) : Mathf.Min(0, value + threshold);
        }
        // public static Vector3 WithDeadzone(this Vector3 value, float threshold) => Mathf.Max(0, value.magnitude - threshold) * value.normalized;
        // public static bool IsDeadzoneExceeded(this Vector3 value, float threshold, out Vector3 result)
        // {
            // result = value.WithDeadzone(threshold);
            // return result != default;
        // }

        public static void SortSiblingOrderByZ(this List<Transform> children)
        {
            static int CompareZ(Transform lhs, Transform rhs)
            {
                var lhsZ = lhs.localPosition.z;
                var rhsZ = rhs.localPosition.z;
                if (lhsZ < rhsZ) return 1;
                if (lhsZ > rhsZ) return -1;
                return 0;
            }

            children.Sort(CompareZ);
            for (var i = 0; i < children.Count; i++)
            {
                if (i != children[i].GetSiblingIndex()) children[i].SetSiblingIndex(i);
            }
        }

        public static Renderer CreateQuadRenderer(Transform parent = null, string name = "QuadMesh", float width = 1, float height = 1)
        {
            MeshFilter filter;
            if (parent) filter = parent.CreateChild<MeshFilter>(name: name);
            else filter = new GameObject(name).AddComponent<MeshFilter>();

            filter.mesh = new Mesh
            {
                vertices = new[]
                {
                    new Vector3(-width * 0.5f, -height * 0.5f, 0),
                    new Vector3(width * 0.5f, -height * 0.5f, 0),
                    new Vector3(-width * 0.5f, height * 0.5f, 0),
                    new Vector3(width * 0.5f, height * 0.5f, 0)
                },
                triangles = new[] { 0, 2, 1, 2, 3, 1 },
                normals = new[] { -Vector3.forward, -Vector3.forward, -Vector3.forward, -Vector3.forward },
                uv = new[]
                {
                    new Vector2(0, 0),
                    new Vector2(1, 0),
                    new Vector2(0, 1),
                    new Vector2(1, 1)
                }
            };

            var renderer = filter.Ensure<MeshRenderer>();

            return renderer;
        }

        public static Renderer CreateSphereRenderer(Transform parent = null, string name = "SphereMesh", float radius = 0.5f)
        {
            MeshFilter filter;
            if (parent) filter = parent.CreateChild<MeshFilter>(name: name);
            else filter = new GameObject(name).AddComponent<MeshFilter>();

            filter.mesh = Resources.GetBuiltinResource<Mesh>("New-Sphere.fbx");
            var renderer = filter.Ensure<MeshRenderer>();

            renderer.transform.localScale = Vector3.one * radius * 2; // Unity's sphere mesh is radius 0.5f
            return renderer;
        }

        public static Material LoadMaterial(string resourcesPath) => new(Resources.Load(resourcesPath) as Material);

        public static TextMeshProUGUI CreateHeadlockedErrorText(this Component source)
        {
            var errorCanvas = source.CreateChild<Canvas>();
            errorCanvas.renderMode = RenderMode.WorldSpace;
            errorCanvas.transform.SetParent(Camera.main.transform, worldPositionStays: false);
            errorCanvas.transform.localPosition = new Vector3(0, 0, 0.35f);
            errorCanvas.transform.SetRectSize(1, 1);
            var rectTransform = errorCanvas.GetComponent<RectTransform>();
            rectTransform.SetSizeWithCurrentAnchors(RectTransform.Axis.Horizontal, 1);
            rectTransform.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, 1);

            var errorText = errorCanvas.CreateChild<TextMeshProUGUI>();
            errorText.fontSize = 0.015f;
            errorText.alignment = TextAlignmentOptions.Center;
            errorText.color = Color.red;
            errorText.transform.SetRectSize(0.25f, 0.25f);
            return errorText;
        }

        public static Vector2 WorldPositionToNormalizedRectCoordinate(this RectTransform rect, Vector3 worldPosition)
        {
            // Transform world position to keyboard rect space
            Vector3 lastCoordInLocalSpace = rect.InverseTransformPoint(worldPosition);

            // Get normalized coordinate from top left of rectangle.
            Vector2 bottomRightNormalizedCoord = PointToNormalizedUnclamped(rect.rect, new Vector2(lastCoordInLocalSpace.x, lastCoordInLocalSpace.y));

            // Remap to -1, 1
            return (2 * (bottomRightNormalizedCoord)) - Vector2.one;
        }
        
        public static Vector3 NormalizedRectCoordinateToWorldPosition(this RectTransform rect, Vector2 normalizedCoordinate)
        {
            // Remap from -1, 1
            Vector2 demapped = (normalizedCoordinate + Vector2.one) / 2f;
            
            // Denormalize coordinate to local rect space
            Vector2 localCoord = NormalizedToPointUnclamped(rect.rect, demapped);
            
            // Transform to world space
            return rect.TransformPoint(localCoord);
        }
        
        // Unclamped variant of Rect.PointToNormalized
        public static Vector2 PointToNormalizedUnclamped(Rect rectangle, Vector2 point) => new Vector2(InverseLerpUnclamped(rectangle.x, rectangle.xMax, point.x), InverseLerpUnclamped(rectangle.y, rectangle.yMax, point.y));
        
        // Unclamped variant of Mathf.InverseLerp
        public static float InverseLerpUnclamped(float a, float b, float value) { return (value - a) / (b - a); }
        
        // Unclamped variant of NormalizedToPoint
        public static Vector2 NormalizedToPointUnclamped(Rect rectangle, Vector2 normalizedRectCoordinates) => new Vector2(Mathf.LerpUnclamped(rectangle.x, rectangle.xMax, normalizedRectCoordinates.x), Mathf.LerpUnclamped(rectangle.y, rectangle.yMax, normalizedRectCoordinates.y));

        
    }
}
