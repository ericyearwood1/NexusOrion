using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static class Line
    {
        public static void Intersect(Plane plane, CapsuleCollider capsule)
        {
            Intersect(plane, capsule.transform, capsule.center, capsule.height, capsule.radius);
        }

        public static void Intersect(Plane plane, Transform capsuleTransform, Vector3 localCenter, float localHeight, float localRadius)
        {
            var worldHeight = localHeight * capsuleTransform.lossyScale.y;
            var worldRadius = localRadius * Mathf.Max(capsuleTransform.lossyScale.x, capsuleTransform.lossyScale.z);
            Intersect(plane, capsuleTransform.up, worldHeight, worldRadius, capsuleTransform.TransformPoint(localCenter));
        }

        public static void Intersect(Plane plane, Vector3 capsuleUp, float worldHeight, float worldRadius, Vector3 worldCenter)
        {
            var circleOffset = capsuleUp * (worldHeight / 2 - worldRadius);
            var a = worldCenter - circleOffset;
            var b = worldCenter + circleOffset;

            var aPlane = plane.ClosestPointOnPlane(a);
            var bPlane = plane.ClosestPointOnPlane(b);

            var aToBPlane = bPlane - aPlane;
            var side1Offset = Vector3.Cross(plane.normal, aToBPlane.normalized) * worldRadius;
            var side2Offset = side1Offset * -1;

            Circle(aPlane, plane.normal, worldRadius, sweep: 0.5f, startDir: side1Offset.normalized);
            Circle(bPlane, plane.normal, worldRadius, sweep: 0.5f, startDir: side2Offset.normalized);
            Draw(aPlane + side1Offset, bPlane + side1Offset);
            Draw(aPlane + side2Offset, bPlane + side2Offset);
        }

        public static void Draw(Target target)
        {
            if (!target) return;
            foreach (var c in target.Colliders) Draw(c);
        }

        public static void Draw(GameObject gameObject)
        {
            if (gameObject.TryGetComponent(out Collider c)) Draw(c);
            else if (gameObject.TryGetComponent(out Renderer r)) Draw(r);
            else if (gameObject.TryGetComponent(out RectTransform rt)) Draw(rt);
        }

        public static void Draw(RectTransform rt)
        {
            Vector3[] v = new Vector3[4];
            rt.GetWorldCorners(v);
            DrawRectFromVectorArray(v);
        }

        public static void Draw(Collider col)
        {
            if (col is CapsuleCollider capsuleCollider)
            {
                // Capsule(capsuleCollider.transform.ToPlaneForward(), capsuleCollider);
                // Capsule(capsuleCollider.transform.ToPlaneRight(), capsuleCollider);
                var candidatePos = col.transform.position;
                var toCamera = Camera.main.transform.position - candidatePos;
                var plane = new Plane(toCamera.normalized, candidatePos);
                Intersect(plane, capsuleCollider);
                return;
            }

            CreateBox(col).Draw();
        }

        public static Box CreateBox(Collider col)
        {
            return col switch
            {
                SphereCollider sphereCollider => CreateBox(sphereCollider),
                BoxCollider boxCol => CreateBox(boxCol),
                CapsuleCollider capsuleCol => CreateBox(capsuleCol),
                MeshCollider meshCol => CreateBox(meshCol),
                _ => Box.FromCenterSize(Vector3.zero, Vector3.one, col.transform)
            };
        }

        public static Box CreateBox(MeshCollider mesh)
        {
            return Box.FromLocalBounds(mesh.sharedMesh.bounds, mesh.transform);
        }

        public static Box CreateBox(CapsuleCollider capsule)
        {
            return Box.FromCenterSize(capsule.center, new Vector3(capsule.radius * 2f, capsule.height, capsule.radius * 2f), capsule.transform);
        }

        public static Box CreateBox(BoxCollider box)
        {
            return Box.FromCenterSize(box.center, box.size, box.transform);
        }

        public static Box CreateBox(SphereCollider sphere)
        {
            return Box.FromCenterSize(sphere.center, Vector3.one * sphere.radius * 2, sphere.transform);
        }

        public static void Draw(Renderer rend)
        {
            Box.FromLocalBounds(rend.localBounds, rend.transform).Draw();
        }

        public static void AngularDegrees(Vector3 worldPoint, float angleDegrees, Camera camera = null)
        {
            if (angleDegrees <= 0) return;
            if (camera == null) camera = Camera.main;
            var fromCamera = camera.transform.position - worldPoint;
            var radius = Mathf.Tan(angleDegrees * Mathf.Deg2Rad) * fromCamera.magnitude;
            Circle(worldPoint, fromCamera.normalized, radius);
        }

        public static void Circle(Vector3 position, Vector3 normal, float radius, int segments = 20, Vector3 startDir = default, float sweep = 1.0f)
        {
            if (startDir == default) startDir = Vector3.Cross(normal.normalized, Vector3.up);

            var segmentRotation = Quaternion.AngleAxis(360f / segments * sweep, normal.normalized);
            var prevPoint = startDir.normalized * radius;
            for (var i = 0; i < segments; ++i)
            {
                var newPoint = segmentRotation * prevPoint;
                Draw(position + prevPoint, position + newPoint);
                prevPoint = newPoint;
            }
        }

        public static void Draw(Vector3 start, Vector3 end, Color color, float lineWidth = default)
        {
            if (lineWidth != default) DebugGizmos.LineWidth = lineWidth;
            DebugGizmos.Color = color;
            Draw(start, end);
        }

        public static void Draw(Vector3 start, Vector3 end)
        {
            if (!Application.isPlaying)
            {
                Debug.DrawLine(start, end, DebugGizmos.Color);
                return;
            }

            DebugGizmos.DrawLine(start, end);
        }

        public static void DrawRectFromVectorArray(Vector3[] array)
        {
            Draw(array[0], array[1]);
            Draw(array[1], array[2]);
            Draw(array[2], array[3]);
            Draw(array[3], array[0]);
        }

        public static void DistanceBetween(List<GameObject> gameObjects)
        {
            for (int i = 1; i < gameObjects.Count; i++)
            {
                var pos1 = gameObjects[i].transform.position;
                var pos2 = gameObjects[i - 1].transform.position;
                Draw(pos1, pos2);
                // Handles.Label(Vector3.Lerp(pos1, pos2, 0.5f), DistanceInUnits(Vector3.Distance(pos1, pos2)), style);
            }
        }

        public struct Box
        {
            private Transform _transform;
            public Transform Transform => _transform;
            public Vector3[] LocalVerts => _localVerts;
            public Vector3[] WorldVerts => _worldVerts;
            private Vector3[] _localVerts;
            private Vector3[] _worldVerts;

            public Box(Transform transform, Vector3[] localVerts)
            {
                _transform = transform;
                _localVerts = localVerts;
                _worldVerts = new Vector3[localVerts.Length];
                for (int i = 0; i < localVerts.Length; i++) _worldVerts[i] = transform.TransformPoint(localVerts[i]);
            }

            public void Draw()
            {
                var array = WorldVerts;

                Line.Draw(array[5], array[1]);
                Line.Draw(array[1], array[7]);
                Line.Draw(array[7], array[3]);
                Line.Draw(array[3], array[5]);

                Line.Draw(array[2], array[6]);
                Line.Draw(array[6], array[4]);
                Line.Draw(array[4], array[0]);
                Line.Draw(array[0], array[2]);

                Line.Draw(array[5], array[2]);
                Line.Draw(array[1], array[6]);
                Line.Draw(array[7], array[4]);
                Line.Draw(array[3], array[0]);
            }

            public static Box FromLocalBounds(Bounds localBounds, Transform transform)
            {
                var box = FromMinMax(localBounds.min, localBounds.max, transform);
                return box;
            }

            public static Box FromCenterSize(Vector3 center, Vector3 size, Transform transform)
            {
                return FromMinMax(center + size * -0.5f, center + size * 0.5f, transform);
            }

            public static Box FromMinMax(Vector3 min, Vector3 max, Transform transform)
            {
                var box = new Box(transform: transform, localVerts: new[]
                {
                    min,
                    max,
                    new(min.x, min.y, max.z),
                    new(min.x, max.y, min.z),
                    new(max.x, min.y, min.z),
                    new(min.x, max.y, max.z),
                    new(max.x, min.y, max.z),
                    new(max.x, max.y, min.z),
                });
                return box;
            }
        }
    }
}
