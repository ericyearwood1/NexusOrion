// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System.Collections.Generic;
using TMPro;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static class DebugUtilities
    {
        static Material _debugMaterial => new Material(Shader.Find("Unlit/Color"));
        static Camera camera => Camera.main;

        public static GameObject DebugPosition(this Vector3 position, Color color, float duration = 3.0f, string label = "")
        {
            GameObject g = CreatePoint(position, color);
            CreateLabel(g, label);

            if (duration < Mathf.Infinity)
            {
                GameObject.Destroy(g.gameObject, duration);
            }

            return g;
        }

        public static GameObject DebugRay(this Ray ray, Color color, float duration = 3.0f)
        {
            float defaultDistance = 2.0f;
            GameObject g = (ray.direction.normalized * defaultDistance).DebugVector(ray.origin, color, duration);
            return g;
        }

        public static GameObject DebugLine(this Vector3 point1, Vector3 point2, Color color, float duration = 3.0f, string point1Label = "", string point2Label = "")
        {
            GameObject g = CreateLine(point1, point2, color, $"{point1.ToString("F3")} {point2.ToString("F3")}");

            GameObject point1Cap = CreatePoint(point1, color);
            CreateLabel(point1Cap, point1Label);
            point1Cap.transform.parent = g.transform;
            GameObject point2Cap = CreatePoint(point2, color);
            CreateLabel(point2Cap, point2Label);
            point2Cap.transform.parent = g.transform;

            if (duration < Mathf.Infinity)
            {
                GameObject.Destroy(g.gameObject, duration);
            }

            return g;
        }

        public static GameObject DebugVector(this Vector3 vector, Vector3 origin, Color color, float duration = 3.0f, string label = "")
        {
            GameObject g = CreateLine(origin, origin + vector, color, vector.ToString("F3"));
            CreateLabel(g, label);

            if (duration < Mathf.Infinity)
            {
                GameObject.Destroy(g.gameObject, duration);
            }

            return g;
        }

        public static void AddToList(this GameObject g, List<GameObject> list)
        {
            list.Add(g);
        }

        static GameObject CreateLine(Vector3 position1, Vector3 position2, Color color, string name)
        {
            float defaultWidth = 0.002f;

            // Instantiate LineRenderer
            LineRenderer lr = CreateLineRenderer();
            lr.gameObject.name = $"DEBUG | {name} | {Time.time}";
            lr.startWidth = defaultWidth;
            lr.endWidth = defaultWidth;
            lr.material = _debugMaterial;
            lr.material.color = color;
            lr.positionCount = 2;
            lr.gameObject.transform.position = position1;
            lr.SetPosition(0, position1);
            lr.SetPosition(1, position2);

            return lr.gameObject;
        }
        
        public static GameObject CreatePoint(Vector3 position, Color color)
        {
            float scale = 0.005f;

            GameObject g = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            g.transform.position = position;
            g.gameObject.name = $"DEBUG | {position.ToString("F3")} | {Time.time}";
            g.transform.localScale = Vector3.one * scale;
            Object.Destroy(g.GetComponent<Collider>());
            g.GetComponent<Renderer>().material = _debugMaterial;
            g.GetComponent<Renderer>().material.color = color;

            return g;
        }

        static void CreateLabel(GameObject source, string label)
        {
            if (label == "") return;

            GameObject l = new GameObject("DebugUtilitiesCreateLabel");
            l.transform.localScale = Vector3.one * 0.005f;
            l.transform.position = source.transform.position + (Vector3.up * 0.01f);
            l.transform.rotation = LookAtY(l.transform.position, camera.transform.position);
            l.transform.parent = source.transform;
            TextMeshPro t = l.AddComponent<TextMeshPro>();
            t.autoSizeTextContainer = true;
            t.alignment = TextAlignmentOptions.Midline;
            t.fontSize = 10;
            t.text = label;
        }

        static Quaternion LookAtY(Vector3 source, Vector3 target)
        {
            Vector3 lookPos = source - target;
            lookPos.y = 0;
            return Quaternion.LookRotation(lookPos);
        }

        static LineRenderer CreateLineRenderer()
        {
            GameObject g = new GameObject("DebugLineRenderer");
            return g.AddComponent<LineRenderer>();
        }

    }
}
