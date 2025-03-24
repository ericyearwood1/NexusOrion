/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * Licensed under the Oculus SDK License Agreement (the "License");
 * you may not use the Oculus SDK except in compliance with the License,
 * which is provided at the time of installation or download, or which
 * otherwise accompanies this software in either electronic or hard copy form.
 *
 * You may obtain a copy of the License at
 *
 * https://developer.oculus.com/licenses/oculussdk/
 *
 * Unless required by applicable law or agreed to in writing, the Oculus SDK
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if OVR_INTERNAL_CODE

using System;
using System.Collections.Generic;
using UnityEngine;

namespace Meta.XR.ImmersiveDebugger
{
    public enum DebugGizmoType
    {
        None = 0,
        /// <summary>
        /// Accepting type: Pose<br/>
        /// Drawing an Axis from a Pose data.
        /// </summary>
        Axis = 1,
        /// <summary>
        /// Accepting type: Vector3<br/>
        /// Drawing a Point from a Vector3 data.
        /// </summary>
        Point,
        /// <summary>
        /// Accepting type: Tuple&lt;Vector3 start, Vector3 end&gt;<br/>
        /// Drawing a Line from two Vector3 data representing start/end of the line.
        /// </summary>
        Line,
        /// <summary>
        /// Accepting type: Vector3[]<br/>
        /// Drawing Lines from list of Vector3 data representing connected points of the lines.
        /// </summary>
        Lines,
        /// <summary>
        /// Accepting type: Tuple&lt;Pose pivot, float width, float height&gt;<br/>
        /// Drawing a Plane from the pivot, width and height.
        /// </summary>
        Plane,
        /// <summary>
        /// Accepting type: Tuple&lt;Vector3 center, float size&gt;<br/>
        /// Drawing a regular Cube from the center and size (width/height/depth are all the same).
        /// </summary>
        Cube,
        /// <summary>
        /// Accepting type: Tuple&lt;Pose pivot, float width, float height, float depth&gt;<br/>
        /// Drawing a box from the top-centered pivot and its width, height, depth lengths.<br/>
        /// Pivot is at the center of the top surface (like Scene Volume).
        /// </summary>
        TopCenterBox,
        /// <summary>
        /// Accepting type: Tuple&lt;Pose pivot, float width, float height, float depth&gt;<br/>
        /// Drawing a normal box from the pivot and its width, height, depth lengths.<br/>
        /// Pivot is at the mass center of the box.
        /// </summary>
        Box,
    }
}

namespace Meta.XR.ImmersiveDebugger.Gizmo
{
    internal struct GizmoTypeInfo
    {
        public readonly Type DataSourceType;
        public readonly Action<object> RenderDelegate;
        public GizmoTypeInfo(Type dataSourceType, Action<object> renderDelegate)
        {
            DataSourceType = dataSourceType;
            RenderDelegate = renderDelegate;
        }
    }

    internal static class GizmoTypesRegistry
    {
        private static Dictionary<DebugGizmoType, GizmoTypeInfo> _gizmoTypeInfos = new Dictionary<DebugGizmoType, GizmoTypeInfo>();

        public static void RegisterGizmoType(DebugGizmoType gizmoType, Type dataSourceType, Action<object> renderDelegate)
        {
            _gizmoTypeInfos.Add(gizmoType, new GizmoTypeInfo(dataSourceType, renderDelegate));
        }

        public static void InitGizmos()
        {
            RegisterGizmoType(DebugGizmoType.Axis, typeof(Pose), dataSource =>
            {
                if (dataSource is Pose pose)
                {
                    DebugGizmos.DrawAxis(pose);
                }
            });
            RegisterGizmoType(DebugGizmoType.Point, typeof(Vector3), dataSource =>
            {
                if (dataSource is Vector3 position)
                {
                    DebugGizmos.DrawPoint(position);
                }
            });
            RegisterGizmoType(DebugGizmoType.Line, typeof(Tuple<Vector3, Vector3>), dataSource =>
            {
                if (dataSource is Tuple<Vector3, Vector3> lineStartEndPair)
                {
                    DebugGizmos.DrawLine(lineStartEndPair.Item1, lineStartEndPair.Item2);
                }
            });
            RegisterGizmoType(DebugGizmoType.Lines, typeof(Vector3[]), dataSource =>
            {
                if (dataSource is Vector3[] lines)
                {
                    for (int i = 1; i < lines.Length; i++)
                    {
                        DebugGizmos.DrawLine(lines[i-1], lines[i]);
                    }
                }
            });
            RegisterGizmoType(DebugGizmoType.Plane, typeof(Tuple<Pose, float, float>), dataSource =>
            {
                if (dataSource is Tuple<Pose, float, float> planeData)
                {
                    DebugGizmos.DrawPlane(planeData.Item1, planeData.Item2, planeData.Item3);
                }
            });
            RegisterGizmoType(DebugGizmoType.Cube, typeof(Tuple<Vector3, float>), dataSource =>
            {
                if (dataSource is Tuple<Vector3, float> cubeData)
                {
                    DebugGizmos.DrawWireCube(cubeData.Item1, cubeData.Item2);
                }
            });
            RegisterGizmoType(DebugGizmoType.TopCenterBox, typeof(Tuple<Pose, float, float, float>), dataSource =>
            {
                if (dataSource is Tuple<Pose, float, float, float> boxData)
                {
                    DebugGizmos.DrawBox(boxData.Item1, boxData.Item2, boxData.Item3, boxData.Item4, true);
                }
            });
            RegisterGizmoType(DebugGizmoType.Box, typeof(Tuple<Pose, float, float, float>), dataSource =>
            {
                if (dataSource is Tuple<Pose, float, float, float> boxData)
                {
                    DebugGizmos.DrawBox(boxData.Item1, boxData.Item2, boxData.Item3, boxData.Item4, false);
                }
            });
        }

        public static bool IsValidDataTypeForGizmoType(Type type, DebugGizmoType gizmoType)
        {
            if (_gizmoTypeInfos.TryGetValue(gizmoType, out GizmoTypeInfo typeInfo))
            {
                return type == typeInfo.DataSourceType;
            }
            Debug.LogWarning($"{gizmoType} not found in GizmoTypeInfos, please registerGizmoType.");
            return false;
        }

        public static void RenderGizmo(DebugGizmoType type, object dataSource)
        {
            if (_gizmoTypeInfos.TryGetValue(type, out GizmoTypeInfo typeInfo) && dataSource != null)
            {
                typeInfo.RenderDelegate(dataSource);
            }
        }
#if OVRPLUGIN_TESTING

        public static void Reset()
        {
            _gizmoTypeInfos.Clear();
        }
#endif
    }
}

#endif // OVR_INTERNAL_CODE
