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

#if OVRPLUGIN_TESTING

using NUnit.Framework;
using UnityEngine;

public class OVRMockMeshCreator
{
    static Vector3[] Vertex =
    {
        new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(1, 1, 0)
    };

    static Vector2[] UV_MaterialDisplay =
    {
        new Vector2(0, 0), new Vector2(1, 0), new Vector2(0, 1),
        new Vector2(1, 1) // 4 UV with all directions! (Plane has 4 uvMaps)
    };

    static Vector3[] SingleVectorArray = { new Vector3(0, 0, 0) };

    static int[] Triangles = new int[6]; // 2 Triangle combinations (2*3=6 vertices/vertexes)

    /// <summary>
    /// Creates a dummy Mesh with provided blend shapes
    /// </summary>
    /// <param name="blendShapeNames">array of blend shape names</param>
    /// <returns></returns>
    public static Mesh GenerateMockMesh(string[] blendShapeNames)
    {
        Assert.NotNull(blendShapeNames);
        Assert.Greater(blendShapeNames.Length, 0);

        Mesh mesh = new Mesh();
        mesh.name = "MyMockMesh";

        mesh.vertices = Vertex;
        mesh.triangles = Triangles;
        mesh.uv = UV_MaterialDisplay;

        mesh.RecalculateNormals();
        mesh.Optimize();

        Assert.AreEqual(mesh.vertexCount, 1);

        for (int i = 0; i < blendShapeNames.Length; i++)
        {
            mesh.AddBlendShapeFrame(blendShapeNames[i], 0, SingleVectorArray, SingleVectorArray, SingleVectorArray);
        }

        Assert.AreEqual(blendShapeNames.Length, mesh.blendShapeCount);
        for (int i = 0; i < blendShapeNames.Length; i++)
        {
            Assert.AreEqual(blendShapeNames[i], mesh.GetBlendShapeName(i));
        }

        return mesh;
    }
}

#endif // OVRPLUGIN_TESTING
