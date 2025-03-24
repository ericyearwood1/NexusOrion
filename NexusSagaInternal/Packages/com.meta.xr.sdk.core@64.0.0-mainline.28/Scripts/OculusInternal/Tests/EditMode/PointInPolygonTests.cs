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

using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using UnityEngine;

[TestFixture]
class PointInPolygonTests
{
    [Test]
    public void TestPointInPolygon()
    {
        int N = 500;
        List<Vector2> boundary = new List<Vector2>();

        boundary.Add(Vector2.zero);
        boundary.Add(new Vector2(2, 4));
        boundary.Add(new Vector2(6, 4));
        boundary.Add(new Vector2(8, 1));
        boundary.Add(new Vector2(6, -3));

        for (int i = 0; i < N; i++)
        {
            var v = new Vector2(Random.Range(-5f, 10f), Random.Range(-5f, 10f));
            Assert.AreEqual(OVRSceneManager.PointInPolygon2D(boundary.ToNativeArray(Allocator.Temp), v),
                InsideConvexPolygon(boundary, v));
        }
    }

    private bool InsideConvexPolygon(List<Vector2> edges, Vector2 point)
    {
        for (var i = 0; i < edges.Count; i++)
        {
            var p1 = edges[i];
            var p2 = edges[(i + 1) % edges.Count];

            var A = -(p2.y - p1.y);
            var B = p2.x - p1.x;
            var C = -(A * p1.x + B * p1.y);
            var D = A * point.x + B * point.y + C;
            if (D > 0) return false;
        }
        return true;
    }
}

#endif // OVRPLUGIN_TESTING
