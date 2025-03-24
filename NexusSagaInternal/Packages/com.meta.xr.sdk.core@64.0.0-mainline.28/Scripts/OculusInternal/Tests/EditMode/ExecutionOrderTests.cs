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

using System;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

[TestFixture]
public class ExecutionOrderTests
{
    private static int GetExecutionOrder<T>() where T : MonoBehaviour
    {
        return GetExecutionOrderInternal(go => go.AddComponent<T>());
    }

    private static int GetExecutionOrder(Type type)
    {
        return GetExecutionOrderInternal(go => (MonoBehaviour)go.AddComponent(type));
    }

    private static int GetExecutionOrderInternal(Func<GameObject, MonoBehaviour> addComponent)
    {
        var go = new GameObject();
        var ms = MonoScript.FromMonoBehaviour(addComponent(go));
        var executionOrder = MonoImporter.GetExecutionOrder(ms);
        UnityEngine.Object.DestroyImmediate(go);
        return executionOrder;
    }

    private static Type[] _afterOVRManager =
    {
        typeof(OVRSpatialAnchor), typeof(OVRBody), typeof(OVRHand), typeof(OVRCustomSkeleton), typeof(OVREyeGaze),
        typeof(OVRFaceExpressions), typeof(OVRMesh), typeof(OVRSkeleton),
        typeof(OVRFace), typeof(OVRSkeletonRenderer), typeof(OVRCameraRig), typeof(OVRPassthroughLayer),
        typeof(OVRGrabber), typeof(OVRTrackedKeyboard), typeof(OVRVirtualKeyboard)
#if OVR_INTERNAL_CODE
        , typeof(OVREyeGazeInteraction)
#endif //  OVR_INTERNAL_CODE
    };

    [Test]
    public void OVRManagerBefore([ValueSource(nameof(_afterOVRManager))] Type type)
    {
        Assert.Less(GetExecutionOrder<OVRManager>(), GetExecutionOrder(type));
    }

    [Test]
    public void OVRCameraRigBeforeOVRSpatialAnchor()
    {
        Assert.Less(GetExecutionOrder<OVRCameraRig>(), GetExecutionOrder<OVRSpatialAnchor>());
    }

    private static Type[] _beforeOVRSkeleton =
    {
        typeof(OVRHand), typeof(OVRBody)
    };

    [Test]
    public void OVRSkeletonAfter([ValueSource(nameof(_beforeOVRSkeleton))] Type type)
    {
        Assert.Greater(GetExecutionOrder<OVRSkeleton>(), GetExecutionOrder(type));
        Assert.Greater(GetExecutionOrder<OVRCustomSkeleton>(), GetExecutionOrder(type));
    }

    private static Type[] _beforeOVRSkeletonRenderer =
    {
        typeof(OVRSkeleton), typeof(OVRCustomSkeleton)
    };

    [Test]
    public void OVRSkeletonRendererAfter([ValueSource(nameof(_beforeOVRSkeletonRenderer))] Type type)
    {
        Assert.Greater(GetExecutionOrder<OVRSkeletonRenderer>(), GetExecutionOrder(type));
    }

    private static Type[] _beforeOVRCameraRig =
    {
        typeof(OVRManager), typeof(OVRScreenFade), typeof(OVRBody), typeof(OVRHand), typeof(OVRCustomSkeleton),
        typeof(OVREyeGaze), typeof(OVRFaceExpressions), typeof(OVRMesh),
        typeof(OVRSkeleton), typeof(OVRFace), typeof(OVRSkeletonRenderer)
#if OVR_INTERNAL_CODE
        , typeof(OVREyeGazeInteraction)
#endif //  OVR_INTERNAL_CODE
    };

    [Test]
    public void OVRCameraRigAfter([ValueSource(nameof(_beforeOVRCameraRig))] Type type)
    {
        Assert.Greater(GetExecutionOrder<OVRCameraRig>(), GetExecutionOrder(type));
    }
}

#endif // OVRPLUGIN_TESTING
