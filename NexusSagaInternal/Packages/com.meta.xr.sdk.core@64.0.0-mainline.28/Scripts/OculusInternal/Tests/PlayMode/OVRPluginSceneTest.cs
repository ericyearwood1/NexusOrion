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
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEngine.TestTools;
using UnityEngine;
using UnityEngine.SceneManagement;
using System.Runtime.InteropServices;
using UnityEditorInternal;

internal class OVRPluginSceneTest : OVRPluginPlayModeTest
{
    private OVRSceneManager _sceneManager;

    [UnitySetUp]
    public override IEnumerator UnitySetUp()
    {
        yield return base.UnitySetUp();

        OVRPlugin.OVRP_1_55_1.mockObj = new FakeOVRPlugin_55_1();
        OVRPlugin.OVRP_1_64_0.mockObj = new FakeOVRPlugin_64();
        OVRPlugin.OVRP_1_72_0.mockObj = new FakeOVRPlugin_72();
        OVRPlugin.OVRP_1_79_0.mockObj = new FakeOVRPlugin_79();
        OVRPlugin.OVRP_1_84_0.mockObj = new FakeOVRPlugin_84();

        yield return LoadTestScene("Oculus/VR/Scenes/OculusInternal/SceneAnchorTests");
        _sceneManager = UnityEngine.Object.FindAnyObjectByType<OVRSceneManager>();
    }

    [UnityTearDown]
    public override IEnumerator UnityTearDown()
    {
        OVRPlugin.OVRP_1_84_0.mockObj = new OVRPlugin.OVRP_1_84_0_TEST();
        OVRPlugin.OVRP_1_79_0.mockObj = new OVRPlugin.OVRP_1_79_0_TEST();
        OVRPlugin.OVRP_1_72_0.mockObj = new OVRPlugin.OVRP_1_72_0_TEST();
        OVRPlugin.OVRP_1_64_0.mockObj = new OVRPlugin.OVRP_1_64_0_TEST();
        OVRPlugin.OVRP_1_55_1.mockObj = new OVRPlugin.OVRP_1_55_1_TEST();

        yield return base.UnityTearDown();
    }

    private class FakeOVRPlugin_55_1 : OVRPlugin.OVRP_1_55_1_TEST
    {
        private bool _spaceQueryCompleted = false;
        private ulong _requestId = 0;
        private IntPtr _eventDataTest;

        public FakeOVRPlugin_55_1()
        {
            _eventDataTest = Marshal.AllocHGlobal(4000);
        }

        ~FakeOVRPlugin_55_1()
        {
            Marshal.FreeHGlobal(_eventDataTest);
        }

        public override OVRPlugin.Result ovrp_PollEvent2(ref OVRPlugin.EventType eventType, ref IntPtr eventData)
        {
            if (_spaceQueryCompleted)
            {
                eventType = OVRPlugin.EventType.SpaceQueryComplete;
                var data = new OVRDeserialize.SpaceQueryCompleteData
                {
                    RequestId = _requestId,
                    Result = 1 // anything greater than 0
                };

                Marshal.StructureToPtr(data, _eventDataTest, true);
                eventData = _eventDataTest;
                _requestId = 0;
                _spaceQueryCompleted = false;
            }

            return OVRPlugin.Result.Success;
        }

        public void QuerySpaceResultCompleted(ulong requestId)
        {
            _requestId = requestId;
            _spaceQueryCompleted = true;
        }
    }

    private class FakeOVRPlugin_64 : OVRPlugin.OVRP_1_64_0_TEST
    {
        public override OVRPlugin.Result ovrp_LocateSpace(ref OVRPlugin.Posef location, ref ulong space,
            OVRPlugin.TrackingOrigin trackingOrigin)
        {
            return OVRPlugin.Result.Success;
        }
    }

    private class FakeOVRPlugin_72 : OVRPlugin.OVRP_1_72_0_TEST
    {
        public FakeOVRPlugin_55_1 fakeOVRPlugin55_1;

        private int _querySpaceCount = 0;

        private ulong _space = 10;

        private bool _roomLayoutChanged;

        private Guid _roomGUID = Guid.NewGuid();

        private Guid _floorGUID = Guid.NewGuid();
        private Guid _ceilingGUID = Guid.NewGuid();
        private Guid _wallGUID = Guid.NewGuid();

        private Guid _newFloorGUID = Guid.NewGuid();

        private ulong _uuidRequestId = 2;
        private ulong _roomLayoutRequestId = 3;

        private bool _lastQueriedRoomLayout = false;
        private bool _lastQueriedUuid = false;

        public override OVRPlugin.Result ovrp_GetSpaceComponentStatus(ref ulong space,
            OVRPlugin.SpaceComponentType componentType, out OVRPlugin.Bool enabled,
            out OVRPlugin.Bool changePending)
        {
            enabled = OVRPlugin.Bool.False;
            changePending = OVRPlugin.Bool.True;
            if (_lastQueriedRoomLayout && componentType == OVRPlugin.SpaceComponentType.RoomLayout)
            {
                enabled = OVRPlugin.Bool.True;
                changePending = OVRPlugin.Bool.False;
            }

            if (_lastQueriedUuid && (componentType == OVRPlugin.SpaceComponentType.Bounded2D ||
                                     componentType == OVRPlugin.SpaceComponentType.Bounded3D))
            {
                enabled = OVRPlugin.Bool.True;
                changePending = OVRPlugin.Bool.False;
            }

            if (componentType == OVRPlugin.SpaceComponentType.Locatable)
            {
                enabled = OVRPlugin.Bool.True;
                changePending = OVRPlugin.Bool.False;
            }

            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_GetSpaceBoundingBox2D(ref ulong space, out OVRPlugin.Rectf rect)
        {
            rect = default;
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_GetSpaceBoundingBox3D(ref ulong space, out OVRPlugin.Boundsf bounds)
        {
            bounds = default;
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_GetSpaceRoomLayout(ref ulong space,
            ref OVRPlugin.RoomLayoutInternal roomLayoutInternal)
        {
            // First time when it is queried, we just return number of walls.
            if (roomLayoutInternal.wallUuidCapacityInput == 0)
            {
                roomLayoutInternal.wallUuidCountOutput = 1;
                return OVRPlugin.Result.Success;
            }

            if (!_roomLayoutChanged)
            {
                roomLayoutInternal.floorUuid = _floorGUID;
            }
            else
            {
                roomLayoutInternal.floorUuid = _newFloorGUID;
            }

            roomLayoutInternal.ceilingUuid = _ceilingGUID;
            Marshal.StructureToPtr(_wallGUID, roomLayoutInternal.wallUuids, true);
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_GetSpaceSemanticLabels(ref ulong space,
            ref OVRPlugin.SpaceSemanticLabelInternal labelsInternal)
        {
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_QuerySpaces(ref OVRPlugin.SpaceQueryInfo queryInfo, out ulong requestId)
        {
            _lastQueriedRoomLayout = false;
            _lastQueriedUuid = false;
            _querySpaceCount++;
            requestId = 1;
            if (queryInfo.IdInfo.NumIds > 0)
            {
                requestId = _uuidRequestId;
                _lastQueriedUuid = true;
            }
            else
            {
                if (queryInfo.ComponentsInfo.Components.Contains(OVRPlugin.SpaceComponentType.RoomLayout))
                {
                    requestId = _roomLayoutRequestId;
                    _lastQueriedRoomLayout = true;
                }
            }

            fakeOVRPlugin55_1.QuerySpaceResultCompleted(requestId);
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_RetrieveSpaceQueryResults(ref UInt64 requestId,
            UInt32 resultCapacityInput, ref UInt32 resultCountOutput, System.IntPtr results)
        {
            // First check is for size of the results.
            if (resultCountOutput == 0)
            {
                resultCountOutput = 1;
                if (requestId == _uuidRequestId)
                {
                    resultCountOutput = 3;
                }

                return OVRPlugin.Result.Success;
            }

            if (requestId == _roomLayoutRequestId)
            {
                var room = new OVRPlugin.SpaceQueryResult();
                room.space = _space;
                room.uuid = _roomLayoutChanged ? Guid.NewGuid() : _roomGUID;

                Marshal.StructureToPtr(room, results, true);
                return OVRPlugin.Result.Success;
            }


            if (requestId == _uuidRequestId)
            {
                var floor = new OVRPlugin.SpaceQueryResult();
                floor.space = _space;

                floor.uuid = _floorGUID;
                if (!_roomLayoutChanged)
                {
                    floor.uuid = _floorGUID;
                }
                else
                {
                    floor.uuid = _newFloorGUID;
                }

                var ceiling = new OVRPlugin.SpaceQueryResult();
                ceiling.space = _space;
                ceiling.uuid = _ceilingGUID;
                var wall = new OVRPlugin.SpaceQueryResult();
                wall.space = _space;
                wall.uuid = _wallGUID;

                Marshal.StructureToPtr(floor, results, true);
                Marshal.StructureToPtr(ceiling, results + Marshal.SizeOf(floor), true);
                Marshal.StructureToPtr(wall, results + Marshal.SizeOf(floor) * 2, true);

                return OVRPlugin.Result.Success;
            }

            return OVRPlugin.Result.Success;
        }

        public int GetQuerySpaceCount() => _querySpaceCount;
        public void SetRoomLayoutChanged(bool value) => _roomLayoutChanged = value;
    }

    private class FakeOVRPlugin_79 : OVRPlugin.OVRP_1_79_0_TEST
    {
#if OVR_INTERNAL_CODE
        public override OVRPlugin.Result ovrp_GetPlaneTrackingSupported(out OVRPlugin.Bool planeTrackingSupported)
        {
            planeTrackingSupported = OVRPlugin.Bool.False;
            return OVRPlugin.Result.Success;
        }
#endif
    }

    private class FakeOVRPlugin_84 : OVRPlugin.OVRP_1_84_0_TEST
    {
        // QPL
        public override OVRPlugin.Result ovrp_QplMarkerStart(int markerId, int instanceKey, long timestampMs) =>
            OVRPlugin.Result.Success;

        public override OVRPlugin.Result ovrp_QplMarkerEnd(int markerId, OVRPlugin.Qpl.ResultType resultTypeId,
            int instanceKey, long timestampMs) => OVRPlugin.Result.Success;

        public override OVRPlugin.Result ovrp_QplMarkerPointCached(int markerId, int nameHandle,
            int instanceKey, long timestampMs) => OVRPlugin.Result.Success;

        public override OVRPlugin.Result ovrp_QplMarkerAnnotation(int markerId,
            string annotationKey,
            string annotationValue, int instanceKey) => OVRPlugin.Result.Success;

        public override OVRPlugin.Result ovrp_QplCreateMarkerHandle(string name, out int nameHandle)
        {
            nameHandle = 1;
            return OVRPlugin.Result.Success;
        }

        public override OVRPlugin.Result ovrp_QplDestroyMarkerHandle(int nameHandle) => OVRPlugin.Result.Success;
    }
}

#endif
