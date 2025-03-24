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

using System;
using System.Runtime.InteropServices;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using static OVRPlugin;

//-------------------------------------------------------------------------------------
/// <summary>
/// Collection of helper methods to facilitate data deserialization
/// </summary>
internal static class OVRDeserialize
{
    public static T ByteArrayToStructure<T>(byte[] bytes) where T : struct
    {
        T stuff;
        GCHandle handle = GCHandle.Alloc(bytes, GCHandleType.Pinned);
        try
        {
            stuff = (T)Marshal.PtrToStructure(handle.AddrOfPinnedObject(), typeof(T));
        }
        finally
        {
            handle.Free();
        }

        return stuff;
    }

    public struct DisplayRefreshRateChangedData
    {
        public float FromRefreshRate;
        public float ToRefreshRate;
    }

    public struct SpaceQueryResultsData
    {
        public UInt64 RequestId;
    }

    public struct SpaceQueryCompleteData
    {
        public UInt64 RequestId;
        public int Result;
    }

    public struct SceneCaptureCompleteData
    {
        public UInt64 RequestId;
        public int Result;
    }

#if OVR_INTERNAL_CODE
    public struct CreatePlaneTrackerResultsData
    {
        public UInt64 Request;
        public int Result;
    }

    public struct DestroyPlaneTrackerResultsData
    {
        public UInt64 Request;
        public int Result;
    }
#endif // OVR_INTERNAL_CODE

#if OVR_INTERNAL_CODE
    public struct CreateSceneObjectTrackerResultsData
    {
        public UInt64 RequestId;
        public int Result;
    }

    public struct DestroySceneObjectTrackerResultsData
    {
        public UInt64 RequestId;
        public int Result;
    }
#endif // OVR_INTERNAL_CODE

#if OVR_INTERNAL_CODE
    public struct CreateSceneReconstructionTrackerResultsData
    {
        public UInt64 RequestId;
        public int Result;
    }

    public struct DestroySceneReconstructionTrackerResultsData
    {
        public UInt64 RequestId;
        public int Result;
    }

    public struct UpdateSceneReconstructionTrackerSettingsResultsData
    {
        public UInt64 RequestId;
        public int Result;
    }

    public struct UpdateSceneReconstructionTrackerRoomboxResultsData
    {
        public UInt64 RequestId;
        public int Result;
    }
#endif // OVR_INTERNAL_CODE

    public struct SpatialAnchorCreateCompleteData
    {
        public UInt64 RequestId;
        public int Result;
        public UInt64 Space;
        public Guid Uuid;
    }

    public struct SpaceSetComponentStatusCompleteData
    {
        public UInt64 RequestId;
        public int Result;
        public UInt64 Space;
        public Guid Uuid;
        public OVRPlugin.SpaceComponentType ComponentType;
        public int Enabled;
    }

    public struct SpaceSaveCompleteData
    {
        public UInt64 RequestId;
        public UInt64 Space;
        public int Result;
        public Guid Uuid;
    }

    public struct SpaceEraseCompleteData
    {
        public UInt64 RequestId;
        public int Result;
        public Guid Uuid;
        public OVRPlugin.SpaceStorageLocation Location;
    }

    public struct SpaceShareResultData
    {
        public UInt64 RequestId;

        public int Result;
    }

    public struct SpaceListSaveResultData
    {
        public UInt64 RequestId;

        public int Result;
    }

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE
    [StructLayout(LayoutKind.Explicit, Size = 40, Pack = 1)]
    public struct LocalGroupStartCompleteData
    {
        [FieldOffset(4)]
        public UInt64 CreateRequestId;

        [FieldOffset(12)]
        public UInt64 CreatorParticipantId;

        [FieldOffset(20)]
        public Int32 Result;

        [FieldOffset(24)]
        public Guid GroupUuid;
    }

    [StructLayout(LayoutKind.Explicit, Size = 20, Pack = 1)]
    public struct LocalGroupDiscoveryCompleteData
    {
        [FieldOffset(4)]
        public UInt64 DiscoveryRequestId;

        [FieldOffset(12)]
        public Int32 Result;

        [FieldOffset(16)]
        public OVRPlugin.LocalGroupDiscoveryStopReason Reason;
    }

    [StructLayout(LayoutKind.Explicit, Size = 1052, Pack = 1)]
    public struct LocalGroupDiscoveredResultData
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 1024)]
        [FieldOffset(0)]
        public string GroupAppInfo;

        [FieldOffset(1028)]
        public UInt64 DiscoveryRequestId;

        [FieldOffset(1036)]
        public Guid GroupUuid;
    }

    [StructLayout(LayoutKind.Explicit, Size = 1048, Pack = 1)]
    public struct LocalGroupJoinCompleteData
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 1024)]
        [FieldOffset(0)]
        public string GroupAppInfo;

        [FieldOffset(1028)]
        public UInt64 JoinRequestId;

        [FieldOffset(1036)]
        public UInt64 JoinerParticipantId;

        [FieldOffset(1044)]
        public Int32 Result;
    }

    [StructLayout(LayoutKind.Explicit, Size = 16, Pack = 1)]
    public struct LocalGroupLeaveCompleteData
    {
        [FieldOffset(4)]
        public UInt64 LeaveRequestId;

        [FieldOffset(12)]
        public Int32 Result;
    }

    [StructLayout(LayoutKind.Explicit, Size = 1052, Pack = 1)]
    public struct LocalGroupParticipantJoinedData
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 1024)]
        [FieldOffset(0)]
        public string ParticipantAppInfo;

        [FieldOffset(1024)]
        public Guid GroupUuid;

        [FieldOffset(1044)]
        public UInt64 ParticipantId;
    }

    [StructLayout(LayoutKind.Explicit, Size = 28, Pack = 1)]
    public struct LocalGroupParticipantLeftData
    {
        [FieldOffset(0)]
        public Guid GroupUuid;

        [FieldOffset(20)]
        public UInt64 ParticipantId;
    }

    [StructLayout(LayoutKind.Explicit, Size = 16, Pack = 1)]
    public struct LocalGroupSetDiscoverabilityModeCompleteData
    {
        [FieldOffset(4)]
        public UInt64 SetDiscoverabilityModeRequestId;

        [FieldOffset(12)]
        public Int32 Result;
    }

    [StructLayout(LayoutKind.Explicit, Size = 16, Pack = 1)]
    public struct LocalGroupSetAppInfoCompleteData
    {
        [FieldOffset(4)]
        public UInt64 SetAppInfoRequestId;

        [FieldOffset(12)]
        public Int32 Result;
    }

    [StructLayout(LayoutKind.Explicit, Size = 16, Pack = 1)]
    public struct LocalGroupParticipantSetAppInfoCompleteData
    {
        [FieldOffset(4)]
        public UInt64 SetParticipantAppInfoRequestId;

        [FieldOffset(12)]
        public Int32 Result;
    }

    [StructLayout(LayoutKind.Explicit, Size = 24, Pack = 1)]
    public struct LocalGroupDiscoverabilityModeUpdatedData
    {
        [FieldOffset(0)]
        public Guid GroupUuid;

        [FieldOffset(20)]
        public OVRPlugin.LocalGroupDiscoverabilityMode DiscoverabilityMode;
    }

    [StructLayout(LayoutKind.Explicit, Size = 1040, Pack = 1)]
    public struct LocalGroupAppInfoUpdatedData
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 1024)]
        [FieldOffset(0)]
        public string GroupAppInfo;

        [FieldOffset(1024)]
        public Guid GroupUuid;
    }

    [StructLayout(LayoutKind.Explicit, Size = 1052, Pack = 1)]
    public struct LocalGroupParticipantAppInfoUpdatedData
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 1024)]
        [FieldOffset(0)]
        public string ParticipantAppInfo;

        [FieldOffset(1024)]
        public Guid GroupUuid;

        [FieldOffset(1044)]
        public UInt64 ParticipantId;
    }

    [StructLayout(LayoutKind.Explicit, Size = 20, Pack = 1)]
    public struct LocalGroupParticipantsQueryResultsReadyData
    {
        [FieldOffset(4)]
        public UInt64 QueryRequestId;

        [FieldOffset(12)]
        public Int32 Result;

        [FieldOffset(16)]
        public UInt32 ParticipantsCount;
    }

    [StructLayout(LayoutKind.Explicit, Size = 16, Pack = 1)]
    public struct LocalGroupShareSpacesCompleteData
    {
        [FieldOffset(4)]
        public UInt64 RequestId;

        [FieldOffset(12)]
        public LocalGroupShareSpacesResult Result;
    }

   	public const int SpacesSharedMaxCount = 32;
    [StructLayout(LayoutKind.Explicit, Size = 536, Pack = 1)]
    public struct LocalGroupSpacesSharedData
    {
        [FieldOffset(4)]
        public Guid GroupUuid;

        [FieldOffset(20)]
        public UInt32 SpaceCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst=SpacesSharedMaxCount)]
        [FieldOffset(24)]
        public Guid[] SpaceUuids;
    }
#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE

    public struct SpaceDiscoveryCompleteData
    {
        public UInt64 RequestId;
        public int Result;
    }

    public struct SpaceDiscoveryResultsData
    {
        public UInt64 RequestId;
    }

    public struct SpacesSaveResultData
    {
        public UInt64 RequestId;
        public OVRAnchor.SaveResult Result;
    }

    public struct SpacesEraseResultData
    {
        public UInt64 RequestId;
        public OVRAnchor.EraseResult Result;
    }

#endif

    public struct PassthroughLayerResumedData
    {
        public int LayerId;
    }

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE // XR_META_return_to_room
    public struct ReturnToRoomUserLocationChanged
    {
        public Guid RoomUuid;
        public ReturnToRoomUserLocation UserLocation;
    }
#endif

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE // XR_META_boundary_visibility
    public struct BoundaryVisibilityChangedData
    {
        public BoundaryVisibility BoundaryVisibility;
    }
#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE // XR_META_boundary_visibility

#if OVR_INTERNAL_CODE // TODO - remove redaction before public release (T175611441)
    public struct CreateDynamicObjectTrackerResultsData
    {
        public OVRPlugin.Result Result;
    }

    public struct SetDynamicObjectTrackedClassesResultsData
    {
        public OVRPlugin.Result Result;
    }
#endif // OVR_INTERNAL_CODE
}
