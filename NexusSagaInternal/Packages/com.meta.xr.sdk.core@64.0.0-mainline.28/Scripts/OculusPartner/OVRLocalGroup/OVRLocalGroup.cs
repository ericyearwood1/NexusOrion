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

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE

using System;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;

/// <summary>
/// Represents a local group.
/// </summary>
/// <remarks>
/// Local Groups are uniquely identified with their <see cref="GroupUuid"/>.
/// </remarks>
[HelpURL("https://developer.oculus.com/reference/unity/latest/class_o_v_r_local_group")]
public class OVRLocalGroup
{
    [OVRResultStatus]
    public enum ResultStatus
    {
        Success = OVRPlugin.Result.Success,
        Failure = OVRPlugin.Result.Failure,
        FailureDataIsInvalid = OVRPlugin.Result.Failure_DataIsInvalid,
        NetworkTimeout = OVRPlugin.Result.Failure_SpaceNetworkTimeout,
        NetworkRequestFailed = OVRPlugin.Result.Failure_SpaceNetworkRequestFailed,
        InvalidGroup = OVRPlugin.Result.Failure_LocalGroupInvalidGroup,
        InsufficientPermissions = OVRPlugin.Result.Failure_LocalGroupInsufficientPermissions
    }

    public Guid GroupUuid { get; internal set; }

    public String GroupAppInfo { get; internal set; }

    public OVRLocalGroupParticipant SelfParticipant { get; internal set; }

    internal OVRPlugin.LocalGroupJoinPolicy JoinPolicy { get; set; }

    internal List<OVRLocalGroupParticipant> ParticipantList { get; }

    internal String selfAppInfo { get; set; }

    public event Action<OVRLocalGroupParticipant> LocalGroupParticipantJoined;
    public event Action<UInt64> LocalGroupParticipantLeft;
    public event Action<String> LocalGroupAppInfoUpdated;
    public event Action<UInt64, String> LocalGroupParticipantAppInfoUpdated;
    public event Action<UInt32, Guid[]> LocalGroupSpatialAnchorsShared;

    internal void AddParticipant(OVRLocalGroupParticipant participant)
    {
        if (FindParticipant(participant.ParticipantId) == null)
        {
            ParticipantList.Add(participant);
        }
    }

    private void RemoveParticipant(OVRLocalGroupParticipant participant)
    {
        ParticipantList.Remove(participant);
    }

    private OVRLocalGroupParticipant FindParticipant(UInt64 participantId)
    {
        foreach (var participant in ParticipantList)
        {
            if (participant.ParticipantId == participantId)
            {
                return participant;
            }
        }

        return null;
    }

    internal OVRLocalGroup(String groupAppInfo,
        OVRPlugin.LocalGroupJoinPolicy joinPolicy = OVRPlugin.LocalGroupJoinPolicy.Public)
    {
        this.GroupAppInfo = groupAppInfo;
        this.JoinPolicy = joinPolicy;
        this.ParticipantList = new List<OVRLocalGroupParticipant>();
        OVRManager.LocalGroupJoinComplete += OnJoinSessionComplete;
        OVRManager.LocalGroupParticipantsQueryResultsReady += OnLocalGroupParticipantsQueryResultsReady;
        OVRManager.LocalGroupLeaveComplete += OnLeaveLocalGroupComplete;
        OVRManager.LocalGroupSetAppInfoComplete += OnLocalGroupSetAppInfoComplete;
        OVRManager.LocalGroupParticipantSetAppInfoComplete += OnLocalGroupParticipantSetAppInfoComplete;
        OVRManager.LocalGroupShareSpacesComplete += OnLocalGroupShareSpatialAnchorsComplete;
    }

    ~OVRLocalGroup()
    {
        OVRManager.LocalGroupJoinComplete -= OnJoinSessionComplete;
        OVRManager.LocalGroupParticipantsQueryResultsReady -= OnLocalGroupParticipantsQueryResultsReady;
        OVRManager.LocalGroupLeaveComplete -= OnLeaveLocalGroupComplete;
        OVRManager.LocalGroupSetAppInfoComplete -= OnLocalGroupSetAppInfoComplete;
        OVRManager.LocalGroupParticipantSetAppInfoComplete -= OnLocalGroupParticipantSetAppInfoComplete;
        OVRManager.LocalGroupShareSpacesComplete -= OnLocalGroupShareSpatialAnchorsComplete;
    }

    /// <summary>
    /// Join a local group.
    /// </summary>
    /// <param name="joinerParticipantAppInfo"> The app info the joiner. </param>
    /// <returns> An <see cref="OVRLocalGroupResult"/> that has wrapped a
    /// OVRLocalGroupParticipant<see cref="OVRLocalGroupParticipant"/> if successful, otherwise error.
    /// </returns>
    public OVRTask<OVRResult<OVRLocalGroupParticipant, ResultStatus>> JoinAsync(String joinerParticipantAppInfo)
    {
        OVRPlugin.LocalGroupJoinInfo joinLocalGroupInfo = new OVRPlugin.LocalGroupJoinInfo();
        joinLocalGroupInfo.JoinerParticipantAppInfo = joinerParticipantAppInfo;
        joinLocalGroupInfo.GroupUuid = this.GroupUuid;
        selfAppInfo = joinerParticipantAppInfo;
        OVRPlugin.Result result = OVRPlugin.JoinLocalGroup(joinLocalGroupInfo, out var requestId);
        if (result == OVRPlugin.Result.Success)
        {
          return OVRTask.FromRequest<OVRResult<OVRLocalGroupParticipant, ResultStatus>>(requestId);
        }
        else
        {
           return OVRTask.FromResult<OVRResult<OVRLocalGroupParticipant, ResultStatus>>(OVRResult<OVRLocalGroupParticipant, ResultStatus>.From(default, (ResultStatus)result));
        }
    }

    /// <summary>
    /// Leave a local group.
    /// </summary>
    /// <returns> An <see cref="OVRLocalGroupResult"/> indicating success or fail.
    /// </returns>
    public OVRTask<OVRResult<ResultStatus>> LeaveAsync()
    {
        //leave the session, remove self from group list.
        OVRPlugin.LocalGroupLeaveInfo leaveLocalGroupInfo = new OVRPlugin.LocalGroupLeaveInfo();
        leaveLocalGroupInfo.GroupUuid = this.GroupUuid;
        OVRPlugin.Result result = OVRPlugin.LeaveLocalGroup(leaveLocalGroupInfo, out var requestId);
        if (result == OVRPlugin.Result.Success)
        {
          return OVRTask.FromRequest<OVRResult<ResultStatus>>(requestId);
        }
        else
        {
           return OVRTask.FromResult<OVRResult<ResultStatus>>(OVRResult<ResultStatus>.From((ResultStatus)result));
        }
    }


    /// <summary>
    /// Queries the participants of a local group.
    /// </summary>
    /// <returns>An array of <see cref="OVRLocalGroupParticipant"/> for the group.
    /// </returns>
    public OVRTask<OVRLocalGroupParticipant[]> GetParticipantsAsync()
    {
        OVRPlugin.LocalGroupParticipantsQueryInfo queryInfo = new OVRPlugin.LocalGroupParticipantsQueryInfo();
        queryInfo.GroupUuid = this.GroupUuid;
        return (OVRPlugin.QueryLocalGroupParticipants(queryInfo, out var requestId))
            ? OVRTask.FromRequest<OVRLocalGroupParticipant[]>(requestId)
            : OVRTask.FromResult<OVRLocalGroupParticipant[]>(null);
    }

    /// <summary>
    /// Sets App Info of the Group.
    /// </summary>
    /// <param name="groupAppInfo"> The group app info.</param>
    /// <returns> An <see cref="OVRLocalGroupResult"/> indicating success or fail.
    /// </returns>
    public OVRTask<OVRResult<ResultStatus>> SetAppInfoAsync(String groupAppInfo)
    {
        OVRPlugin.LocalGroupAppInfoSetInfo localGroupAppInfoSetInfo = new OVRPlugin.LocalGroupAppInfoSetInfo();
        localGroupAppInfoSetInfo.GroupUuid = this.GroupUuid;
        localGroupAppInfoSetInfo.GroupAppInfo = groupAppInfo;
        OVRPlugin.Result result = OVRPlugin.SetLocalGroupAppInfo(localGroupAppInfoSetInfo, out var requestId);
        if (result == OVRPlugin.Result.Success)
        {
          return OVRTask.FromRequest<OVRResult<ResultStatus>>(requestId);
        }
        else
        {
           return OVRTask.FromResult<OVRResult<ResultStatus>>(OVRResult<ResultStatus>.From((ResultStatus)result));
        }
    }

    /// <summary>
    /// Sets Participant (self) App Info of the Group.
    /// </summary>
    /// <param name="participantAppInfo"> The participant app info.</param>
    /// <returns> An <see cref="OVRLocalGroupResult"/> indicating success or fail.
    /// </returns>
    public OVRTask<OVRResult<ResultStatus>> SetParticipantAppInfoAsync(String participantAppInfo)
    {
        OVRPlugin.LocalGroupParticipantAppInfoSetInfo localGroupParticipantAppInfoSetInfo =
            new OVRPlugin.LocalGroupParticipantAppInfoSetInfo();
        localGroupParticipantAppInfoSetInfo.ParticipantAppInfo = participantAppInfo;
        localGroupParticipantAppInfoSetInfo.GroupUuid = GroupUuid;
        SelfParticipant.ParticipantAppInfo = participantAppInfo;
        OVRPlugin.Result result = OVRPlugin.SetLocalGroupParticipantAppInfo(localGroupParticipantAppInfoSetInfo, out var requestId);
        if (result == OVRPlugin.Result.Success)
        {
          return OVRTask.FromRequest<OVRResult<ResultStatus>>(requestId);
        }
        else
        {
           return OVRTask.FromResult<OVRResult<ResultStatus>>(OVRResult<ResultStatus>.From((ResultStatus)result));
        }
    }

    /// <summary>
    /// Shares spatial anchors in local group.
    /// </summary>
    /// <params name="anchors"> Collection of spatial anchors to be shared within the group.</param>
    /// <returns> An <see cref="OVRLocalGroupResult"/> indicating success or fail.
    /// </returns>
    public OVRTask<OVRResult<ResultStatus>> LocalGroupShareSpatialAnchorsAsync(ICollection<OVRSpatialAnchor> anchors)
    {
        OVRPlugin.LocalGroupShareSpacesInfo localGroupShareSpacesInfo =
            new OVRPlugin.LocalGroupShareSpacesInfo();
        localGroupShareSpacesInfo.SpaceCount = (uint) anchors.Count;
        localGroupShareSpacesInfo.GroupUuid = GroupUuid;
        unsafe {
            localGroupShareSpacesInfo.Spaces = (IntPtr)Unity.Collections.LowLevel.Unsafe.NativeArrayUnsafeUtility.GetUnsafePtr(ToNativeArray(anchors)); // void* -> IntPtr explicit conversion.
        }

        OVRPlugin.Result result =  OVRPlugin.LocalGroupShareSpaces(ref localGroupShareSpacesInfo, out var requestId);
        if (result == OVRPlugin.Result.Success) {
            return OVRTask.FromRequest<OVRResult<ResultStatus>>(requestId);
        }
        else {
            return OVRTask.FromResult<OVRResult<ResultStatus>>(OVRResult<ResultStatus>.From((ResultStatus)result));
        }
    }

    private void OnLocalGroupParticipantsQueryResultsReady(ulong requestId, bool result, uint participantsCount)
    {
        OVRPlugin.LocalGroupParticipantsGetQueryResultsInfo getQueryInfo =
            new OVRPlugin.LocalGroupParticipantsGetQueryResultsInfo();
        getQueryInfo.QueryRequestId = requestId;
        getQueryInfo.ParticipantsCount = participantsCount;
        OVRPlugin.LocalGroupParticipantInfo[] participants = new OVRPlugin.LocalGroupParticipantInfo[participantsCount];
        OVRPlugin.GetLocalGroupParticipantsQueryResults(getQueryInfo, out participants);

        OVRLocalGroupParticipant[] participantList = new OVRLocalGroupParticipant[participantsCount];
        for (int i = 0; i < participantsCount; i++)
        {
            OVRLocalGroupParticipant participant = new OVRLocalGroupParticipant();
            participant.ParticipantId = participants[i].ParticipantId;
            participant.ParticipantAppInfo = participants[i].ParticipantAppInfo;
            participantList[i] = participant;
        }

        var task = OVRTask.GetExisting<OVRLocalGroupParticipant[]>(requestId);
        task.SetResult(participantList);
    }

    private void OnJoinSessionComplete(UInt64 requestId, UInt64 joinerParticipantId, int result, String groupAppInfo)
    {
        //add self to the group.
        var task = OVRTask.GetExisting<OVRResult<OVRLocalGroupParticipant, ResultStatus>>(requestId);
        if (result == (int) OVRPlugin.Result.Success)
        {
            OVRLocalGroupParticipant localGroupParticipant = new OVRLocalGroupParticipant();
            localGroupParticipant.ParticipantId = joinerParticipantId;
            localGroupParticipant.ParticipantAppInfo = selfAppInfo;
            this.SelfParticipant = localGroupParticipant;
            AddParticipant(localGroupParticipant);
            task.SetResult(OVRResult<OVRLocalGroupParticipant, ResultStatus>.From(localGroupParticipant, (ResultStatus)result));
        } else {
            OVRPlugin.Result ovrResult = (OVRPlugin.Result) result;
            task.SetResult(OVRResult<OVRLocalGroupParticipant, ResultStatus>.From(default, (ResultStatus)result));
        }
    }

    private void OnLeaveLocalGroupComplete(UInt64 requestId, int result)
    {
        var task = OVRTask.GetExisting<OVRResult<ResultStatus>>(requestId);
        RemoveParticipant(SelfParticipant);
        task.SetResult(OVRResult<ResultStatus>.From((ResultStatus)result));
    }

    private void OnLocalGroupSetAppInfoComplete(UInt64 requestId, int result)
    {
        var task = OVRTask.GetExisting<OVRResult<ResultStatus>>(requestId);
        task.SetResult(OVRResult<ResultStatus>.From((ResultStatus)result));
    }

    private void OnLocalGroupParticipantSetAppInfoComplete(UInt64 requestId, int result)
    {
        var task = OVRTask.GetExisting<OVRResult<ResultStatus>>(requestId);
        task.SetResult(OVRResult<ResultStatus>.From((ResultStatus)result));
    }

    private void OnLocalGroupShareSpatialAnchorsComplete(UInt64 requestId, OVRPlugin.LocalGroupShareSpacesResult result)
    {
        var task = OVRTask.GetExisting<OVRResult<ResultStatus>>(requestId);
        task.SetResult(OVRResult<ResultStatus>.From((ResultStatus)result));
    }

    internal void OnParticipantJoined(UInt64 participantId, string participantAppInfo)
    {
        var participant = new OVRLocalGroupParticipant();
        participant.ParticipantId = participantId;
        participant.ParticipantAppInfo = participantAppInfo;
        if (LocalGroupParticipantJoined != null)
        {
            LocalGroupParticipantJoined(participant);
        }

        AddParticipant(participant);
    }

    internal void OnParticipantLeft(UInt64 participantId)
    {
        if (LocalGroupParticipantLeft != null)
        {
            LocalGroupParticipantLeft(participantId);
        }

        var participant = FindParticipant(participantId);
        if (participant == null)
        {
            return;
        }

        RemoveParticipant(participant);
    }

    internal void OnGroupAppInfoUpdated(String appInfo)
    {
        if (LocalGroupAppInfoUpdated != null)
        {
            LocalGroupAppInfoUpdated(appInfo);
        }

        GroupAppInfo = appInfo;
    }

    internal void OnParticipantAppInfoUpdated(UInt64 participantId, String participantAppInfo)
    {
        if (LocalGroupParticipantAppInfoUpdated != null)
        {
            LocalGroupParticipantAppInfoUpdated(participantId, participantAppInfo);
        }

        var participant = FindParticipant(participantId);
        if (participant == null)
        {
            return;
        }

        participant.ParticipantAppInfo = participantAppInfo;
    }

    internal void OnLocalGroupSpatialAnchorsShared(UInt32 spaceCount, Guid[] localSpacesShared)
    {
        if (LocalGroupSpatialAnchorsShared != null)
        {
            LocalGroupSpatialAnchorsShared(spaceCount, localSpacesShared);
        }
    }

    private static NativeArray<ulong> ToNativeArray(ICollection<OVRSpatialAnchor> anchors)
    {
        var count = anchors.Count;
        var spaces = new NativeArray<ulong>(count, Allocator.Temp);
        var i = 0;
        foreach (var anchor in anchors)
        {
           spaces[i++] = anchor ? anchor._anchor.Handle : 0;
        }
        return spaces;
    }
}
#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE
