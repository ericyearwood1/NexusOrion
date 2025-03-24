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
/// Local Groups Manager allows users to create and discover Local Groups.
/// </summary>
[HelpURL("https://developer.oculus.com/reference/unity/latest/class_o_v_r_local_group_manager")]
public class OVRLocalGroupManager
{
    // create a local group method call returns a local group.
    static Dictionary<ulong, OVRLocalGroup> _localGroupMap = new Dictionary<ulong, OVRLocalGroup>();
    static Dictionary<Guid, OVRLocalGroup> _guidToLocalGroupMap = new Dictionary<Guid, OVRLocalGroup>();

    /// <summary>
    /// Occurs when the local group has been discovered.
    /// @params (OVRLocalGroup localGroup)
    /// </summary>
    public static event Action<OVRLocalGroup> LocalGroupDiscoveredResult;

    /// <summary>
    /// Occurs when the discovery stops
    /// @params (LocalGroupDiscoveryStopReason stopReason)
    /// </summary>
    public static event Action<LocalGroupDiscoveryStopReason> LocalGroupDiscoveryComplete;

    public enum LocalGroupJoinPolicy
    {
        Public = OVRPlugin.LocalGroupJoinPolicy.Public,
        FriendsOnly = OVRPlugin.LocalGroupJoinPolicy.FriendsOnly,
    }

    public enum LocalGroupDiscoveryStopReason
    {
        Timeout = OVRPlugin.LocalGroupDiscoveryStopReason.Timeout,
        Stopped = OVRPlugin.LocalGroupDiscoveryStopReason.Stopped,
        Error = OVRPlugin.LocalGroupDiscoveryStopReason.Error,
    }

    private static readonly Lazy<OVRLocalGroupManager> lazy =
        new Lazy<OVRLocalGroupManager>(() => new OVRLocalGroupManager());

    public static OVRLocalGroupManager Instance
    {
        get { return lazy.Value; }
    }

    private OVRLocalGroupManager()
    {
        OVRManager.LocalGroupStartComplete += OnLocalGroupStartComplete;
        OVRManager.LocalGroupDiscoveryComplete += OnLocalGroupDiscoveryComplete;
        OVRManager.LocalGroupDiscoveredResult += OnLocalGroupDiscoveredResult;
        OVRManager.LocalGroupParticipantJoined += OnLocalGroupParticipantJoined;
        OVRManager.LocalGroupParticipantLeft += OnLocalGroupParticipantLeft;
        OVRManager.LocalGroupAppInfoUpdated += OnLocalGroupAppInfoUpdated;
        OVRManager.LocalGroupParticipantAppInfoUpdated += OnLocalGroupParticipantAppInfoUpdated;
	    OVRManager.LocalGroupSpacesShared += OnLocalGroupSpatialAnchorsShared;
    }

    ~OVRLocalGroupManager()
    {
        OVRManager.LocalGroupStartComplete -= OnLocalGroupStartComplete;
        OVRManager.LocalGroupDiscoveryComplete -= OnLocalGroupDiscoveryComplete;
        OVRManager.LocalGroupDiscoveredResult -= OnLocalGroupDiscoveredResult;
        OVRManager.LocalGroupParticipantJoined -= OnLocalGroupParticipantJoined;
        OVRManager.LocalGroupParticipantLeft -= OnLocalGroupParticipantLeft;
        OVRManager.LocalGroupAppInfoUpdated -= OnLocalGroupAppInfoUpdated;
        OVRManager.LocalGroupParticipantAppInfoUpdated -= OnLocalGroupParticipantAppInfoUpdated;
	    OVRManager.LocalGroupSpacesShared -= OnLocalGroupSpatialAnchorsShared;
    }

    /// <summary>
    /// Asynchronous method that creates a local group.
    /// </summary>
    /// <param name="groupAppInfo"> The app info associate with local group. </param>s
    /// <param name="creatorParticipantAppInfo"> The app info of creator. </param>
    /// <param name="joinPolicy"> Determines who can join the session.</param>
    /// <returns> An <see cref="OVRLocalGroupResult"/> that has wrapped <see cref="OVRTask{OVRLocalGroup}"/>
    /// If sucessful an OVRLocalGroup instance is retured else error.</returns>
    public OVRTask<OVRResult<OVRLocalGroup, OVRLocalGroup.ResultStatus>> StartLocalGroupAsync(String groupAppInfo, String creatorParticipantAppInfo,
        LocalGroupJoinPolicy joinPolicy = LocalGroupJoinPolicy.Public)
    {
        OVRPlugin.LocalGroupStartInfo startLocalGroup = new OVRPlugin.LocalGroupStartInfo();
        OVRPlugin.LocalGroupJoinPolicy internalJoinPolicy = (OVRPlugin.LocalGroupJoinPolicy)joinPolicy;
        startLocalGroup.GroupAppInfo = groupAppInfo;
        startLocalGroup.CreatorParticipantAppInfo = creatorParticipantAppInfo;
        startLocalGroup.JoinPolicy = internalJoinPolicy;
        OVRPlugin.Result result = OVRPlugin.StartLocalGroup(startLocalGroup, out var requestId);

        if (result == (int) OVRPlugin.Result.Success)
        {
            OVRLocalGroup localGroup = new OVRLocalGroup(groupAppInfo, internalJoinPolicy);
            OVRLocalGroupParticipant localGroupParticipant = new OVRLocalGroupParticipant();
            localGroupParticipant.ParticipantAppInfo = creatorParticipantAppInfo;
            localGroup.AddParticipant(localGroupParticipant);
            _localGroupMap.Add(requestId, localGroup);
            return OVRTask.FromRequest<OVRResult<OVRLocalGroup, OVRLocalGroup.ResultStatus>>(requestId);
        }
        else
        {
           return OVRTask.FromResult<OVRResult<OVRLocalGroup, OVRLocalGroup.ResultStatus>>(OVRResult<OVRLocalGroup, OVRLocalGroup.ResultStatus>.From(default, (OVRLocalGroup.ResultStatus)result));
        }
    }

    /// <summary>
    /// Discover nearby groups.
    /// </summary>
    /// <param name="maxDiscoveryDurationSec"> Time for which to discover nearby local groups </param>
    /// <returns> An <see cref="OVRLocalGroupResult"/> that has wrapped a result which is true
    /// if the call to discover groups is successful, otherwise error.
    /// the results of discovery are delivered via <see cref="LocalGroupDiscoveredResult"> and when discovery steps a <see cref="LocalGroupDiscoveryComplete"> is send.
    /// </returns>
    public static OVRTask<OVRResult<OVRLocalGroup.ResultStatus>> StartDiscoveryLocalGroupAsync(int maxDiscoveryDurationSec)
    {
        OVRPlugin.LocalGroupDiscoveryStartInfo discoveryInfo = new OVRPlugin.LocalGroupDiscoveryStartInfo();
        discoveryInfo.MaxDiscoveryDuration = maxDiscoveryDurationSec;
        OVRPlugin.Result result = OVRPlugin.StartLocalGroupDiscovery(discoveryInfo, out var requestId);
        if (result == (int) OVRPlugin.Result.Success)
        {
            return OVRTask.FromRequest<OVRResult<OVRLocalGroup.ResultStatus>>(requestId);
        }
        else
        {
           Debug.LogError($"Local Group {result} in creating a local group");
           return OVRTask.FromResult<OVRResult<OVRLocalGroup.ResultStatus>>(OVRResult<OVRLocalGroup.ResultStatus>.From((OVRLocalGroup.ResultStatus)result));
        }
    }

    private void OnLocalGroupStartComplete(ulong requestId, ulong participantId, int result, Guid uuid)
    {
        var task = OVRTask.GetExisting<OVRResult<OVRLocalGroup, OVRLocalGroup.ResultStatus>>(requestId);
        if (result == (int) OVRPlugin.Result.Success)
        {
            OVRLocalGroup localGroup = _localGroupMap[requestId];
            OVRLocalGroupParticipant participant = localGroup.ParticipantList[0];
            participant.ParticipantId = participantId;
            localGroup.SelfParticipant = participant;
            localGroup.GroupUuid = uuid;
            if (!_guidToLocalGroupMap.ContainsKey(uuid))
            {
                _guidToLocalGroupMap.Add(uuid, localGroup);
            }
            task.SetResult(OVRResult<OVRLocalGroup, OVRLocalGroup.ResultStatus>.From(localGroup, (OVRLocalGroup.ResultStatus)result));
        }
        else
        {
            Debug.LogError($"Local Group {result}");
            task.SetResult(OVRResult<OVRLocalGroup, OVRLocalGroup.ResultStatus>.From(default, (OVRLocalGroup.ResultStatus)result));
        }
    }

    private static void OnLocalGroupDiscoveredResult(ulong requestId, Guid uuid, String groupAppInfo)
    {
        if (!_guidToLocalGroupMap.ContainsKey(uuid))
        {
            OVRLocalGroup localGroup = new OVRLocalGroup(groupAppInfo);
            localGroup.GroupUuid = uuid;
            _guidToLocalGroupMap.Add(uuid, localGroup);
        }

        if (LocalGroupDiscoveredResult != null)
        {
            LocalGroupDiscoveredResult(_guidToLocalGroupMap[uuid]);
        }
    }

    private static void OnLocalGroupDiscoveryComplete(ulong requestId, bool isSuccess,
        OVRPlugin.LocalGroupDiscoveryStopReason stopReason)
    {
        if (LocalGroupDiscoveryComplete != null)
        {
            LocalGroupDiscoveryComplete((LocalGroupDiscoveryStopReason)stopReason);
        }
    }

    private void OnLocalGroupParticipantJoined(Guid uuid, UInt64 parId, string appinfo)
    {
        if (_guidToLocalGroupMap.ContainsKey(uuid))
        {
            _guidToLocalGroupMap[uuid].OnParticipantJoined(parId, appinfo);
        }
    }

    private void OnLocalGroupParticipantLeft(Guid uuid, UInt64 parId)
    {
        if (_guidToLocalGroupMap.ContainsKey(uuid))
        {
            _guidToLocalGroupMap[uuid].OnParticipantLeft(parId);
        }
    }

    internal void OnLocalGroupAppInfoUpdated(Guid uuid, string appInfo)
    {
        if (_guidToLocalGroupMap.ContainsKey(uuid))
        {
            _guidToLocalGroupMap[uuid].OnGroupAppInfoUpdated(appInfo);
        }
    }

    internal void OnLocalGroupParticipantAppInfoUpdated(Guid uuid, UInt64 participantId, string appInfo)
    {
        if (_guidToLocalGroupMap.ContainsKey(uuid))
        {
            _guidToLocalGroupMap[uuid].OnParticipantAppInfoUpdated(participantId, appInfo);
        }
    }

   internal void OnLocalGroupSpatialAnchorsShared(Guid uuid, UInt32 spaceCount, Guid[] spaceUuids)
   {
        if (_guidToLocalGroupMap.ContainsKey(uuid))
        {
            _guidToLocalGroupMap[uuid].OnLocalGroupSpatialAnchorsShared(spaceCount, spaceUuids);
        }
   }
}
#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE
