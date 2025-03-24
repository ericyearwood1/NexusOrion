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

#if OVR_INTERNAL_CODE // BB_INTERFACE

#if UNITY_SERVICES_RELAY_MODULE_DEFINE && UNITY_SERVICES_LOBBY_MODULE_DEFINE && UNITY_NGO_MODULE_DEFINE
#define UNITY_SERVICES_INSTALLED
#endif

#if UNITY_SERVICES_INSTALLED
using System.Collections;
using Unity.Services.Authentication;
using Unity.Services.Core;
using Unity.Services.Lobbies.Models;
using Unity.Services.Lobbies;
using Unity.Services.Relay;
using Unity.Netcode;
using Unity.Netcode.Transports.UTP;
#endif

using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

namespace Meta.XR.BuildingBlocks
{
    public class AutoMatchmakingNGO : MonoBehaviour
    {
        public int maxPlayers = 4;
        public string lobbyName = "lobbyName";
        private const string JoinCodeKey = "joinCode";

#pragma warning disable CS1998 // Async method lacks 'await' operators and will run synchronously
        private async void Awake()
#pragma warning restore CS1998 // Async method lacks 'await' operators and will run synchronously
        {
#if UNITY_SERVICES_INSTALLED
            await UnityServices.InitializeAsync();
            await AuthenticationService.Instance.SignInAnonymouslyAsync();

            _connectedLobby = await CreateOrJoinLobby();
            if (IsLobbyHost(_connectedLobby))
            {
                // Send a heartbeat every 15 seconds to keep the room alive
                StartCoroutine(HeartbeatLobbyCoroutine(_connectedLobby.Id, 15));
            }

#else
            throw new InvalidOperationException(
                "It's required to install the Unity Game Services packages to use this component.");
#endif
        }

#if UNITY_SERVICES_INSTALLED
        private Lobby _connectedLobby;
        private static bool IsLobbyHost(Lobby lobby) => lobby.HostId == AuthenticationService.Instance.PlayerId;

        private async Task<Lobby> CreateOrJoinLobby()
        {
            try
            {
                return await JoinLobby();
            }
            catch (LobbyServiceException)
            {
                // No lobbies available, create a new one instead
                return await CreateLobby();
            }
        }

        private static async Task<Lobby> JoinLobby()
        {
            var lobby = await Lobbies.Instance.QuickJoinLobbyAsync();
            var joinAllocation = await RelayService.Instance.JoinAllocationAsync(joinCode: lobby.Data[JoinCodeKey].Value);

            FindObjectOfType<UnityTransport>().SetClientRelayData(joinAllocation.RelayServer.IpV4, (ushort)joinAllocation.RelayServer.Port,
                joinAllocation.AllocationIdBytes, joinAllocation.Key, joinAllocation.ConnectionData, joinAllocation.HostConnectionData);

            NetworkManager.Singleton.StartClient();
            return lobby;
        }

        private async Task<Lobby> CreateLobby()
        {
            var allocation = await RelayService.Instance.CreateAllocationAsync(maxPlayers);
            var joinCode = await RelayService.Instance.GetJoinCodeAsync(allocation.AllocationId);

            var lobby = await Lobbies.Instance.CreateLobbyAsync(lobbyName, maxPlayers, new CreateLobbyOptions
            {
                Data = new Dictionary<string, DataObject> { { JoinCodeKey, new DataObject(DataObject.VisibilityOptions.Public, joinCode) } },
                IsPrivate = false
            });

            FindObjectOfType<UnityTransport>().SetHostRelayData(allocation.RelayServer.IpV4, (ushort)allocation.RelayServer.Port, allocation.AllocationIdBytes, allocation.Key, allocation.ConnectionData);
            NetworkManager.Singleton.StartHost();
            return lobby;
        }

        private IEnumerator HeartbeatLobbyCoroutine(string lobbyId, float waitTimeSeconds) {
            var delay = new WaitForSecondsRealtime(waitTimeSeconds);
            while (_connectedLobby != null) {
                Lobbies.Instance.SendHeartbeatPingAsync(lobbyId);
                yield return delay;
            }
        }

        private void OnDestroy() {
            try {
                if (_connectedLobby == null)
                {
                    return;
                }

                LeaveLobby();
            }
            catch (Exception e) {
                Debug.Log($"Error shutting down lobby: {e}");
            }
        }

        private void LeaveLobby()
        {
            if (IsLobbyHost(_connectedLobby))
            {
                Lobbies.Instance.DeleteLobbyAsync(_connectedLobby.Id);
            }
            else
            {
                Lobbies.Instance.RemovePlayerAsync(_connectedLobby.Id, AuthenticationService.Instance.PlayerId);
            }

            _connectedLobby = null;
        }
#endif
    }
}

#endif // OVR_INTERNAL_CODE
