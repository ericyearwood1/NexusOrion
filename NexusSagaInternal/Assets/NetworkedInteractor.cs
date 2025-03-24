using Mirror;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Mirror
{
    public class NetworkedInteractor : NetworkBehaviour
    {
        public GameManager gameManager;

        private void Start()
        {
            gameManager = GameObject.Find("GameManager").GetComponent<GameManager>();
        }

        [Command]
        public void CmdPerformAction()
        {
            RpcNotifyClients();
        }

        [ClientRpc]
        void RpcNotifyClients()
        {
            gameManager.ChangeCubeColor();
        }
    }
}

