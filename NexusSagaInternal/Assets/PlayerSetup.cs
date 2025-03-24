using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mirror;

public class PlayerSetup : MonoBehaviour
{

    public OrionNetworkingAssistant orionNetworkingAssistant;
    public NetworkIdentity networkIdentity;
    void Start()
    {
        orionNetworkingAssistant = GameObject.Find("GameManager").GetComponent<OrionNetworkingAssistant>();
        if (networkIdentity.isLocalPlayer) { orionNetworkingAssistant.localPlayer = gameObject; }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
