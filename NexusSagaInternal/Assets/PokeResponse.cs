using UnityEngine;
using Oculus.Interaction;
using Mirror;

public class PokeResponse : MonoBehaviour
{
    private Renderer _renderer;
    public OrionNetworkingAssistant orionNetworkingAssistant;

    void Awake()
    {
        _renderer = GetComponent<Renderer>();
        orionNetworkingAssistant = GameObject.Find("GameManager").GetComponent<OrionNetworkingAssistant>();
    }

    public void OnPoke()
    {
        // Change the color of the object when poked
        _renderer.material.color = Random.ColorHSV();
        if (orionNetworkingAssistant.localPlayer)
        {
            orionNetworkingAssistant.localPlayer.GetComponent<NetworkedInteractor>().CmdPerformAction();
        }
        else
        {
            Debug.Log("No localPlayer found");
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        OnPoke();
    }
}