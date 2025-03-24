using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mirror;
using Mirror.Discovery;
using Mirror.BouncyCastle.Asn1.Crmf;

public class OrionNetworkingAssistant : MonoBehaviour
{
    public bool isHost;

    public enum CurrentPlatform
    {
        Phone,
        Quest,
        Orion
    }

    public CurrentPlatform platform;

    public NetworkDiscoveryHUD discoveryHUD;
    public GameObject ovrCameraRig;
    public GameObject mainCam;
    public GameObject orionCameraRig;

    public GameObject localPlayer;

    public bool upBool;
    public bool downBool;

    public float moveSpeed;

    public GameObject[] debugConsole;


    public void Awake()
    {
        if (platform == CurrentPlatform.Phone)
        {
            mainCam.SetActive(true);
            ovrCameraRig.SetActive(false);
            orionCameraRig.SetActive(false);
            debugConsole[0].SetActive(true);
        }
        else if (platform == CurrentPlatform.Quest)
        {
            mainCam.SetActive(false);
            ovrCameraRig.SetActive(true);
            orionCameraRig.SetActive(false);
            debugConsole[1].SetActive(true);
        }
        else
        {
            mainCam.SetActive(false);
            ovrCameraRig.SetActive(false);
            orionCameraRig.SetActive(true);
            debugConsole[1].SetActive(true);
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        if (isHost) 
        {
            //mainCam.SetActive(false);
            //ovrCameraRig.SetActive(true);
            Invoke("StartAsHost", 2f);
        }
        else
        {
            //mainCam.SetActive(true);
            //ovrCameraRig.SetActive(false);
            discoveryHUD.flaggedAsClient = true;
            discoveryHUD.networkDiscovery.StartDiscovery();
        }

        
    }

    // Update is called once per frame
    void Update()
    {
        if (upBool)
        {
            if (localPlayer)
            {
                localPlayer.transform.Translate(Vector3.up * moveSpeed);
            }
        }
        if (downBool) 
        {
            if (localPlayer)
            {
                localPlayer.transform.Translate(Vector3.down * moveSpeed);
            }
        }
    }

    public void MoveUp(bool buttonPressed)
    {
        if (buttonPressed)
        {
            upBool = true;
        }
        else
        {
            upBool = false;
        }
    }

    public void MoveDown(bool buttonPressed) 
    {
        if (buttonPressed)
        {
            downBool = true;
        }
        else
        {
            downBool = false;
        }
    }


    public void StartAsHost()
    {
        
        NetworkManager.singleton.StartHost();
        discoveryHUD.networkDiscovery.AdvertiseServer();
    }
}
