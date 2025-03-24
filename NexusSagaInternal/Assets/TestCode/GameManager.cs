using Microsoft.CognitiveServices.Speech;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameManager : MonoBehaviour
{
    public GameObject sphereIndicator;
    public GameObject cubeIndicator;
    public GameObject pinchIndicator;
    public GameObject colorChangeCube;

    public bool sphereBool;
    public bool cubeBool;
    public bool pinchBool;

    private void Update()
    {
        if (sphereBool == true)
        {
            sphereIndicator.SetActive(true);
        }
        else
        {
            sphereIndicator.SetActive(false);
        }

        if (cubeBool == true)
        {
            cubeIndicator.SetActive(true);
        }
        else
        {
            cubeIndicator.SetActive(false);
        }

        if (pinchBool == true)
        {
            pinchIndicator.SetActive(true);
        }
        else
        {
            pinchIndicator.SetActive(false);
        }
    }

    public void ChangeCubeColor()
    {
        Renderer colorRenderer = colorChangeCube.GetComponent<Renderer>();
        colorRenderer.material.color = Random.ColorHSV();
    }


}
