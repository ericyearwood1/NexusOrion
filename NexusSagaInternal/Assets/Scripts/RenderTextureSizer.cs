using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RenderTextureSizer : MonoBehaviour
{
    public Camera headlockedL, headlockedR;
    public Camera surface;
    public Camera objectLockedL, objectLockedR;

    public OVROverlay headlockedOverlay;
    public OVROverlay surfaceOverlay;
    public OVROverlay objectLockedOverlay;

    public int headlockedSize, surfaceSize, objectLockedSize;

    public void UpdateHeadlocked()
    {
        headlockedL.targetTexture = new RenderTexture(headlockedSize, headlockedSize, 24);
        headlockedR.targetTexture = new RenderTexture(headlockedSize, headlockedSize, 24);
        headlockedOverlay.textures[0] = headlockedL.targetTexture;
        headlockedOverlay.textures[1] = headlockedR.targetTexture;
    }

    public void UpdateSurface()
    {
        surface.targetTexture = new RenderTexture(surfaceSize, surfaceSize, 24);
        surfaceOverlay.textures[0] = surface.targetTexture;
    }

    public void UpdateObjectLocked()
    {
        objectLockedL.targetTexture = new RenderTexture(objectLockedSize, objectLockedSize, 24);
        objectLockedR.targetTexture = new RenderTexture(objectLockedSize, objectLockedSize, 24);
        objectLockedOverlay.textures[0] = objectLockedL.targetTexture;
        objectLockedOverlay.textures[1] = objectLockedR.targetTexture;
    }
}
