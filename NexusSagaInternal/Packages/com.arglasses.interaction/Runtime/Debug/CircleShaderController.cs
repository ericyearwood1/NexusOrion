using UnityEngine;

public class CircleShaderController : MonoBehaviour
{
    public Material circleMaterial;
    public Vector3 circleWorldPosition = new Vector3(0, 0, 0);
    public float angularSize = 1.0f;
    public float borderThickness = 1.0f;
    public float borderSmoothness = 0.5f;
    public Color borderColor = Color.white;

    private void Start()
    {
        UpdateShaderProperties();
    }

    void UpdateShaderProperties()
    {
        if (circleMaterial)
        {
            circleMaterial.SetVector("_WorldPosition", circleWorldPosition);
            circleMaterial.SetFloat("_AngularSize", angularSize);
            circleMaterial.SetFloat("_BorderThickness", borderThickness);
            circleMaterial.SetFloat("_BorderSmoothness", borderSmoothness);
            circleMaterial.SetColor("_Color", borderColor);
        }
    }
    
    // Example: Call this function to update properties at runtime.
    public void UpdateProperties(Vector3 worldPos, float size, float thickness, float smoothness, Color color)
    {
        circleWorldPosition = worldPos;
        angularSize = size;
        borderThickness = thickness;
        borderSmoothness = smoothness;
        borderColor = color;
        
        UpdateShaderProperties();
    }
}
