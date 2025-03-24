Shader "ARIM/VisualDegreeOverlay"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        _LineIncrements("Line Increments", vector) = (20.0, 5.0, 1.0, 0)
        _LineWidths("Line Widths Radius", vector) = (0.003, 0.025, 0.1, 0)
        _LineColor("Line Color", Color) = (0, 1, 1, 0.015)

        _Fov("FOV", vector) = (60.0, 40.0, 0, 0)
        _FovColor("FOV Color", Color) = (0, 0, 0, 1)

        _FillColor0("Fill Color 0", Color) = (1, 0, 0, 0.04)
        _BorderColor0("Border Color 0", Color) = (1, 0, 0, 0.04)
        _Radius0("Radius 0 (degrees)", Float) = 4.5
        _BorderWidth0("Border Width 0", Float) = 0.2
        _Position0("Position 0 (camera space)", vector) = (-0.1, 0, 1, 0)

        _FillColor1("Fill Color 1", Color) = (0, 1, 0, 0.04)
        _BorderColor1("Border Color 1", Color) = (0, 1, 0, 0.04)
        _Radius1("Radius 1 (degrees)", Float) = 1.5
        _BorderWidth1("Border Width 1", Float) = 0.2
        _Position1("Position 1 (camera space)", vector) = (0, 0, 1, 0)

        _FillColor2("Fill Color 2", Color) = (1, 1, 1, 0.5)
        _BorderColor2("Border Color 2", Color) = (1, 1, 1, 0.5)
        _Radius2("Radius 2 (degrees)", Float) = 0.15
        _BorderWidth2("Border Width 2", Float) = 0
        _Position2("Position 2 (camera space)", vector) = (0.1, 0, 1, 0)
    }

    SubShader
    {
        Tags
        {
            "Queue"="Overlay" "RenderType"="Opaque"
        }

        Blend SrcAlpha OneMinusSrcAlpha, One One
        ZTest Always
        ZWrite Off

        Cull Front

        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #pragma multi_compile_local __ UNITY_UI_ALPHACLIP

            #include "UnityCG.cginc"
            #include "VisualDegreeLib.cginc"

            sampler2D _MainTex;
            float4 _MainTex_ST;

            float4 _LineIncrements;
            float4 _LineWidths;
            float4 _LineColor;

            float4 _Fov;
            float4 _FovColor;

            float4 _FillColor0;
            float4 _BorderColor0;
            float _Radius0;
            float _BorderWidth0;
            float4 _Position0;

            float4 _FillColor1;
            float4 _BorderColor1;
            float _Radius1;
            float _BorderWidth1;
            float4 _Position1;

            float4 _FillColor2;
            float4 _BorderColor2;
            float _Radius2;
            float _BorderWidth2;
            float4 _Position2;

            v2f vert(appdata v)
            {
                v2f o;
                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_TRANSFER_INSTANCE_ID(v, o);
                UNITY_INITIALIZE_OUTPUT(v2f, o);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = TRANSFORM_TEX(v.uv, _MainTex);
                o.posWorld = v.vertex;
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                UNITY_SETUP_INSTANCE_ID(i);

                float3 headToPixel = normalize(i.posWorld.xyz);
                const float headRad = max(DegToRad(0.5), acos(dot(float3(0, 0, 1), headToPixel)));
                float2 degreePos = HeadPixelToDegreePos(headToPixel);

                fixed4 col = fixed4(0, 0, 0, 0);
                col = lerp(col, _LineColor, AxisOverlay(headToPixel, _LineWidths[1]));
                col = lerp(col, _LineColor, AngleOverlay(headRad, _LineIncrements, _LineWidths));

                col = lerp(col, _FillColor0,  DrawRingFilled(headToPixel, _Position0, _Radius0) * 0.5);
                col = lerp(col, _BorderColor0, DrawRingBorder(headToPixel, _Position0, _Radius0, _BorderWidth0));

                col = lerp(col, _FillColor1, DrawRingFilled(headToPixel, _Position1, _Radius1)* 0.5);
                col = lerp(col, _BorderColor1, DrawRingBorder(headToPixel, _Position1, _Radius1, _BorderWidth1));

                col = lerp(col, _FillColor2, DrawRingFilled(headToPixel, _Position2, _Radius2)* 0.5);
                col = lerp(col, _BorderColor2, DrawRingBorder(headToPixel, _Position2, _Radius2, _BorderWidth2));

                // col = lerp(col, float4(1, 0, 0, .013), CircleGrid(degreePos, 10*.5, 0));
                col = lerp(col, _FovColor, FovFill(headToPixel, _Fov));

                return col;
            }
            ENDCG
        }
    }
}
