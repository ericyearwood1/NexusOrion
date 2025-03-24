Shader "Meta/Depth/Builtin Rendering Pipeline/DynamicOcclusionLit"
{
    Properties
    {
        _Color ("Color", Color) = (1,1,1,1)
        _MainTex ("Albedo (RGB)", 2D) = "white" {}
        _Glossiness ("Smoothness", Range(0,1)) = 0.5
        _Metallic ("Metallic", Range(0,1)) = 0.0
        _EnvironmentDepthBias ("Environment Depth Bias", Float) = 0.0
    }
    SubShader
    {
        Tags { "LightMode" = "ForwardBase" }
        LOD 200

        CGPROGRAM

        #pragma target 3.5

        #include "EnvironmentOcclusionBiRP.cginc"

        #pragma surface surf Standard fullforwardshadows vertex:vert

        // DepthAPI Environment Occlusion
        #pragma multi_compile _ HARD_OCCLUSION SOFT_OCCLUSION

        sampler2D _MainTex;
        half _Glossiness;
        half _Metallic;
        fixed4 _Color;
        float _EnvironmentDepthBias;

        struct Input {
            float2 uv_MainTex : TEXCOORD0;
            float4 vertex : SV_POSITION;

            META_DEPTH_VERTEX_OUTPUT(1)

            UNITY_VERTEX_INPUT_INSTANCE_ID
            UNITY_VERTEX_OUTPUT_STEREO
        };

        void vert (inout appdata_full v, out Input o) {
            float4 _MainTex_ST;
            UNITY_SETUP_INSTANCE_ID(v);
            UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o); // required to support stereo
            o.vertex = UnityObjectToClipPos(v.vertex);
            o.uv_MainTex = TRANSFORM_TEX(v.texcoord, _MainTex);
            META_DEPTH_INITIALIZE_VERTEX_OUTPUT(o, v.vertex);
        }

        void surf (Input IN, inout SurfaceOutputStandard o) {
            UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(IN);
            fixed4 c = tex2D (_MainTex, IN.uv_MainTex) * _Color;
            META_DEPTH_OCCLUDE_OUTPUT_PREMULTIPLY(IN, c, _EnvironmentDepthBias);

            o.Albedo = c.rgb;
            o.Metallic = _Metallic;
            o.Smoothness = _Glossiness;
            o.Alpha = c.a;
        }
        ENDCG
    }
}
