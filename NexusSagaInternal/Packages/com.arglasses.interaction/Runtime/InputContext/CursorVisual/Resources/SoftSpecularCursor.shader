Shader "Unlit/SoftSpecularCursor"
{
    Properties
    {
        _MainTex("Main Tex", 2D) = "white" {}
        [PerRendererData]_StencilComp("Stencil Comparison", Float) = 8
        [PerRendererData]_Stencil("Stencil ID", Float) = 0
        [PerRendererData]_StencilOp("Stencil Operation", Float) = 0
        [PerRendererData]_StencilWriteMask("Stencil Write Mask", Float) = 255
        [PerRendererData]_StencilReadMask("Stencil Read Mask", Float) = 255
        [PerRendererData]_ColorMask("Color Mask", Float) = 15
    }
    SubShader
    {
        Tags
        {
            "Queue"="Transparent"
            "IgnoreProjector"="True"
            "RenderType"="Transparent"
            "PreviewType"="Plane"
            "CanUseSpriteAtlas"="True"
        }
        Stencil
        {
            Ref [_Stencil]
            Comp [_StencilComp]
            Pass [_StencilOp]
            ReadMask [_StencilReadMask]
            WriteMask [_StencilWriteMask]
        }

        Cull Off
        Lighting Off
        ZWrite Off
        ZTest [unity_GUIZTestMode]
        Blend SrcAlpha One
        ColorMask [_ColorMask]

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            
            #pragma multi_compile_local _ UNITY_UI_CLIP_RECT
            #pragma multi_compile_local _ UNITY_UI_ALPHACLIP

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = TRANSFORM_TEX(v.uv, _MainTex);
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                float value = 1 - smoothstep(0, 0.5, length(float2(0.5, 0.5) - i.uv.xy));
                float inCircle = value;
                fixed4 color = fixed4(1,1,1,1) * inCircle;
                color.a = 0.25;

                #ifdef UNITY_UI_CLIP_RECT
                    // color.a *= UnityGet2DClipping(IN.worldPosition.xy, _ClipRect);
                #endif

                #ifdef UNITY_UI_ALPHACLIP
                    clip (color.a - 0.001);
                #endif

                return color;
            }
            ENDCG
        }
    }
}
