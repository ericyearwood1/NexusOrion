Shader "OCUI/OC Icon" {
  Properties {
    _ColorA("Color", Color) = (1,1,1,1)
    _MainTex("Texture", 2D) = "white" {}

    _StencilComp("Stencil Comparison", Float) = 8
    _Stencil("Stencil ID", Float) = 0
    _StencilOp("Stencil Operation", Float) = 0
    _StencilWriteMask("Stencil Write Mask", Float) = 255
    _StencilReadMask("Stencil Read Mask", Float) = 255
    _ColorMask("Color Mask", Float) = 15
  }

  SubShader {
    Tags { 
      "RenderType" = "Transparent"
      "Queue" = "Transparent"
    }
    Stencil {
      Ref[_Stencil]
      Comp[_StencilComp]
      Pass[_StencilOp]
      ReadMask[_StencilReadMask]
      WriteMask[_StencilWriteMask]
    }

    ZWrite Off
    Cull Off
    Blend SrcAlpha OneMinusSrcAlpha, One OneMinusSrcAlpha
    LOD 100
    ColorMask[_ColorMask]

    Pass {
      Name "MAIN"
      CGPROGRAM
      #pragma vertex vert
      #pragma fragment frag
      #pragma multi_compile_local __ UNITY_UI_ALPHACLIP
      #include "UnityCG.cginc"

      struct appdata {
        float4 vertex : POSITION;
        float2 uv : TEXCOORD0;
        float4 color : COLOR;
        UNITY_VERTEX_INPUT_INSTANCE_ID
      };

      struct v2f {
        float2 uv : TEXCOORD0;
        float4 vertex : SV_POSITION;
        float4 color : COLOR;

        UNITY_VERTEX_INPUT_INSTANCE_ID
        UNITY_VERTEX_OUTPUT_STEREO
      };

      sampler2D _MainTex;
      float4 _MainTex_ST;
      uniform float4 _ColorA;

      v2f vert(appdata v) {
        v2f o;

        UNITY_SETUP_INSTANCE_ID(v);
        UNITY_TRANSFER_INSTANCE_ID(v, o);
        UNITY_INITIALIZE_OUTPUT(v2f, o);
        UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);

        o.vertex = UnityObjectToClipPos(v.vertex);
        o.uv = TRANSFORM_TEX(v.uv, _MainTex);
        o.color = v.color;
        return o;
      }

      fixed4 frag(v2f i) : SV_Target {
        float4 final = tex2D(_MainTex, i.uv);
        final = _ColorA * final * i.color.a;

        // enabled for masked panels
        #ifdef UNITY_UI_ALPHACLIP
        clip(final.a - 0.001);
        #endif

        return final;
      }
      ENDCG
    }
  }
}
