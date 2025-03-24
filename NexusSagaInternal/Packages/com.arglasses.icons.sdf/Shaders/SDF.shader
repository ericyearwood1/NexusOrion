Shader "GDPX/SDF"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        _Color("Color", Color) = (1,1,1,1)
        _Cutoff ("Cutoff", Float) = 0.5
        _Softness ("Softness", Float) = 1
    }
    SubShader
    {
        Tags { "RenderType"="Transparent" "Queue"="Transparent"}
        LOD 100
        Blend SrcAlpha OneMinusSrcAlpha
        ZWrite Off
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag


            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
                fixed4 color : COLOR;
            };

            struct v2f
            {
                float4 uv : TEXCOORD0;
                half4 color : TEXCOORD1;
                float4 vertex : SV_POSITION;
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
            half4 _Color;
            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = float4(v.uv,0,-0.5);
                o.color = v.color * _Color;
                return o;
            }
            half _Cutoff;
            half _Softness;

            float filterwidth(float2 v) {   float2 fw = max(abs(ddx(v)), abs(ddy(v)));   return max(fw.x, fw.y); } 
            fixed4 frag (v2f i) : SV_Target
            {
                half4 color = i.color;
                half dist = (_Cutoff - tex2Dbias(_MainTex, i.uv).a);
                float fw = filterwidth(i.uv);

                // sdf distance per pixel (gradient vector)
                // float2 ddist = 4*float2(ddx(dist), ddy(dist));

                // // distance to edge in pixels (scalar)
                // float pixelDist = dist / length(ddist);
                half colorThreshold = min(2*fw*_Softness, 0.499);
                float alpha = smoothstep(colorThreshold, -colorThreshold, dist);
                
                 color.a *= alpha;
                return color;
            }
            ENDCG
        }
    }
}
