Shader "GDPX/SDF_Effects"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        _Color("Color", Color) = (1,1,1,1)
        _Softness (" Softness", Float) = 1
        _Cutoff ("Cutoff", Float) = 0.5
        _StrokeColor("Stroke Color", Color) = (1,1,1,1)
        _StrokeSoftness ("Stroke Softness", Float) = 1
        _StrokeCutoff ("Stroke Cutoff", Float) = 0.5
        _GlowColor("Glow Color", Color) = (1,1,1,1)
        _GlowCutoff ("Glow Cutoff", Float) = 0.5
        _ShadowColor("Shadow Color", Color) = (0,0,0,1)
        _ShadowAngle ("Shadow Angle", Float) = 0.5
        _ShadowDistance ("Shadow Distance", Float) = 0.5
        _ShadowSoftness ("Shadow Softness", Float) = 1
    }
    SubShader
    {
        Tags { "RenderType"="Transparent" "Queue"="Transparent"}
        LOD 100
        Blend One OneMinusSrcAlpha
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
                half4 strokeColor : TEXCOORD2;
                half4 glowColor : TEXCOORD3;
                half4 shadowColor : TEXCOORD5;
                float4 shadowUV : TEXCOORD4;
                float4 vertex : SV_POSITION;
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
            half4 _Color;
            half4 _StrokeColor;
            half4 _GlowColor;
            half4 _ShadowColor;
            half _GlowAngle;
            half _GlowDistance;
            half _ShadowAngle;
            half _ShadowDistance;
            half _ShadowSoftness;
            half _Softness;
            half _StrokeSoftness;
            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = float4(v.uv,0,-0.5);
                half rad = 0.0174533 * _ShadowAngle;
                o.shadowUV = float4(v.uv +_ShadowDistance * 0.01 * half2( sin(rad), cos(rad)),0,-0.5);
                o.color = v.color * _Color;
                o.strokeColor = v.color * _StrokeColor;
                o.glowColor = v.color * _GlowColor;
                o.shadowColor = v.color * _ShadowColor;
                return o;
            }
            half _Cutoff;
            half _StrokeCutoff;
            half _GlowCutoff;
            
            float filterwidth(float2 v) {   float2 fw = max(abs(ddx(v)), abs(ddy(v)));   return max(fw.x, fw.y); } 
            fixed4 frag (v2f i) : SV_Target
            {
                
                half d = tex2Dbias(_MainTex, i.uv).a;
                half dist = (_Cutoff - d);
                // float2 ddist = float2(ddx(dist), ddy(dist));

                // distance to edge in pixels (scalar)
                // float pixelDist = dist / length(ddist);
                
                half glowDist = 2*max(0,d- _GlowCutoff)/(1-_GlowCutoff);
                half strokeDist = (_StrokeCutoff - d);
                
                float fw =  2 * filterwidth(i.uv);

                half colorThreshold = min(_Softness*fw, 0.499);
                float alpha = smoothstep(colorThreshold, -colorThreshold, dist);

                half strokeThreshold = min(_StrokeSoftness*fw, 0.499);
                float alphaStroke = smoothstep(strokeThreshold, -strokeThreshold, strokeDist);

                half shadow = tex2Dbias(_MainTex, i.shadowUV).a;
                half shadowDist = (_Cutoff - shadow);

                
                half shadowThreshold = min(_ShadowSoftness*fw, 0.499);
                float shadowAlpha = smoothstep(shadowThreshold, -shadowThreshold, shadowDist);

                half4 color = i.color * alphaStroke + i.strokeColor *(1-alphaStroke);
                color.a *= alpha;


                

                half4 glow = i.glowColor;
                glow.a *= smoothstep(0,1,glowDist);
                half4 result = i.shadowColor * shadowAlpha;

                // result += glow *glow.a + result * (1-glow.a);

                result = color*color.a + result * (1-color.a);
                result.rgb *= result.a;
                result.rgb  += glow.rgb*glow.a* (1-color.a);
                return result;
            }
            ENDCG
        }
    }
}
