Shader "ARIM/CircleBorder" {
    Properties {
        _WorldPosition("Circle World Position", Vector) = (0,0,0,1)
        _AngularSize("Angular Size (degrees)", Range(0, 180)) = 1.0
        _BorderThickness("Border Thickness", Range(0, 10)) = 1.0
        _BorderSmoothness("Border Smoothness", Range(0, 5)) = 0.5
        _Color("Color", Color) = (1, 1, 1, 1)
    }

    SubShader {
        Tags { "Queue" = "Transparent" "RenderType" = "Transparent" }
        Pass {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f {
                float2 uv : TEXCOORD0;
                float3 viewDir : TEXCOORD1;
                float4 vertex : SV_POSITION;
            };

            float4 _WorldPosition;
            float _AngularSize;
            float _BorderThickness;
            float _BorderSmoothness;
            float4 _Color;

            v2f vert(appdata v) {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                
                // Calculate the view direction in world space
                o.viewDir = normalize(_WorldPosition.xyz - mul(unity_ObjectToWorld, v.vertex).xyz);
                o.uv = v.uv;

                return o;
            }

            half4 frag(v2f i) : SV_Target {
                // Compute the angle between view direction and the vector pointing to the circle's world position
                float angle = degrees(acos(dot(normalize(i.viewDir), normalize(-_WorldPosition.xyz))));
                
                float halfAngularSize = _AngularSize * 0.5;
                float outerRadius = halfAngularSize;
                float innerRadius = halfAngularSize - _BorderThickness;

                // Distance from the current fragment to the inner and outer border
                float distToOuter = outerRadius - angle;
                float distToInner = angle - innerRadius;

                // Calculate the alpha based on the proximity to the border
                float alphaOuter = smoothstep(0, _BorderSmoothness, distToOuter);
                float alphaInner = 1.0 - smoothstep(0, _BorderSmoothness, distToInner);
                
                // Combine both alphas
                float combinedAlpha = min(alphaOuter, alphaInner);

                // Test if the pixel falls within the circle's border
                if (angle <= outerRadius && angle >= innerRadius) {
                    return float4(_Color.rgb, _Color.a * combinedAlpha);
                }
                else {
                    discard;
                }

                return half4(1,0,0,1); // default return, should never reach this
            }
            ENDCG
        }
    }
}
