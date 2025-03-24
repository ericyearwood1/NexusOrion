Shader "Unlit/RimLit" {

    Properties {
        _Color1 ("Color", Color) = (1,1,1,1)

        _Radius ("Radius", Range(0.0, 1.0)) = 0.5

        _ScaleX ("Scale X", Float) = 1.0
        _ScaleY ("Scale Y", Float) = 1.0

        _Offset ("Offset", Range(0.0, 1.0)) = 0.0

        _Stroke ("Stroke", Range(0.0,.1)) = .02
        _StrokeAlpha ("Stroke Alpha", Range(0.0,1.0)) = .5

        _Reveal ("Reveal", Float) = 0.0
        _RevealIntensity ("RevealIntensity", Range(0.0,1.0)) = 0.1
        _RevealX ("Reveal X", Range(-10.0, 10.0)) = 0.0
        _RevealY ("Reveal Y", Range(-10.0, 10.0)) = 0.0
        _RevealRadius ("Reveal Radius", Range(0.0,10.0)) = 1
    }

    SubShader {

        Tags {
            "IgnoreProjector"="True"
            "Queue"="Transparent"
            "RenderType"="Transparent"
            }
        LOD 200

        Pass {

            Name "FORWARD"
            Blend SrcAlpha OneMinusSrcAlpha
            Cull Off
            ZWrite Off

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            UNITY_INSTANCING_BUFFER_START(Props)
                UNITY_DEFINE_INSTANCED_PROP(float4, _Color1)

                UNITY_DEFINE_INSTANCED_PROP(float, _Radius)
                UNITY_DEFINE_INSTANCED_PROP(float, _ScaleX)
                UNITY_DEFINE_INSTANCED_PROP(float, _ScaleY)

                UNITY_DEFINE_INSTANCED_PROP(float, _Offset)

                UNITY_DEFINE_INSTANCED_PROP(float, _Stroke)
                UNITY_DEFINE_INSTANCED_PROP(float, _StrokeAlpha)

                UNITY_DEFINE_INSTANCED_PROP(float, _Reveal)
                UNITY_DEFINE_INSTANCED_PROP(float, _RevealIntensity)

                UNITY_DEFINE_INSTANCED_PROP(float, _RevealX)
                UNITY_DEFINE_INSTANCED_PROP(float, _RevealY)
                UNITY_DEFINE_INSTANCED_PROP(float, _RevealRadius)

            UNITY_INSTANCING_BUFFER_END(Props)

            float roundedRect (float2 center, float2 halfSize, float radius) {

                // Corners
                float circleTopRight = step(radius, distance(center, halfSize));
                float circleBottomLeft = step(radius, distance(center,-1. * halfSize));
                float circleTopLeft = step(radius, distance(center, float2(halfSize.x * -1., halfSize.y)));
                float circleBottomRight = step(radius, distance(center, float2(halfSize.x, halfSize.y * -1.)));

                float corners = circleTopRight * circleBottomLeft * circleTopLeft * circleBottomRight;

                // Vertical fill
                // bottom limit
                float2 vb = step(float2(-1. * halfSize.x, -1. * halfSize.y - radius), center);
                // top limit
                float2 vt = step(center, float2( halfSize.x,  halfSize.y + radius));

                // Create vertical fill
                float vert = vb.x * vb.y;
                vert *= vt.x * vt.y;
                vert = 1. - vert;

                // Horizontal fill
                // bottom limit
                float2 hb = step(float2(-1. * halfSize.x - radius, -1. * halfSize.y), center);
                // top limit
                float2 ht= step(center, float2( halfSize.x + radius,  halfSize.y));

                // create horizontal fill
                float horiz = hb.x * hb.y;
                horiz *= ht.x * ht.y;
                horiz = 1. - horiz;

                // Creat Rect
                float infill = horiz * vert;
                float roundedRect =  corners * infill;

                return roundedRect;
            }


            struct vertexInput {
                float4 vertex : POSITION;
                float4 texcoord0 : TEXCOORD0;
            };

            struct fragmentInput{
                float4 position : SV_POSITION;
                float4 texcoord0 : TEXCOORD0;
            };

            fragmentInput vert(vertexInput i){
                fragmentInput o;
                o.position = UnityObjectToClipPos (i.vertex);
                o.texcoord0 = i.texcoord0;
                return o;
            }
            fixed4 frag(fragmentInput i) : SV_Target {

                 // retrive variables
                float4 inputColor = UNITY_ACCESS_INSTANCED_PROP(Props, _Color1);
                float radius = UNITY_ACCESS_INSTANCED_PROP(Props, _Radius);
                float scaleX = UNITY_ACCESS_INSTANCED_PROP(Props, _ScaleX);
                float scaleY = UNITY_ACCESS_INSTANCED_PROP(Props, _ScaleY);
                float offset = UNITY_ACCESS_INSTANCED_PROP(Props, _Offset);
                float stroke = UNITY_ACCESS_INSTANCED_PROP(Props, _Stroke);
                float strokeAlpha = UNITY_ACCESS_INSTANCED_PROP(Props, _StrokeAlpha);
                float reveal = UNITY_ACCESS_INSTANCED_PROP(Props, _Reveal);
                float revealIntensity = UNITY_ACCESS_INSTANCED_PROP(Props, _RevealIntensity);
                float revealX = UNITY_ACCESS_INSTANCED_PROP(Props, _RevealX);
                float revealY = UNITY_ACCESS_INSTANCED_PROP(Props, _RevealY);
                float revealRadius = UNITY_ACCESS_INSTANCED_PROP(Props, _RevealRadius);

                // create color base
                float3 rgb = inputColor.xyz;

                float pct;

                // get coords
                float2 st = i.texcoord0.xy;

                // Remap to center from -1 to 1
                float size = 2.0;
                float2 scale = float2(scaleX, scaleY);

                // set each side to scale independently
                st.x = (st.x * size * scale.x) - (scale.x);
                st.y = (st.y * size * scale.y) - (scale.y);



                // Rename
                float inset = offset;

                // coords for coners
                float2 coords = float2(1., 1.);

                // Place coords based on scale and radius
                coords.x *= scale.x - radius - inset;
                coords.y *= scale.y - radius - inset;

                // Create full face
                float face = roundedRect(st, coords, radius);

                // Create outline for face
                float outlineOuter = roundedRect(st, coords - stroke, radius * (1.-stroke));

                // Remove the middle of stroke
                float outline = outlineOuter - face;

                // Create glow effect
                float glow = 0.0;

                float2 revealPosition = float2(revealX, revealY);

                // If glow, apply it
                if (reveal != 0) {
                    glow = 1.0 - smoothstep(.0,revealRadius,distance(st,revealPosition));
                }

                // Inverse face
                pct = (1. - (face)) + (outline * strokeAlpha);

                // Apply drawing to the color fields
                rgb *= pct+1.;

                // Apply global color alpha
                float alpha = pct * inputColor.a;

                // Add glow
                alpha *= (glow * revealIntensity + inputColor.a);

                // Add outline, add glow to outline
                alpha += (outlineOuter * pct * (glow * revealIntensity));
                alpha += (outlineOuter * pct * (glow * revealIntensity));

                float4 color = float4(rgb, alpha);

                return color;
            }

            ENDCG
        }
    }
}
