Shader "Unlit/RoundedRectAdditive"
{
    Properties
    {
        _MainTex("Main Tex", 2D) = "white" {}
        _SpecularColor("Specular Color", Color) = (1,1,1,1)
        _SpecularStrength("Specular Strength", Float) = 0.5
        _SpecularRadius("Specular Radius", Float) = 0.5
        _CursorPosition("Cursor Position", Vector) = (0,0,0)
        _HideOnBlack("Hide On Black", Float) = 0.5
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
        Blend SrcAlpha OneMinusSrcAlpha, One OneMinusSrcAlpha
        ColorMask [_ColorMask]

        GrabPass
        {

        }

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #pragma multi_compile_local __ UNITY_UI_ALPHACLIP
            #pragma multi_compile PROTOKIT_UI_GAMMA_CORRECTED_ALPHA


            #include "UnityCG.cginc"
            

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0; // Regular UV
                float4 uv2 : TEXCOORD2; // X: Outline Corner Radius, Y : Corner Radius, ZW : Corner Center Coordinates 
                float4 color : COLOR; // vertexColor
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                float2 uv : TEXCOORD0;
                float4 uv2 : TEXCOORD2;
                float4 color : COLOR;
                float4 grabPos : TEXCOORD1;

                #ifdef ITK_CLIPPING_ON
                ITK_CLIP_COORDS(1) // TEXCOORD1
                #endif

                UNITY_VERTEX_INPUT_INSTANCE_ID
                UNITY_VERTEX_OUTPUT_STEREO
            };

            v2f vert(appdata v)
            {
                v2f o;
                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_TRANSFER_INSTANCE_ID(v, o);
                UNITY_INITIALIZE_OUTPUT(v2f, o);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);


                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                o.uv2 = v.uv2;
                o.color = v.color;
                o.grabPos = ComputeGrabScreenPos(o.vertex);

                return o;
            }

            sampler2D _MainTex;
            sampler2D _GrabTexture;
            fixed4 _SpecularColor;
            fixed _SpecularStrength;
            fixed _SpecularRadius;
            fixed3 _CursorPosition;
            fixed _HideOnBlack;

            fixed4 frag(v2f i) : SV_Target
            {
                UNITY_SETUP_INSTANCE_ID(i);
                

                //We generate negative uvs for edge triangles
                bool isEdge = i.uv.x <= -100;
                bool isGlowEdge = i.uv.x <= -200;

                // edge radius = minSDF
                float edgeRadius = i.uv2.x;
                //radius  = maxSDF
                float radius = i.uv2.y;
                //float opacity = i.uv3.x;

                //Calculate signed distance to the curvature box
                float d2Box = length(max(i.uv2.zw, 0.0)) + min(max(i.uv2.z, i.uv2.w), 0.0);

                //Calculate the anti-aliasing width based on current pixel-size of this
                //fragment.  Can adjust 0.5 constant for greater or lesser edge smoothing
                float aaWidth = 0.5 * max(abs(ddx(i.uv2.z)), abs(ddy(i.uv2.z)));

                //If we are drawing the panel mesh, and if there is a border, we extrude
                //the panel mesh out by a small fraction so that it makes a tight seam
                //with the border.  Otherwise there is a 0.5 pixel gap due to the fuzzy 
                //edges
                if (!isEdge && edgeRadius != radius)
                {
                    edgeRadius += aaWidth;
                }

                //Calculate anti-aliased masks 
                float outerMask = smoothstep(radius + aaWidth, radius - aaWidth, d2Box);
                float innerMask = smoothstep(edgeRadius + aaWidth, edgeRadius - aaWidth, d2Box);
                

                //Adjustment so that non-curved edge pixels are always 100% opaque
                //Allows panels that are flush against each other to always be seamless
                if (min(i.uv2.z, i.uv2.w) < 0)
                {
                    outerMask = 1;
                }

                float4 finalColor;
                if (isEdge)
                {
                    finalColor = i.color;
                    if(isGlowEdge)
                    {
                        finalColor.a *= 1-smoothstep(edgeRadius,radius , d2Box);

                    }else
                    {
                        finalColor.a *= outerMask * (1 - innerMask);
                    }
                }
                else
                {
                    finalColor = tex2D(_MainTex, i.uv) * i.color;
                    finalColor.a *= innerMask;

                }

                // apply glow
                float dist = length(_CursorPosition.xy - i.uv.xy);
                float value = 1 - smoothstep(0, _SpecularRadius, dist);
                
                fixed4 grabColor = tex2Dproj(_GrabTexture, i.grabPos);
                if(finalColor.a > 0.001 && (grabColor.r + grabColor.g + grabColor.b) > 0.001)
                {
                    finalColor.a = finalColor.a + value * _SpecularStrength;
                }

                // if HideOnBlack ~1, do not render if the bg is black
                // if ~0, do not render if the bg is NOT black
                if(_HideOnBlack < 0.25)
                {
                    fixed4 grabColor = tex2Dproj(_GrabTexture, i.grabPos);
                    if((grabColor.r + grabColor.g + grabColor.b) > 0.001)
                        clip(-1);
                }
                else if(_HideOnBlack > 0.75)
                {
                    fixed4 grabColor = tex2Dproj(_GrabTexture, i.grabPos);
                    clip((grabColor.r + grabColor.g + grabColor.b) - 0.001);
                }

                // clipping
                #ifdef UNITY_UI_CLIP_RECT
                    finalColor.a *= UnityGet2DClipping(IN.worldPosition.xy, _ClipRect);
                #endif
                
                #ifdef PROTOKIT_UI_GAMMA_CORRECTED_ALPHA
                    finalColor.a = GammaToLinearSpace(finalColor.aaa).x;
                #endif

                #ifdef UNITY_UI_ALPHACLIP
                clip(finalColor.a - 0.001);
                #endif
                
                
                return finalColor;
            }
            ENDCG
        }
    }
}