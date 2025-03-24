// https://developer.download.nvidia.com/cg/index_stdlib.html
// https://theorangeduck.com/page/avoiding-shader-conditionals
/*
float4 when_eq(float4 x, float4 y) {
  return 1.0 - abs(sign(x - y));
}

float4 when_neq(float4 x, float4 y) {
  return abs(sign(x - y));
}

float4 when_gt(float4 x, float4 y) {
  return max(sign(x - y), 0.0);
}

float4 when_lt(float4 x, float4 y) {
  return max(sign(y - x), 0.0);
}

float4 when_ge(float4 x, float4 y) {
  return 1.0 - when_lt(x, y);
}

float4 when_le(float4 x, float4 y) {
  return 1.0 - when_gt(x, y);
}

float4 and(float4 a, float4 b) {
    return a * b;
}

float4 or(float4 a, float4 b) {
    return min(a + b, 1.0);
}

float4 xor(float4 a, float4 b) {
    return (a + b) % 2.0;
}

float4 not(float4 a) {
    return 1.0 - a;
}
*/

struct appdata
{
    float4 vertex : POSITION;
    float2 uv : TEXCOORD0;
    UNITY_VERTEX_INPUT_INSTANCE_ID
};

struct v2f
{
    float2 uv : TEXCOORD0;
    float4 vertex : SV_POSITION;
    float4 posWorld : TEXCOORD2;
    UNITY_VERTEX_INPUT_INSTANCE_ID
    UNITY_VERTEX_OUTPUT_STEREO
};

float Remap(float value, float inputFrom, float inputTo, float outputFrom, float outputTo)
{
    return (value - inputFrom) / (inputTo - inputFrom) * (outputTo - outputFrom) + outputFrom;
}

float Remap01(float value, float inputFrom, float inputTo)
{
    return Remap(value, inputFrom, inputTo, 0, 1);
}

float DegToRad(float deg)
{
    return radians(deg); // deg * (3.1415926 / 180.0);
}

float RadToDeg(float rad)
{
    return degrees(rad); //rad * (180.0 / 3.1415926);
}

float2 HeadPixelToDegreePos(float3 headToPixel)
{
    float acosX = acos(dot(float3(0, 0, 1), normalize(float3(headToPixel.x, 0, headToPixel.z))));
    float acosY = acos(dot(float3(0, 0, 1), normalize(float3(0, headToPixel.y, headToPixel.z))));
    return float2(RadToDeg(acosX), RadToDeg(acosY));
}

float CircleShape(float radius, float2 pos)
{
    float value = distance(pos, float2(radius, radius));
    return step(radius, value);
}

float CircleGrid(float2 degreePos, float circleRadius, float centered)
{
    float2 fmodDeg = fmod(degreePos + float2(circleRadius, circleRadius) * centered, circleRadius * 2);
    return 1- CircleShape(circleRadius, fmodDeg);
}

float DrawRingBorder(float3 headToPixel, float3 cameraSpaceTarget, float radiusDeg, float borderWidth)
{
    float3 headToTarget = normalize(cameraSpaceTarget);
    float rad = acos(dot(headToPixel, headToTarget));
    float radExpandedByRadius = cos(rad - DegToRad(radiusDeg));
    // return saturate(Remap01(radExpandedByRadius, 0.999999, 1));
    return saturate(sign(radExpandedByRadius - 1 + borderWidth * 0.000005));
}

float DrawRingFilled(float3 headToPixel, float3 cameraSpaceTarget, float radiusDeg)
{
    float3 headToTarget = normalize(cameraSpaceTarget);
    float radExpandedByRadius = 1 - acos(dot(headToPixel, headToTarget)) + DegToRad(radiusDeg);
    // return saturate(Remap01(ringDot, 0.999999, 1));
    return saturate(sign(radExpandedByRadius - 1));
}

float AxisOverlay(float3 headToPixel, float width)
{
    float minAxis = min(abs(headToPixel.x), abs(headToPixel.y));
    return saturate(1 - minAxis* 600 );
}

float AngleLines(float headRad, float increment, float lineSize)
{
    float angleLines = cos(headRad * (360 / increment));
    return saturate(Remap01(angleLines, 1 - lineSize, 1));
}

float AngleOverlay(float headRad, float3 increments, float3 lineWidths)
{
    float overlay = 0;
    overlay += AngleLines(headRad, increments[0], lineWidths[0]);
    overlay += AngleLines(headRad, increments[1], lineWidths[1]);
    overlay += AngleLines(headRad, increments[2], lineWidths[2]);
    return saturate(overlay);
}

float DrawRing(float4 col, float3 headToPixel, float3 cameraSpaceTarget, float radiusDeg,
               float borderWidth, float4 fillColor, float4 borderColor)
{
    col = lerp(col, fillColor, DrawRingFilled(headToPixel, cameraSpaceTarget, radiusDeg));
    col = lerp(col, borderColor, DrawRingBorder(headToPixel, cameraSpaceTarget, radiusDeg, borderWidth));
    return col;
}

float FovFill(float3 headToPixel, float2 fov)
{
    float acosX = saturate(acos(dot(float3(0, 0, 1), normalize(float3(headToPixel.x, 0, headToPixel.z)))));
    float xRad = sign(acosX - DegToRad(fov.x * 0.5));

    float acosY = saturate(acos(dot(float3(0, 0, 1), normalize(float3(0, headToPixel.y, headToPixel.z)))));
    float yRad = sign(acosY - DegToRad(fov.y * 0.5));

    float2 headToPixel2 = headToPixel;
    float absTotal = abs(headToPixel.x) + abs(headToPixel.y);
    float boundary = 0.65;

    if(dot(headToPixel, float3(0, 0, 1)) > 0.94) return 0; // hacking out dead pixels along the axis :(
    if(absTotal > boundary) return 1;

    return saturate(1 + xRad + yRad);
}

float FovFill_bu(float3 headToPixel, float2 fov)
{
    float xRad = acos(dot(float3(0, 0, 1), normalize(fixed3(headToPixel.x, 0, headToPixel.z))));
    float yRad = acos(dot(float3(0, 0, 1), normalize(fixed3(0, headToPixel.y, headToPixel.z))));
    int xFov = sign(xRad - DegToRad(fov.x * 0.5));
    int yFov = sign(yRad - DegToRad(fov.y * 0.5));
    return saturate(1 + xFov + yFov);
}
