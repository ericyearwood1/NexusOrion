# UXF Motion
> Jump to [feature backlog](https://fburl.com/arim/motion/todo).

This library is a joint effort within the UX Foundations (formerly AR Interaction Model) team to increase the accessibility of second-order animations for everyday prototyping. It provides convenient accessors and an editor visualization for the otherwise abstract concepts related to second-order motion.

We recommend watching these two talks for context on second-order systems in UI animation:
- [Giving Personality to Procedural Animations using Math](https://www.youtube.com/watch?v=KPoeNZZ6H4s&t=256s) provides a primer to the fundamental concepts of second-order dynamics and how these can be applied in game engines like Unity.
- [WWDC 2018: Designing Fluid Interfaces](https://developer.apple.com/videos/play/wwdc2018/803/) explains how these concepts can be deployed in UI animation to create more responsive interfaces that respond to user intent.

## Installing the package
> Note: The Examples scene is in a separate package here [com.arglasses.motion.examples](https://ghe.oculus-rep.com/Orion-Design/OrionInterfaces?path=/packages/com.arglasses.motion.examples) to limit UI dependencies.

This package will be published to ProtoKit shortly as `[ARGlasses] Motion`. Until then, you can add the package manually.

**Add via Unity Package Manager** ([Instructions](https://docs.unity3d.com/Manual/upm-ui-giturl.html))
- Window > Package Manager
- Add (+) > Add Package from git URL...
- `https://ghe.oculus-rep.com/Orion-Design/OrionInterfaces.git?path=/packages/com.arglasses.motion`
- Optional: Add examples package `https://ghe.oculus-rep.com/Orion-Design/OrionInterfaces.git?path=/packages/com.arglasses.motion.examples`

**Add to MyProject/Packages/manifest.json** ([Instructions](https://docs.unity3d.com/Manual/upm-manifestPrj.html))
```json
"dependencies":
{
    "com.arglasses.motion": "https://ghe.oculus-rep.com/Orion-Design/OrionInterfaces.git?path=/packages/com.arglasses.motion",
    "com.arglasses.motion.examples": "https://ghe.oculus-rep.com/Orion-Design/OrionInterfaces.git?path=/packages/com.arglasses.motion.examples"
}
```

## Usage
The library is written to allow different forms of calling, depending on the needs of the use case. Check out *Motion Examples.unity* in ([com.arglasses.motion.examples](https://ghe.oculus-rep.com/Orion-Design/OrionInterfaces/tree/develop/packages/com.arglasses.motion.examples)) package for implementations.

**For simplest usage, one-shot calls are supported.**
In these cases, the Motion system will drive the transform in the background (no per-update setting required).
```c#
void Start()
{
    MotionParams motionParams = MotionParams.Default;
    // One-shot with takeover
    transform.MoveOneShot(targetPosition, motionParams);
}
```

```c#
void Start()
{
    MotionParams motionParams = new MotionParams(1.0f, 0.5f, 0f);
    Renderer _renderer = GetComponent<Renderer>();
    // One-shot with custom setter
    _renderer.material.color.Move((c) => _renderer.material.color = c, UnityEngine.Random.ColorHSV(), motionParams);
}
```

**For more control, the calling class can manage handling of the updates.**
Passing a deltaTime to the motion instance will return an update of the provided type, which can be applied in the Update loop.
```c#
public MotionParams _motionParams = new MotionParams(1.0f, 0.5f, .1f);
private Motion<Vector3> _motion;

void Start()
{
    _motion = new Motion<Vector3>(transform.position, goalPosition, _motionParams);
}

void Update()
{
    transform.position = _motion.Step(Time.deltaTime);
}
```

**Or, it can be used like an implicit FloatModel, to manage interpolations.**
One power of second-order systems is when they are used in the abstract, for example to drive a float value that can in turn drive interpolations of other parameters.
```c#
public MotionParams _motionParams;
private Motion<float> _motion;

void Start()
{
    _motion = new Motion<float>(0f, 1f, _motionParams);
}

void Update()
{
    _renderer.material.color = Color.Lerp(position1, position2, _motion.Step(Time.deltaTime));
}
```

**Complex types are supported as well.**
Building on this logic, the ability to conveniently pass a struct or class to the Motion engine is supported, through an implicit reduction to a proxy float driver. The Motion instance will automatically create a proxy driver and manage interpolation of all fields in the class, meaning that your calling class only needs to handle the application of the output. This can be useful when interpolating between UI states where several parameters are changing.
```c#

public struct MyStruct
{
    public Color iconColor;
    public float borderRadius;
    public Vector3 localPos;
}

public MyStruct _defaultState;
public MyStruct _goalState;

public MotionParams _motionParams;
private Motion<MyStruct> _motion;

void Start()
{
    _motion = new Motion<MyStruct>(_defaultState, _goalState, motionParams);
}

void Update()
{
    MyStruct stateThisFrame = _motion.Step(time.deltaTime);
    ApplyState(stateThisFrame);
}

void ApplyState()
{
    // ...
}
```

**Backwards compatibility with traditional duration-based easing curves is also supported.**
For cases where second-order dynamics are not applicable, the Motion system also accepts traditional duration-based animation with easing via AnimationCurves. You can easily access common presets from the Easing class (for example, Easing.InOutQuart returns a Unity AnimationCurve with the specific preset).

```c#
void Start()
{
    // Define motion in terms of an easing curve and duration.
    MotionParamsEasing motionParams = MotionParamsEasing(Easing.InOutQuart, 0.8f);
    // One-shot with takeover
    transform.MoveOneShot(targetPosition, motionParams);
}
```

**History utility makes tracking derivatives easy.**
Rather than needing to implement velocty and acceleration tracking within your client class, MotionHistory class keeps track and allows you to query when convenient to your logic.

```c#
private MotionHistory<Vector3> _posHistory;

void Start()
{
    _posHistory = transform.PositionHistory();
}

void OnRelease()
{
    // Query velocity, acceleration on transform
    Vector3 velLatest = _posHistory.LatestVelocity;
    Vector3 accelThisFrame = _posHistory.LatestAcceleration;
    Vector3 accelAverage = _posHistory.AverageAcceleration;
    
    // Example: Trigger one-shot that respects user momentum
    _motion = transform.MoveOneShot(_goal.position, _motionParams).SetCurrentVelocity(_posHistory.AverageVelocity);
}

```

**Lambda Support.**
Pass lambda setters to make it simple to automatically apply return values to specific fields.

```c#
private RoundedRect _roundedRect;
private float _radiusPixels;

Start()
{
    _radiusPixels.Move((r) => _roundedRect.SetRadius(r), 64f, MotionParams.Default);
}

```

## Design
Currently, all the library does is provide convenience wrappers around the second-order dynamics logic described in [this video](https://www.youtube.com/watch?v=KPoeNZZ6H4s&t=256s). This wrapping makes it easier to invoke a second-order animation, and also improves handling of different types (float, Vector2-Vector4, Quaternion, Color, others). 

- The **MotionParams** class defines the characteristics of a motion, using the second-order principles of *frequency, duration and response*. Together, you can think of these as forming something like an easing curve. MotionParams can be created at runtime, or set as a field and tweaked with a convenient inspector utility.
- The **Motion** class takes those motion parameters and applies them to a generic type, based on a deltaTime and goal state. You can create a Motion instance yourself, or use an extension method to create it.
- The **MotionUtils** and **MotionRunner** classes are mostly invisible to you as a developer. Under the hood, they provide convenient extension methods that generate managed *Motion* classes, and inject a MonoBehavior in the scene to assist with stepping takeover motions on Update.

<img src="Docs/layer-diagram.png" width="50%">

## Principles
**Second-order everything**
These scripts take a generic approach, allowing you to use second-order dynamics to tween any parameter. Implicitly, everything is reduced to second-order scalar or vector motion, to improve fluidity.

**Layer-cake approach**
Consumers can either work with a Motion instance directly, or convenience wrappers. In either case, motion parameters can be defined and tuned in the inspector.

**Convenience is priority**
You should be able to deploy animations in the way that makes sense for your use case. The separation of concerns between MotionParams, Motion instances, and static MotionUtils are set up to allow for you to call one-shots, pass custom setters, or step through an interpolation manually.

## Backlog
This is a work in-progress that seeks to be immediately useful, but still lacks some core features. If you discover a bug or find a use case that should be incorporated, please add to the feature backlog here: https://fburl.com/arim/motion/todo
