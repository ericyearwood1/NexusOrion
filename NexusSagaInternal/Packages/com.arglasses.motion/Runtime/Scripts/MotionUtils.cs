using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Object = UnityEngine.Object;

namespace ARGlasses.Interaction.Motion
{
    /// <summary>
    /// Static utilities, including extension methods, meant to provide
    /// convenience when generating motion. You won't usually interact
    /// with this class directly.
    /// </summary>
    public static class MotionUtils
    {
        #region Runner instance management

        // Since RandomRangeInt is not allowed to be called from a MonoBehaviour instance field initializer
        public static readonly System.Random _random = new();
        public static System.Random Random => _random;
        public static int RandomRange(int min, int max) => _random.Next(min, max);

        private static MotionRunner _runner;
        public static MotionRunner Runner
        {
            get
            {
                _runner = EnsureRunner();
                return _runner;
            }

            set
            {
                if (_runner.GetInstanceID() != value.GetInstanceID())
                {
                    _runner = value;
                }
            }
        }
        public static MotionRunner EnsureRunner()
        {
            MotionRunner r = _runner;
            if (r == null)
            {
                r = Object.FindObjectOfType<MotionRunner>();
                if (r == null)
                {
                    GameObject g = new GameObject();
                    g.name = "[ARIM] Runner";
                    r = g.AddComponent<MotionRunner>();
                }
            }
            return r;
        }

        #endregion

        #region Transform utilities

        public static Motion<Vector3> MoveOneShot(this Transform transformToMove, Vector3 goalPosition, AMotionParams motionParams = null)
        {
            return Runner.AddPositionMotion(transformToMove, goalPosition, motionParams);
        }
        
        public static Motion<Vector3> MoveLocalOneShot(this Transform transformToMove, Vector3 goalPosition, AMotionParams motionParams = null)
        {
            motionParams ??= AMotionParams.Default;
            return Runner.AddPositionMotion(transformToMove, goalPosition, motionParams, true);
        }

        public static Motion<Quaternion> RotateOneShot(this Transform transformToMove, Quaternion goalRotation, AMotionParams motionParams = null)
        {
            motionParams ??= AMotionParams.Default;
            return Runner.AddRotationMotion(transformToMove, goalRotation, motionParams);
        }
        
        public static Motion<Quaternion> RotateLocalOneShot(this Transform transformToMove, Quaternion goalRotation, AMotionParams motionParams = null)
        {
            motionParams ??= AMotionParams.Default;
            return Runner.AddRotationMotion(transformToMove, goalRotation, motionParams, true);
        }

        public static Motion<Vector3> ScaleOneShot(this Transform transformToScale, Vector3 goalScale, AMotionParams motionParams = null)
        {
            return Runner.AddScaleMotion(transformToScale, goalScale, motionParams);
        }

        public static Motion<float> FadeOneShot(this CanvasGroup canvasGroupToFade, float goalAlpha, AMotionParams motionParams = null)
        {
            return Runner.AddFadeMotion(canvasGroupToFade, goalAlpha, motionParams);
        }

        public static MotionHistory<Vector3> PositionHistory(this Transform transformToTrack)
        {
            return new MotionHistory<Vector3>(()=> transformToTrack.position);
        }

        public static MotionHistory<Quaternion> RotationHistory(this Transform transformToTrack)
        {
            return new MotionHistory<Quaternion>(()=> transformToTrack.rotation);
        }

        #endregion

        #region Generic utilities

        public static Motion Begin(this Motion motion)
        {
            return Runner.AddMotion(motion);
        }

        public static Motion Begin<T>(this Motion<T> motion, T current) where T : unmanaged
        {
            return Runner.AddMotion(motion, current);
        }

        /// <summary>
        /// One-Shot generic Motion function. Pass any value to the Motion system and watch its value move towards a target using MotionParams.
        /// </summary>
        /// <param name="start">The value to adjust.</param>
        /// <param name="step">The Action used each Step, typically used to assign the updated value.</param>
        /// <param name="goal">The goal value for the Motion.</param>
        /// <param name="motionParams">Describes the rules for motion.  Will use AMotionParams.Default if not specified.</param>
        /// <param name="owner">Used to clean up and resolve multiple subsequent OneShot calls on the same object.</param>
        /// <returns></returns>
        public static Motion<T> OneShot<T>(this T start, Action<T> step, T goal, AMotionParams motionParams = null, Component owner = null) where T : unmanaged
        {
            return Runner.AddMotion(start, step, goal, motionParams, owner);
        }

        // todo @aaronfaucher I wonder if it is best that we nudge the user to always provide an owner?
        public static Motion<T> OneShot<T>(this Component owner, T start, T goal, Action<T> step, AMotionParams motionParams = null) where T : unmanaged
        {
            return Runner.AddMotion(start, step, goal, motionParams, owner);
        }

        public static MotionHistory InitializeHistory<T>(MotionHistory<T> motionHistory)
        {
            return Runner.AddHistory(motionHistory);
        }

        public static Func<Vector4, T> ConvertVectorToType<T>(Type type)
        {
            if (type == typeof(float))
            {
                return (vector) => (T)(object) vector.x;
            }

            else if (type == typeof(Vector2))
            {
                return (vector) => (T)(object)(Vector2) vector;
            }

            else if (type == typeof(Vector3))
            {
                return (vector) => (T)(object)(Vector3) vector;
            }

            else if (type == typeof(Vector4))
            {
                return (vector) => (T)(object) vector;
            }

            else if (type == typeof(Quaternion))
            {
                return (vector) => (T)(object) new Quaternion(vector.x, vector.y, vector.z, vector.w);
            }

            else if (type == typeof(Color))
            {
                return (vector) => (T)(object) new Color(vector.x, vector.y, vector.z, vector.w);
            }

            else
            {
                // Debug.Log($"Vector cannot be cast to Type {type}");
                return (vector) => (T)(object) vector;
            }
        }

        public static Func<T, Vector4> ConvertTypeToVector<T>(Type type)
        {
            if (type == typeof(float))
            {
                return (t) => new Vector4((float)(object)t, 0f, 0f, 0f);
            }

            else if (type == typeof(Vector2))
            {
                return (t) => (Vector2)(object) t;
            }

            else if (type == typeof(Vector3))
            {
                return (t) => (Vector3)(object) t;
            }

            else if (type == typeof(Vector4))
            {
                return (t) => (Vector4)(object) t;
            }

            else if (type == typeof(Quaternion))
            {
                return (t) =>
                {
                    Quaternion q = (Quaternion) (object) t;
                    return new Vector4(q.x, q.y, q.z, q.w);
                };
            }

            else if (type == typeof(Color))
            {
                return (t) =>
                {
                    Color c = (Color) (object) t;
                    return new Vector4(c.r, c.g, c.b, c.a);
                };
            }

            else
            {
                // Debug.Log($"Type {type} cannot be cast to Vector4");
                return (t) => Vector4.zero;
            }
        }
        
        public static bool IsVectorOrScalar(Type type)
        {
            bool isVectorOrScalar = false;

            isVectorOrScalar = type == typeof(float) ||
                               type == typeof(Vector2) ||
                               type == typeof(Vector3) ||
                               type == typeof(Vector4) ||
                               type == typeof(Quaternion) ||
                               type == typeof(Color);

            return isVectorOrScalar;
        }
        
        private static object Lerp(object a, object b, float d)
        {
            Type type = a.GetType();
            if (type == typeof(float))
                return (float)a + d * ((float)b - (float)a);
            else if (type == typeof(int))
                return (int)((float)(int)a + d * ((float)(int)b - (float)(int)a));
            else if (type == typeof(Vector2))
                return Vector2.LerpUnclamped((Vector2)a, (Vector2)b, d);
            else if (type == typeof(Vector3))
                return Vector3.LerpUnclamped((Vector3)a, (Vector3)b, d);
            else if (type == typeof(Vector4))
                return Vector4.LerpUnclamped((Vector4)a, (Vector4)b, d);
            else if (type == typeof(Quaternion))
                return Quaternion.Lerp((Quaternion)a, (Quaternion)b, d);
            else if (type == typeof(Color))
            {
                return Color.LerpUnclamped((Color)a, (Color)b, d);
            }
            else
                throw new NotSupportedException("Type " + type + " is not supported");
        }

        private static float InverseLerp(object a, object b, object value)
        {
            Type type = value.GetType();
            if (type == typeof(float))
                return ((float)value - (float)a) / ((float)b - (float)a);
            else if (type == typeof(int))
                return ((float)value - (float)a) / ((float)b - (float)a);
            else if (type == typeof(Vector2))
                return Vector2.Dot((Vector2)value - (Vector2)a, (Vector2)b - (Vector2)a) / Vector2.Dot((Vector2)b - (Vector2)a, (Vector2)b - (Vector2)a);
            else if (type == typeof(Vector3))
                return Vector3.Dot((Vector3)value - (Vector3)a, (Vector3)b - (Vector3)a) / Vector3.Dot((Vector3)b - (Vector3)a, (Vector3)b - (Vector3)a);
            else if (type == typeof(Vector4))
                return Vector4.Dot((Vector4)value - (Vector4)a, (Vector4)b - (Vector4)a) / Vector4.Dot((Vector4)b - (Vector4)a, (Vector4)b - (Vector4)a);
            else if (type == typeof(Color))
            {
                Color cV = (Color) value;
                Color cA = (Color) a;
                Color cB = (Color) b;
                return Vector4.Dot(new Vector4(cV.r, cV.g, cV.b, cV.a) - new Vector4(cA.r, cA.g, cA.b, cA.a), new Vector4(cB.r, cB.g, cB.b, cB.a) - new Vector4(cA.r, cA.g, cA.b, cA.a)) / Vector4.Dot(new Vector4(cB.r, cB.g, cB.b, cB.a) - new Vector4(cA.r, cA.g, cA.b, cA.a), new Vector4(cB.r, cB.g, cB.b, cB.a) - new Vector4(cA.r, cA.g, cA.b, cA.a));;
            }
            else
                throw new NotSupportedException("Type " + type + " is not supported");
        }
        
        #endregion

        #region Lifecycle

        public static Motion Stop(this Motion motion)
        {
            return _runner != null ? _runner.Stop(motion) : null;
        }

        public static void ResetRunner()
        {
            if (_runner != null)
            {
                Object.DestroyImmediate(_runner.gameObject);
                _runner = null;
            }
        }

        #endregion

    }
}
