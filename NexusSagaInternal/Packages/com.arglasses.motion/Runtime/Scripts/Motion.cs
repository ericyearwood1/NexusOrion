using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Linq.Expressions;
using System.Reflection;
using System.Runtime.InteropServices;
using ARGlasses.Motion;
using UnityEngine;
using Random = System.Random;

namespace ARGlasses.Interaction.Motion
{
    /// <summary>
    /// Core class for managing second-order motion.
    /// Can be instantiated by a caller or MotionUtils.
    /// Critical function is Step(), which returns an updated
    /// value based on deltaTime and goal value.
    /// Abstract Motion class is required to allow
    /// allow rolling up into lists, e.g. MotionRunner.
    /// </summary>
    [Serializable]
    public abstract class Motion
    {
        public bool Debug = false;
        public string _id;
        public bool _isComplete = false;
        public bool _wasComplete = false;

        protected const float DEFAULT_EPSILON = .0001f;
        protected float _epsilon = DEFAULT_EPSILON;
        public float Epsilon {
            get => _epsilon;
            set => _epsilon = value;
        }
        
        public abstract void RunnerTriggerStep(float deltaTime);
    }

    [Serializable]
    public class Motion<T> : Motion where T : unmanaged
    {
        public AMotionParams MotionParams;

        [SerializeField] private Type _type;
        
        [SerializeField] private T _start => GetStart(); // only used for easing, not second-order motion
        [SerializeField] private T _current => GetCurrent();
        [SerializeField] private T _goal => GetGoal();
        
        public T Start => _start;
        public T Current => _current;
        public T Goal => _goal;
        
        
        [SerializeField] private float _elapsedTime;
        [SerializeField] private float _prevStepTime;
        public float PrevStepTime => _prevStepTime;
        
        private float _deltaTimeCeiling => 1f / 30f; // Used to prevent extreme motion jumps on Start
        
        private Action<T> DoStep;
        private Action DoComplete;

        #region Unsafe

        private int _size;
        private int _count;
        private System.IntPtr _current_mem;
        private System.IntPtr _goal_mem;
        private System.IntPtr _start_mem;
        private System.IntPtr _currentVel_mem;
        private System.IntPtr _prev_mem;
        private System.IntPtr _isComplete_mem;
        
        #endregion
        
        #region Initialization

        // used by reflection to create custom Motion types
        // ReSharper disable once RedundantOverload.Global
        public Motion(T current, AMotionParams motionParams = null) : this(current, current, motionParams, null) { }

        public Motion(T current, T goal, AMotionParams motionParams = null) : this(current, goal, motionParams, null) { }

        public Motion (T current, T goal, AMotionParams motionParams = null, Action<T> step = null)
        {
            _type = current.GetType();
            InitializeUnsafe(current, goal);
            InitializeComplete();
            InitializeVelocity();

            // RandomRangeInt is not allowed to be called from a MonoBehaviour instance field initializer :(
            // _id = UnityEngine.Random.Range(10000, 99999).ToString();
            _id = MotionUtils.RandomRange(10000, 99999).ToString();

            _elapsedTime = 0f;
            
            SetPrevious(current);
            SetStart(current);
            SetCurrent(current);
            SetGoal(goal);
            
            MotionParams = motionParams ?? AMotionParams.Default;
            
            if(step != null) OnStep(step);
            EnforceFinalValue();
        }

        ~Motion()
        {
            Marshal.FreeHGlobal(_current_mem);
            Marshal.FreeHGlobal(_goal_mem);
            Marshal.FreeHGlobal(_currentVel_mem);
            Marshal.FreeHGlobal(_prev_mem);
            Marshal.FreeHGlobal(_isComplete_mem);
            Marshal.FreeHGlobal(_start_mem);
        }

        void InitializeUnsafe(T current, T goal)
        {
            _size = Marshal.SizeOf(default(T));
            _count = _size / sizeof(float);
            
            _goal_mem = Marshal.AllocHGlobal(_size);
            _current_mem = Marshal.AllocHGlobal(_size);
            _currentVel_mem = Marshal.AllocHGlobal(_size);
            _prev_mem = Marshal.AllocHGlobal(_size);
            _isComplete_mem = Marshal.AllocHGlobal(_size);
            _start_mem = Marshal.AllocHGlobal(_size);
            
            SetCurrent(current);
            SetGoal(goal);
        }
        
        private void InitializeComplete()
        {
            unsafe
            {
                float* complete_prime = (float*)(_isComplete_mem);
                for (int i = 0; i < _count; i++)
                {
                    complete_prime[i] = 0f;
                }
            }
        }
        
        private void InitializeVelocity()
        {
            unsafe
            {
                float* currentVel_prime = (float*)(_currentVel_mem);
                for (int i = 0; i < _count; i++)
                {
                    currentVel_prime[i] = 0f;
                }
            }
        }
        
        void EnforceFinalValue()
        {
            DoComplete += () => { DoStep?.Invoke(_goal); }; // Ensure final value
        }

        #endregion

        #region Step

        /// Step forward through the motion based on the implicit
        /// original values of _current and _goal.
        public T Step(float deltaTime) => Step(_current, _goal, deltaTime);

        public T Step(T goal, float deltaTime) => Step(_current, goal, deltaTime);

        public T Step(T current, T goal, float deltaTime)
        {
            if(Debug) UnityEngine.Debug.Log($"Motion[{_id}] Motion.Step | current {current} | goal {goal}");
            
            // Limit to ceiling to avoid unexpected jumps on slow frames
            deltaTime = Mathf.Min(deltaTime, _deltaTimeCeiling);
            
            SetCurrent(current);
            SetGoal(goal);
            CheckDiscontinuity(current);
            _wasComplete = _isComplete;
            _isComplete = CheckComplete();
            
            if (!_wasComplete && _isComplete)
            {
                DoComplete?.Invoke();
            }

            if (_isComplete)
            {
                if(Debug) UnityEngine.Debug.Log($"Motion[{_id}] Motion.Step | isComplete {_isComplete} | returning goal {_goal}");
                return _goal;
            }

            ApplyMotionInfluence(current, goal, deltaTime); // immediately applies to memory

            if(Debug) UnityEngine.Debug.Log($"Motion[{_id}] Motion.Step | isComplete {_isComplete} | returning current {_current}");
            
            if (!_isComplete) DoStep?.Invoke(_current);
            return _current;
        }

        private Motion<T> SetPrevious(T previous)
        {
            unsafe
            {
                *(T*)_prev_mem = previous;
            }
            return this;
        }

        
        private T GetCurrent()
        {
            return Marshal.PtrToStructure<T>(_current_mem);
        }
        
        public Motion<T> SetCurrent(T current)
        {
            if(Debug) UnityEngine.Debug.Log($"Motion[{_id}] Motion.SetCurrent | current {current}");
            unsafe
            {
                *(T*)_current_mem = current;
            }
            return this;
        }

        private T GetStart()
        {
            return Marshal.PtrToStructure<T>(_start_mem);
        }
        
        private Motion<T> SetStart(T start)
        {
            unsafe
            {
                *(T*)_start_mem = start;
            }
            return this;
        }

        public Motion<T> SetEpsilon(float epsilon)
        {
            _epsilon = epsilon;
            return this;
        }

        private T GetGoal()
        {
            return Marshal.PtrToStructure<T>(_goal_mem);
        }
        
        public Motion<T> SetGoal(T goal)
        {
            if(Debug) UnityEngine.Debug.Log($"Motion[{_id}] Motion.SetGoal | goal {goal}");
            unsafe
            {
                *(T*)_goal_mem = goal;
            }
            return this;
        }
        
        public Motion<T> SetCurrentVelocity(T currentVelocity)
        {
            unsafe
            {
                *(T*)_currentVel_mem = currentVelocity;
            }
            return this;
        }
        
        public Motion<T> ResetEasingMotion(T start)
        {
            SetStart(start);
            _elapsedTime = 0f;
            _isComplete = false;
            return this;
        }
        
        // Chainable function to allow custom actions
        public Motion<T> OnStep(Action<T> action)
        {
            DoStep += action;
            return this;
        }

        public Motion<T> OnComplete(Action action)
        {
            DoComplete += action;
            return this;
        }

        public Motion<T> RemoveComplete(Action action)
        {
            DoComplete -= action;
            return this;
        }

        public Motion<T> ResetCallbacks()
        {
            DoComplete = null;
            EnforceFinalValue();
            return this;
        }

        public Motion<T> ResetEpsilon()
        {
            Epsilon = DEFAULT_EPSILON;
            return this;
        }
        
        #endregion

        #region Motion Logic
        
        public T ApplyMotionInfluence(T current, T goal, float deltaTime)
        {
            unsafe
            {
                float* current_prime = (float*)(_current_mem);
                float* goal_prime = (float*)(_goal_mem);

                if (MotionParams is MotionParams motionParamsSecondOrder)
                {
                    motionParamsSecondOrder.UpdateConstants();

                    for (int i = 0; i < _count; i++)
                    {
                        current_prime[i] = ApplySecondOrderMotion(i, motionParamsSecondOrder, deltaTime, current_prime[i], goal_prime[i], 0f);
                        current_prime[i] = IsValid(current_prime[i]) ? current_prime[i] : goal_prime[i];
                    }
                    
                } else if (MotionParams is MotionParamsEasing motionParamsEasing)
                {
                    _elapsedTime += deltaTime;
            
                    if (_elapsedTime < motionParamsEasing._delay)
                    {
                        return Marshal.PtrToStructure<T>(_start_mem);
                    }
                        
                    float t = motionParamsEasing._easing.Evaluate((_elapsedTime - motionParamsEasing._delay)/motionParamsEasing._duration);

                    for (int i = 0; i < _count; i++)
                    {
                        current_prime[i] = ApplyEasingMotion(i, t, deltaTime);
                        current_prime[i] = IsValid(current_prime[i]) ? current_prime[i] : goal_prime[i];
                    }
                }
            }
            return Marshal.PtrToStructure<T>(_current_mem);
        }

        public float ApplyEasingMotion(int i, float t, float deltaTime)
        {
            unsafe
            {
                float* start_prime = (float*)(_start_mem);
                float* goal_prime = (float*)(_goal_mem);
                return Mathf.Lerp(start_prime[i], goal_prime[i], t);
            }
        }

        // Source: https://youtu.be/KPoeNZZ6H4s
        public float ApplySecondOrderMotion(int i, MotionParams motionParams, float deltaTime, float currentPos, float goalPos, float targetVel)
        {
            unsafe
            {
                float* prev_prime = (float*)(_prev_mem);
                float* currVel_prime = (float*)(_currentVel_mem);
                
                //estimate velocity
                if (targetVel < 0.000001f)
                {
                    targetVel = (goalPos - prev_prime[i]) * (1f/deltaTime);
                    prev_prime[i] = goalPos;
                }

                // See here regarding instability:
                // https://youtu.be/KPoeNZZ6H4s?t=553

                //integrate pos by velocity
                float newCurrent = currentPos + deltaTime * currVel_prime[i];
                //integrate velocity by acceleration
                float newCurrentVelocity = currVel_prime[i] + deltaTime * (goalPos + motionParams._k3 * targetVel - newCurrent - motionParams._k1 * currVel_prime[i]) / motionParams._k2;
                currVel_prime[i] = newCurrentVelocity;
                
                if(Debug) UnityEngine.Debug.Log($"Motion[{_id}][{i}] Motion.ApplySecondOrderMotion | current {currentPos} | goal {goalPos} | targetVel {targetVel} | prev_prime {prev_prime[i]} | newCurrentVelocity {newCurrentVelocity} | newCurrent {newCurrent}");
                
                return newCurrent;   
            }
        }

        #endregion

        #region Utility

        public override void RunnerTriggerStep(float deltaTime)
        {
            Step(_current, _goal, deltaTime);
        }
        
        private bool IsValid(float f)
        {
            return !(float.IsNaN(f) || float.IsInfinity(f));
        }
        
        /// Enables re-use of the same Tween across interaction
        /// moments by ensuring we don't have a stale _previous value
        /// due to discontinuities in when Step was called.
        private void CheckDiscontinuity(T current)
        {
            if (Time.time - _prevStepTime > 0.1f)
            {
                _elapsedTime = 0f;
                SetPrevious(current);
                SetStart(current);
            }
        }

        private bool CheckComplete()
        {
            unsafe
            {
                bool isComplete = false;
                
                float* current_prime = (float*)(_current_mem);
                float* goal_prime = (float*)(_goal_mem);
                float* currVel_prime = (float*)(_currentVel_mem);
                float* isComplete_prime = (float*)(_isComplete_mem);
                
                for(int i=0; i < _count; i++){
                    bool isClose = Mathf.Abs(goal_prime[i] - current_prime[i]) < _epsilon;
                    bool hasStopped = Mathf.Abs(currVel_prime[i]) < _epsilon;
                    isComplete_prime[i] = (isClose && hasStopped) ? 1.0f : 0.0f;
                }

                bool areAllComplete = true;
                for (int i = 0; i < _count; i++)
                {
                    if (Mathf.Approximately(isComplete_prime[i], 0.0f))
                    {
                        areAllComplete = false;
                        break;
                    }
                }
                
                isComplete = areAllComplete;
                if(!isComplete) _prevStepTime = Time.time;
                return isComplete;
            }
        }

        #endregion

    }
}
