using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ARGlasses.Interaction.Motion
{
    /// <summary>
    /// Caches history of motion to offload velocity and acceleration
    /// tracking from calling classes. Generics are collapsed to Vector4 for common operations.
    /// </summary>
    public abstract class MotionHistory
    {
        public abstract void LogHistory();
    }
    
    public class MotionHistory<T> : MotionHistory
    {
        private BoundedInverseList<MotionHistorySample> _history = new BoundedInverseList<MotionHistorySample>();
        private Func<T> _getter;

        private Type _type;
        public Func<T, Vector4> TypeToVector;
        public Func<Vector4, T> VectorToType;

        private Vector4 _displacement =>
            _history.BoundedWindow.Count <= 1 ? Vector4.zero : _history.BoundedWindow[0].Position - _history.BoundedWindow[1].Position;
        public T LatestDisplacement => VectorToType(_displacement);
        
        private Vector4 _position =>
            _history.BoundedWindow.Count == 0 ? Vector4.zero : _history.BoundedWindow[0].Position;
        public T LatestPosition => VectorToType(_position);
        private Vector4 _averagePosition => _history.BoundedWindow.Count == 0 ? Vector4.zero : GetAveragePosition();
        public T AveragePosition => VectorToType(_averagePosition);

        private Vector4 _velocity =>
            _history.BoundedWindow.Count == 0 ? Vector4.zero : _history.BoundedWindow[0].Velocity;
        public T LatestVelocity => VectorToType(_velocity);

        private Vector4 _averageVelocity => _history.BoundedWindow.Count == 0 ? Vector4.zero : GetAverageVelocity();
        public T AverageVelocity => VectorToType(_averageVelocity);
        
        private Vector4 _acceleration =>
            _history.BoundedWindow.Count == 0 ? Vector4.zero : _history.BoundedWindow[0].Acceleration;
        public T LatestAcceleration => VectorToType(_acceleration);
        
        public MotionHistory(Func<T> getter, int windowSize = 5)
        {
            _history.UpperBound = windowSize;
            
            _type = (getter.GetType()).GenericTypeArguments[0];
            VectorToType = MotionUtils.ConvertVectorToType<T>(_type);
            TypeToVector = MotionUtils.ConvertTypeToVector<T>(_type);
            
            _getter = getter;
            LogHistory();
            MotionUtils.InitializeHistory(this);
        }

        private Vector4 GetAveragePosition()
        {
            Vector4 sum = Vector4.zero;
            for (int i = 0; i < _history.BoundedWindow.Count; i++)
            {
                sum += _history.BoundedWindow[i].Position;
            }

            Vector4 average = sum * (1f / _history.BoundedWindow.Count);
            return average;
        }
        
        // Returns average across all samples
        private Vector4 GetAverageVelocity()
        {
            Vector4 sum = Vector4.zero;
            for (int i = 0; i < _history.BoundedWindow.Count; i++)
            {
                sum += _history.BoundedWindow[i].Velocity;
            }

            Vector4 average = sum * (1f / _history.BoundedWindow.Count);
            return average;
        }
        
        public override void LogHistory()
        {
            MotionHistorySample sampleThisFrame = new MotionHistorySample();

            sampleThisFrame.Position = TypeToVector(_getter.Invoke());
            sampleThisFrame.Time = Time.time;
            sampleThisFrame.DeltaTime = Time.deltaTime;

            if (_history.BoundedWindow.Count <= 1)
            {
                sampleThisFrame.Velocity = Vector4.zero;
                sampleThisFrame.Acceleration = Vector4.zero;
            }
            else
            {
                MotionHistorySample sampleLastFrame = _history.BoundedWindow[1];
                sampleThisFrame.Velocity = (sampleThisFrame.Position - sampleLastFrame.Position) / sampleThisFrame.DeltaTime;
                sampleThisFrame.Acceleration = (sampleThisFrame.Velocity - sampleLastFrame.Velocity) / sampleThisFrame.DeltaTime;
            }

            _history.Add(sampleThisFrame);
        }
    }

    public class MotionHistorySample
    {
        public static MotionHistorySample Empty = new MotionHistorySample()
        {
            Position = Vector4.zero,
            Velocity = Vector4.zero,
            Acceleration = Vector4.zero,
            Time = 0f,
            DeltaTime = 0f
        };
        
        public Vector4 Position;
        public Vector4 Velocity;
        public Vector4 Acceleration;
        public float Time;
        public float DeltaTime;
    }

    public class BoundedInverseList<T>
    {
        public int UpperBound = 5;
        public List<T> BoundedWindow = new List<T>();
        
        public void Add (T value)
        {
            if (BoundedWindow.Count == UpperBound)
            {
                BoundedWindow.Insert(0, value);
                BoundedWindow.RemoveAt(UpperBound);
            }
            else
            {
                BoundedWindow.Insert(0, value);
            }
        }
    }

}
