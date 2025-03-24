using System;
using System.Collections;
using System.Collections.Generic;
using ARGlasses.Interaction.Motion;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public abstract class MotionField<TState>
    {
        public Func<TState, AMotionParams> MotionParamsGetter;
            
        public abstract void InitializeMotion(TState current, TState target);
        public abstract TState StepState(TState transientStateToModify, TState currentState, TState targetState, float deltaTime);
        public abstract void ApplyState(TState state);
    }
        
    public class MotionField<TValue, TState> : MotionField<TState> where TValue : unmanaged
    {
        public Func<TValue> RefGetter;
        public Action<TValue> RefSetter;

        public Func<TState, TValue> StateGetter;
        public Action<TValue, TState> StateSetter;

        public Motion<TValue> Motion;

        public override void InitializeMotion(TState current, TState target)
        {
            Motion = new Motion<TValue>(StateGetter.Invoke(current), StateGetter.Invoke(target), MotionParamsGetter.Invoke(target));
        }

        public override TState StepState(TState transientStateToModify, TState currentState, TState targetState, float deltaTime)
        {
            Motion.MotionParams = MotionParamsGetter.Invoke(targetState);
            TValue value = Motion.Step(StateGetter.Invoke(currentState), StateGetter.Invoke(targetState), deltaTime);
            StateSetter?.Invoke(value, transientStateToModify);
            return transientStateToModify;
        }
            
        public override void ApplyState(TState state)
        {
            RefSetter?.Invoke(StateGetter.Invoke(state));
        }
    }
}
