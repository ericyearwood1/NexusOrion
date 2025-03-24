using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ARGlasses.Interaction.Motion
{
    /// <summary>
    /// MonoBehavior that is instantiated at runtime to drive any takeover/one-shot
    /// motions that the calling class wants managed. You shouldn't have
    /// to deal with this class directly.
    /// </summary>
    public class MotionRunner : MonoBehaviour
    {
        List<Motion> _runnerMotions = new List<Motion>(); // Motions this runner is responsible for advancing in Update
        private Dictionary<string, Motion> _activeLookup = new Dictionary<string, Motion>(); // used to find active Motions by target
        private List<MotionHistory> _historyToLog = new List<MotionHistory>();

        void Update()
        {
            TrackHistory();
            DriveOneShots(Time.deltaTime);
        }
        
        #region Execution

        void TrackHistory()
        {
            for (int i = 0; i < _historyToLog.Count; i++)
            {
                _historyToLog[i].LogHistory();
            }
        }

        private readonly List<Motion> _runnerMotionsRemovalCache = new();
        void DriveOneShots(float deltaTime)
        {
            _runnerMotionsRemovalCache.Clear(); // do we need to do this to avoid concurrent modification of _runnerMotions while we loop it?

            for(int i = 0; i < _runnerMotions.Count; i++)
            {
                _runnerMotions[i].RunnerTriggerStep(deltaTime);
                if (_runnerMotions[i]._isComplete) _runnerMotionsRemovalCache.Add(_runnerMotions[i]);
            }

            foreach (var remove in _runnerMotionsRemovalCache) RemoveFromActive(remove);
        }
        
        #endregion

        #region History

        public MotionHistory<T> AddHistory<T>(MotionHistory<T> h)
        {
            _historyToLog.Add(h);
            return h;
        }

        #endregion
        
        #region Registry

        private string RunnerMatchKey(int instanceId, string matchType) => $"{instanceId.ToString()}+{matchType}";

        private void AddToActive(Motion motionToAdd, string matchKey)
        {
            // Generate a match key so this can be retargeted easily through convenience methods
            _activeLookup.Add(matchKey, motionToAdd);
            _runnerMotions.Add(motionToAdd);
        }

        private void AddToActive(Motion motionToAdd)
        {
            _runnerMotions.Add(motionToAdd);
        }

        private void RemoveFromActive(Motion motionToRemove)
        {
            // Remove from lookup
            foreach(var lookup in _activeLookup.Where(kvp => kvp.Value == motionToRemove).ToList())
            {
                _activeLookup.Remove(lookup.Key);
            }

            // Remove from active
            _runnerMotions.Remove(motionToRemove);
        }
        
        #endregion
        
        #region Transform takeovers

        public Motion TryGetExistingMotion(int instanceId, string matchType)
        {
            _activeLookup.TryGetValue(RunnerMatchKey(instanceId, matchType), out Motion motion);
            return motion;
        }
        
        public Motion<Vector3> AddPositionMotion(Transform transformToModify, Vector3 goalPosition, AMotionParams motionParams = null, bool isLocal = false)
        {
            string matchType = isLocal ? "LOCAL_POSITION" : "POSITION";
            var current = isLocal ? transformToModify.localPosition : transformToModify.position;
            void Setter(Vector3 v)
            {
                if (isLocal) transformToModify.localPosition = v;
                else transformToModify.position = v;
            }

            return AddMotion(current, Setter, goalPosition, motionParams, transformToModify, matchType);
        }
        
        public Motion<Quaternion> AddRotationMotion(Transform transformToModify, Quaternion goalRotation, AMotionParams motionParams = null, bool isLocal = false)
        {
            string matchType = isLocal ? "LOCAL_ROTATION" : "ROTATION";
            var current = isLocal ? transformToModify.localRotation : transformToModify.rotation;
            void Setter(Quaternion v)
            {
                if (isLocal) transformToModify.localRotation = v;
                else transformToModify.rotation = v;
            }

            return AddMotion(current, Setter, goalRotation, motionParams, transformToModify, matchType);
        }
        
        public Motion<Vector3> AddScaleMotion(Transform transformToModify, Vector3 goalScale, AMotionParams motionParams = null)
        {
            string matchType = "SCALE";
            void Setter(Vector3 v) => transformToModify.localScale = v;
            return AddMotion(transformToModify.localScale, Setter, goalScale, motionParams, transformToModify, matchType);
        }

        public Motion<float> AddFadeMotion(CanvasGroup canvasGroupToModify, float goalAlpha, AMotionParams motionParams = null)
        {
            string matchType = "FADE";
            void Setter(float v) => canvasGroupToModify.alpha = v;
            return AddMotion(canvasGroupToModify.alpha, Setter, goalAlpha, motionParams, canvasGroupToModify, matchType);
        }
        
        #endregion

        #region Generic takeovers

        public Motion AddMotion(Motion motion)
        {
            AddToActive(motion);
            return motion;
        }

        public Motion AddMotion<T>(Motion<T> motion, T current) where T : unmanaged
        {
            motion.SetCurrent(current);
            AddToActive(motion);
            return motion;
        }
        
        public Motion<T> AddMotion<T>(T current, Action<T> step, T goal, AMotionParams motionParams = null, UnityEngine.Object owner = null, string matchType = null) where T : unmanaged
        {
            motionParams ??= AMotionParams.Default;

            int instanceId = owner != null ? owner.GetInstanceID() : UnityEngine.Random.Range(10000,99999);
            if(string.IsNullOrEmpty(matchType)) matchType = "default";
            Motion match = TryGetExistingMotion(instanceId, matchType);

            if (match != null)
            {
                Motion<T> matchCast = match as Motion<T>;
                matchCast.MotionParams = motionParams;
                matchCast.SetGoal(goal);
                matchCast.ResetCallbacks();
                matchCast.ResetEpsilon();
                matchCast.ResetEasingMotion(matchCast.Current);
                return matchCast;
            }

            T startPosition = current;
            Motion<T> motion = new Motion<T>(startPosition, goal, motionParams, step);

            AddToActive(motion, RunnerMatchKey(instanceId, matchType));
            return motion;
        }

        #endregion
        
        #region Lifecycle

        public Motion Stop(Motion motionToStop)
        {
            // Check if Motion is in list
            Motion m = _runnerMotions.Find(x => x._id == motionToStop._id);
            if (motionToStop == null || m == null)
            {
                return null;
            }
            else
            {
                RemoveFromActive(m);
                return m;
            }
        }
        
        #endregion
        
    }
}
