using System.Collections.Generic;
using UnityEngine;

namespace Logger.Runtime
{
    /**
     * Use for development only
     */
    public static class DevLog
    {
        private static int _level;
        public const int DEBUG = 1;
        public const int WARNING = 2;
        public const int ERROR = 3;
        private static List<string> _filters = new List<string>();

        public static void Initialise(int logLevel = DEBUG)
        {
            _level = logLevel;
        }

        public static void AddFilter(string filter)
        {
            _filters.Add(filter);
        }
        
        public static void RemoveFilter(string filter)
        {
            _filters.Remove(filter);
        }

        public static void ClearFilters()
        {
            _filters.Clear();
        }
        
        public static void LogError(string log)
        {
            if (_level >= ERROR || IsAllowed(log)) return;
            Debug.LogError(log);
        }

        public static void LogWarning(string log)
        {
            if (_level >= WARNING || IsAllowed(log)) return;
            Debug.LogWarning(log);
        }
        
        public static void Log(string log)
        {
            if (_level >= DEBUG || IsAllowed(log)) return;
            Debug.Log(log);
        }

        private static bool IsAllowed(string log)
        {
            if (_filters.Count == 0) return true;
            foreach (var filter in _filters)
            {
                if (log.Contains(filter)) return true;
            }

            return false;
        }
    }
}