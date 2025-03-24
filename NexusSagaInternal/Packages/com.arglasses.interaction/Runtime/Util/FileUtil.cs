using System;
using System.IO;
using System.Linq;
using System.Threading;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static class FileUtil
    {
        public static string CombineExpandPaths(string s1, string s2)
        {
            return Path.GetFullPath(Path.Combine(s1, s2));
        }
        public static readonly string EditorProjectPath = Application.dataPath;
        public static readonly string UnityProjectTempFolder = Application.temporaryCachePath;
        public static string UserPath => Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
        public static string DesktopPath { get; } = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);

        public static string MostRecentFile(string folderPath)
        {
            var files = Directory.GetFiles(folderPath);
            var descending = files.OrderByDescending(f => new FileInfo(f).CreationTime);
            var mostRecentFilePath = descending.FirstOrDefault();
            return mostRecentFilePath;
        }

        private static void CleanDirectory(string folder)
        {
            try
            {
                if (Directory.Exists(folder))
                {
                    Directory.Delete(folder, true);
                    Thread.Sleep(100); // Recursive deletes seem to happen async... better way to do this..?
                }

                Directory.CreateDirectory(folder);
            }
            catch
            {
                Debug.LogError($"Couldn't recreate {folder}... do you have a window open at that location?");
            }
        }
    }
}
