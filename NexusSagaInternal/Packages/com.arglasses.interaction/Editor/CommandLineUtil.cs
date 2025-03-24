using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static class CommandLineUtil
    {
        public static void WriteCSC()
        {
            var cscPath = Path.Combine(Application.dataPath, "csc.rsp");

            if (File.Exists(cscPath) && File.ReadAllText(cscPath).Contains("ARGLASSES_PROJECT")) return;

            using (StreamWriter sw = File.AppendText(cscPath))
            {
                sw.WriteLine("-define:GAZEIX_PROJECT");
                sw.WriteLine("-r:System.IO.Compression.FileSystem.dll");
            }

            AssetDatabase.Refresh();
        }

        public static Dictionary<string, string> GetCommandLineArguments()
        {
            const char commandStartCharacter = '-';

            Dictionary<string, string> commandToValueDictionary = new Dictionary<string, string>();
            var args = Environment.GetCommandLineArgs();
            for (var i = 0; i < args.Length; i++)
            {
                if (!args[i].StartsWith(commandStartCharacter.ToString())) continue;

                var command = args[i];
                var value = string.Empty;
                if (i < args.Length - 1 && !args[i + 1].StartsWith(commandStartCharacter.ToString()))
                {
                    value = args[i + 1];
                    i++;
                }

                if (!commandToValueDictionary.ContainsKey(command))
                {
                    commandToValueDictionary.Add(command, value);
                    Debug.LogWarning($"Found Argument: {command} => {value}");
                }
                else
                {
                    Debug.LogError("Duplicate command line argument " + command);
                }
            }

            return commandToValueDictionary;
        }

        public static void Log(params string[] details)
        {
            string tag = nameof(ARGlassesBuilder);
            string timeStamp = $"{DateTime.Now.ToLongTimeString()} [{Time.time}] -- ";
            Debug.Log(string.Join(", ", details.Prepend(tag).Append(timeStamp)));
        }
    }
}
