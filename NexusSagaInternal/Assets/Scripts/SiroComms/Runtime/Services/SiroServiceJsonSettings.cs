using Newtonsoft.Json;
using Newtonsoft.Json.Converters;
using Newtonsoft.Json.UnityConverters.Math;

namespace SiroComms.Runtime.Services
{
    public static class SiroServiceJsonSettings
    {
        public static readonly JsonSerializerSettings Settings;
        public static readonly JsonSerializerSettings TypeSettings;

        static SiroServiceJsonSettings()
        {
            Settings = new JsonSerializerSettings
            {
                // TypeNameHandling = TypeNameHandling.All,
                // TypeNameAssemblyFormatHandling = TypeNameAssemblyFormatHandling.Simple,
                NullValueHandling = NullValueHandling.Include,
                DefaultValueHandling = DefaultValueHandling.Include,
                Converters = new JsonConverter[] {
                    new StringEnumConverter(),
                    new ColorConverter(),
                    new Color32Converter(),
                    new VersionConverter(),
                    new Vector2Converter(),
                    new Vector3Converter(),
                    new QuaternionConverter()
                }
            };

            TypeSettings = new JsonSerializerSettings
            {
                TypeNameHandling = TypeNameHandling.All,
                TypeNameAssemblyFormatHandling = TypeNameAssemblyFormatHandling.Simple,
                NullValueHandling = NullValueHandling.Include,
                DefaultValueHandling = DefaultValueHandling.Include,
                Converters = new JsonConverter[] {
                    new StringEnumConverter(),
                    new ColorConverter(),
                    new Color32Converter(),
                    new VersionConverter(),
                    new Vector2Converter(),
                    new Vector3Converter(),
                    new QuaternionConverter()
                }
            };
        }
        
    }
}