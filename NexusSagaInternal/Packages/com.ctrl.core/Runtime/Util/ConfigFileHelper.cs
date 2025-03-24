using System;

using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

using CTRL.Utils.Logging;

namespace CTRL.Utils
{
  class ConfigFileHelper
  {
    public static JToken GetConfigFileJson(CTRLLogger logger)
    {
      string dirpath;
      try
      {
        dirpath = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData);
      }
      catch (Exception e)
      {
        logger.LogError("Could not get config file path: " + e.ToString());
        return null;
      }

      string filepath = dirpath + @"\ctrl.json";
      logger.Log("Attempting to read from config file: " + filepath);

      string content;
      try
      {
        content = System.IO.File.ReadAllText(filepath);
      }
      catch (Exception e)
      {
        logger.LogWarning("Could not load config file: " + e.ToString());
        return null;
      }

      return JObject.Parse(content);
    }

    public static int? ParseInt(JToken content, string key, CTRLLogger logger)
    {
      JToken data = ParseData(content, key, JTokenType.Integer, logger);
      if (data != null)
      {
        return (int) data;
      }
      else
      {
        return null;
      }
    }

    public static string ParseString(JToken content, string key, CTRLLogger logger)
    {
      JToken data = ParseData(content, key, JTokenType.String, logger);
      return data != null ? (string) data : null;
    }

    private static JToken ParseData(JToken json, string key, JTokenType type, CTRLLogger logger)
    {
      JToken token = json[key];

      if (token != null && token.Type == type)
      {
        logger.Log("Read value for '" + key + "' from config file: " + token);
        return token;
      }
      else
      {
        logger.LogWarning("Missing value for '" + key + "' in config file");
        return null;
      }
    }
  }
}
