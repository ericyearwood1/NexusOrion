using UnityEngine;
using UnityEditor;

using System.Collections.Generic;
using Newtonsoft.Json.Linq;

using CTRL.ClientBehaviors;

namespace CTRL.Inspector
{
  [CustomEditor(typeof(ParameterModification))]
  public class ParameterModificationEditor : Editor
  {
    protected Dictionary<string, bool> foldoutState = new Dictionary<string, bool>();

    protected JObject baseData = null;
    protected JObject currData = null;
    protected JObject edits = new JObject();

    protected virtual void OnDisable()
    {
      baseData = null;
      currData = null;
    }

    public override void OnInspectorGUI()
    {
      var oBackgroundColor = GUI.backgroundColor;

      DrawDefaultInspector();

      var vis = target as ParameterModification;
      if (vis == null)
      {
        return;
      }

      EditorGUILayout.Separator();

      bool shouldCommit = false;

      // Button can activate save
      GUI.enabled = edits.Count > 0;
      if (GUILayout.Button("Send new parameters (or press Enter)"))
      {
        shouldCommit = true;
      }
      GUI.enabled = true;

      if (shouldCommit)
      {
        var stream = vis.Stream;
        if (stream != null)
        {
          var client = stream.Client;
          if (client != null)
          {
            client.SendRequest(new JObject {
            {"change_parameter_request", new JObject {
              {"transforms", edits}
            }}
          });
          }

          edits = new JObject();
        }
      }

      if (GUILayout.Button("Reload from CTRL-R"))
      {
        baseData = null;
        currData = null;
        edits = new JObject();
      }

      EditorGUILayout.Separator();

      // Load first data
      JObject latestData = null;
      if (vis.Stream != null && vis.Stream.Latest.HasValue)
      {
        latestData = vis.Stream.Latest.Value.data;
      }

      if (baseData == null && latestData != null)
      {
        baseData = latestData;
      }

      if (currData == null && baseData != null)
      {
        currData = (JObject)baseData.DeepClone();
      }

      // Render nothing if we haven't gotten data
      if (baseData == null || currData == null)
      {
        return;
      }

      foreach (var transformer in currData)
      {
        if (vis.TransformerFilter != "" && vis.TransformerFilter != "*" && !transformer.Key.Contains(vis.TransformerFilter))
        {
          continue;
        }

        if (!foldoutState.ContainsKey(transformer.Key))
        {
          foldoutState.Add(transformer.Key, false);
        }

        var folderLabel = transformer.Key;
        var wasFolded = EditorGUILayout.Foldout(foldoutState[folderLabel], folderLabel);
        foldoutState[transformer.Key] = wasFolded;

        if (wasFolded)
        {
          continue;
        }

        EditorGUI.indentLevel++;

        var parameters = (JObject)transformer.Value;
        foreach (var param in parameters)
        {
          var fieldLabel = param.Key;

          JToken baseToken = baseData[transformer.Key]?[param.Key]?["value"];
          JToken currToken = param.Value["value"];

          string reportedType = param.Value["type"].ToObject<string>();
          JTokenType tokenType = currToken.Type;

          // Set background color after change
          bool hasDifference = baseToken.ToString() != currToken.ToString();
          if (hasDifference)
          {
            GUI.backgroundColor = Color.yellow;
          }

          if (tokenType == JTokenType.Boolean && reportedType.Equals("boolean"))
          {
            bool currValue = currToken.ToObject<bool>();
            bool newValue = EditorGUILayout.Toggle(fieldLabel, currValue);
            TryMakeEdit(transformer.Key, param.Key, currValue, newValue);
          }
          else if (reportedType.Equals("integer"))
          {
            int currValue = currToken.ToObject<int>();
            int newValue;

            var range = param.Value["range"];
            if (range != null && range.HasValues)
            {
              newValue = EditorGUILayout.IntSlider(fieldLabel, currValue, range.Value<int>(0), range.Value<int>(1));
            }
            else
            {
              newValue = EditorGUILayout.IntField(fieldLabel, currValue);
            }

            TryMakeEdit(transformer.Key, param.Key, currValue, newValue);
          }
          else if (reportedType.Equals("float"))
          {
            float currValue = currToken.ToObject<float>();
            float newValue;

            var range = param.Value["range"];
            if (range != null && range.HasValues)
            {
              newValue = EditorGUILayout.Slider(fieldLabel, currValue, range.Value<float>(0), range.Value<float>(1));
            }
            else
            {
              newValue = EditorGUILayout.FloatField(fieldLabel, currValue);
            }

            TryMakeEdit(transformer.Key, param.Key, currValue, newValue);
          }
          else if (reportedType.Equals("string"))
          {
            string currValue = currToken.ToObject<string>();
            string newValue = EditorGUILayout.TextField(fieldLabel, currValue);
            TryMakeEdit(transformer.Key, param.Key, currValue, newValue);
          }
          else if (reportedType.Equals("object"))
          {
            string currValue;
            if (tokenType == JTokenType.String)
            {
              currValue = currToken.ToObject<string>();
            }
            else
            {
              currValue = currToken.ToString(Newtonsoft.Json.Formatting.Indented);
            }

            // Special color coding for objects
            bool hasEdit = true;
            bool hasParseError = false;
            try
            {
              var baseStr = baseToken.ToString();
              var currStr = JToken.Parse(currValue).ToString();
              hasEdit = baseStr != currStr;
            }
            catch
            {
              hasParseError = true;
            }

            if (hasParseError)
            {
              GUI.backgroundColor = Color.red;
            }
            else if (hasEdit)
            {
              GUI.backgroundColor = Color.yellow;
            }
            else
            {
              GUI.backgroundColor = oBackgroundColor;
            }

            EditorGUILayout.LabelField(fieldLabel);
            string newValue = EditorGUILayout.TextArea(currValue);
            newValue = newValue.Replace("\r", "");
            TryMakeEdit(transformer.Key, param.Key, currValue, newValue);
          }
          else
          {
            EditorGUILayout.LabelField($"{fieldLabel} has reported type {reportedType}");
          }

          // Always reset background color
          GUI.backgroundColor = oBackgroundColor;
        }

        EditorGUI.indentLevel--;
      }
    }

    void TryMakeEdit(string transformerId, string paramId, bool currValue, bool newValue)
    {
      if (currValue == newValue)
      {
        return;
      }

      MakeEdit(transformerId, paramId, newValue.ToString());
    }

    void TryMakeEdit(string transformerId, string paramId, int currValue, int newValue)
    {
      if (currValue == newValue)
      {
        return;
      }

      MakeEdit(transformerId, paramId, newValue.ToString());
    }

    void TryMakeEdit(string transformerId, string paramId, float currValue, float newValue)
    {
      if (Mathf.Abs(currValue - newValue) < 0.0000001)
      {
        return;
      }

      MakeEdit(transformerId, paramId, newValue.ToString());
    }

    void TryMakeEdit(string transformerId, string paramId, string currValue, string newValue)
    {
      if (currValue.Equals(newValue))
      {
        return;
      }

      MakeEdit(transformerId, paramId, newValue);
    }

    void MakeEdit(string transformerId, string paramId, string newValue)
    {
      currData[transformerId][paramId]["value"] = newValue;

      // Create default edit message
      if (edits[transformerId] == null)
      {
        edits[transformerId] = new JObject {
          {"parameters", new JObject {}}
        };
      }

      // Message format involves only strings as values
      edits[transformerId]["parameters"][paramId] = newValue;
    }
  }
}
