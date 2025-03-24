using UnityEngine;
using UnityEngine.Events;

using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;

using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using NativeWebSocket;

using CTRL.Utils;
using CTRL.Utils.Logging;

namespace CTRL
{
  [System.Serializable]
  /// <summary>
  /// Loads and maintains app version information (if available).
  /// Shows warning (enables children) if version mismatch.
  /// Disables itself if versions match or can't find the data.
  /// </summary>
  public class VersionControl : MonoBehaviour
  {
    public enum VersionStatus
    {
      Loading,
      Loaded,
      Timeout,
      Error,
      Destroyed,
    }

    [SerializeField]
    private CTRLLogger logger = new CTRLLogger("VersionControl");
    public CTRLLogger Logger => logger;

    private VersionStatus status = VersionStatus.Loading;
    public string currentVersion = null;
    public string latestVersion = null;
    private int timeout = 60;

    private TextMesh warningTextMesh;

    void Awake()
    {
      warningTextMesh = transform.Find("WarningText").GetComponent<TextMesh>();
      WaitForConfigFile(this.timeout);
    }

    void OnDestroy()
    {
      this.status = VersionStatus.Destroyed;
    }

    private async void WaitForConfigFile(int timeout)
    {
      logger.Log("Start looking for ctrl.json.");
      for (int i = 0; i < timeout; i++)
      {
          await Task.Delay(1000);
          if (this.status == VersionStatus.Loading)
          {
            TryLoad();
          }
          else
          {
            break;
          }
      }
      logger.LogWarning("ctrl.json not found after " + timeout + " seconds.");
      this.status = VersionStatus.Timeout;
    }

    private void TryLoad()
    {
      JToken content = ConfigFileHelper.GetConfigFileJson(logger);
      if (content != null)
      {
        this.currentVersion = ConfigFileHelper.ParseString(content, "current_version", logger);
        this.latestVersion = ConfigFileHelper.ParseString(content, "latest_version", logger);
        if (this.currentVersion != null && this.latestVersion != null)
        {
          this.status = VersionStatus.Loaded;
          if (this.latestVersion != this.currentVersion)
          {
            // If we loaded the version from ctrl.json and they do NOT match, show the warning
            warningTextMesh.text = ""+
              "Warning: App is out of date.\n\n"+
              "Current Version: " + this.currentVersion + "\n" +
              "Latest Version: " + this.latestVersion + "\n\n" +
              "(gaze click to dismiss)";
            foreach (Transform child in transform)
            {
              logger.LogWarning("This app is out of data according to ctrl.json");
              child.gameObject.SetActive(true);
            }
          }
          else
          {
            // If we loaded the version from ctrl.json and they DO match, set this to inactive
            logger.Log("App is up to date. Current version: " + this.currentVersion);
            this.gameObject.SetActive(false);
          }
        }
        else
        {
          // File loaded, but no version data
          this.status = VersionStatus.Error;
          this.gameObject.SetActive(false);
        }
      }
    }
  }
}
