using UnityEngine;
using UnityEditor;

namespace CTRL.Inspector
{

  [ExecuteAlways]
  public class GameObjectMenuCore
  {
    [MenuItem("GameObject/CTRL-SDK/CTRLClient", false, 1)]
    static void CreateCTRLClientGameObject(MenuCommand menuCommand)
    {
      PrefabUtil.CreatePrefab("Packages/com.ctrl.core/Prefabs/CTRLClient.prefab", menuCommand.context as GameObject);
    }

    [MenuItem("GameObject/CTRL-SDK/CTRLDebugPanel", false, 1)]
    static void CreateCTRLDebugPanelGameObject(MenuCommand menuCommand)
    {
      PrefabUtil.CreatePrefab("Packages/com.ctrl.core/Prefabs/CTRLDebugPanel.prefab", menuCommand.context as GameObject);
    }
  }
}
