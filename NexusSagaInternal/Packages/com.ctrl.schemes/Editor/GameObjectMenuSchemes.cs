using UnityEngine;
using UnityEditor;

namespace CTRL.Inspector
{

  [ExecuteAlways]
  public class GameObjectMenuScheme
  {
    [MenuItem("GameObject/CTRL-SDK/CTRLPinchWithIMU", false, 2)]
    static void CreateCTRLPinchObject(MenuCommand menuCommand)
    {
      // Defined in [CTRL] Core/Editor/MenuPrefabCreator.cs
      PrefabUtil.CreatePrefab("Packages/com.ctrl.schemes/Prefabs/CTRLPinchWithIMU.prefab", menuCommand.context as GameObject);
    }

    [MenuItem("GameObject/CTRL-SDK/PinchDiagnosticsPanel", false, 2)]
    static void CreatePinchDiagnosticsPanel(MenuCommand menuCommand)
    {
      // Defined in [CTRL] Core/Editor/MenuPrefabCreator.cs
      PrefabUtil.CreatePrefab("Packages/com.ctrl.schemes/Prefabs/PinchDiagnosticsPanel.prefab", menuCommand.context as GameObject);
    }
  }
}
