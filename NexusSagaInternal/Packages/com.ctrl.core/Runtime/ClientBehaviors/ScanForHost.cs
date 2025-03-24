using UnityEngine;
using UnityEngine.Events;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Stopwatch = System.Diagnostics.Stopwatch;

using CTRL.Utils;
using CTRL.Utils.Logging;

namespace CTRL.ClientBehaviors
{
  [ExecuteInEditMode]
  [DefaultExecutionOrder(ExecutionOrder.ScanForHost)]
  public class ScanForHost : ITK.DependBehavior
  {
    #if !UNITY_WEBGL

    [ITK.Depend(Flags = ITK.DependFlags.Scene)]
    public CTRLClient client;

    [SerializeField]
    private CTRLLogger logger = new CTRLLogger("ScanForHost", LogType.Error);
    public CTRLLogger Logger => logger;

    public int TimeoutMs = 1000;

    private bool isScanning = false;
    public bool IsScanning { get => isScanning; }

    /// Lifecycle events
    public class PotentialHostFoundEvent : UnityEvent<string> {}

    protected PotentialHostFoundEvent onPotentialHostFound = new PotentialHostFoundEvent();
    public PotentialHostFoundEvent OnPotentialHostFound { get => onPotentialHostFound; }

    protected CancellationTokenSource cancellationTokenSource = null;

    protected virtual void OnEnable()
    {
      client.ConfigCallbacks.Add(ScanOnTryConnect);
    }

    protected virtual void OnDisable()
    {
      if (cancellationTokenSource != null)
      {
        cancellationTokenSource.Cancel();
        cancellationTokenSource.Dispose();
        cancellationTokenSource = null;
      }

      client.ConfigCallbacks.Remove(ScanOnTryConnect);
    }

    /// <summary>
    /// Scans the local network for potential hosts, returning a list of hosts sorted by priority
    /// (highest first).
    /// </summary>
    /// <remarks>
    /// The previous host is read from CTRLClient on startup. This becomes the highest priority
    /// result if found during the scane. The highest priority result is assigned back to
    /// CTRLClient on completion.
    /// </remarks>
    public async Task<List<string>> Scan()
    {
      if (isScanning)
      {
        throw new Exception("Re-entrant call not allowed");
      }

      isScanning = true;
      cancellationTokenSource = new CancellationTokenSource();
      List<string> hosts = new List<string>();

      try
      {
        hosts = await ScanInternal(client.Port);
      }
      catch (Exception e)
      {
        logger.LogException(e);
      }

      if (cancellationTokenSource != null)
      {
        cancellationTokenSource.Dispose();
        cancellationTokenSource = null;
      }

      isScanning = false;

      return hosts;
    }

    public void CancelScan()
    {
      if (cancellationTokenSource != null)
      {
        cancellationTokenSource.Cancel();
      }
    }

    private async Task<List<string>> ScanInternal(int port)
    {
      Stopwatch sw = Stopwatch.StartNew();
      logger.Log($"Begin scanning for host on port {port} (timeout: {TimeoutMs} ms)");

      var hosts = new List<string>();

      var scanner = new NetworkScanner(
          port,
          TimeSpan.FromMilliseconds(TimeoutMs),
          cancellationTokenSource.Token,
          (string host) =>
          {
            logger.Log($"During scan, found potential host: {host}");
            hosts.Add(host);
            onPotentialHostFound.Invoke(host);
          });

      await scanner.Scan();

      hosts = scanner.SortHosts(hosts, client.Host);

      logger.Log($"Done scanning for host (elapsed time: {sw.ElapsedMilliseconds} ms)");

      return hosts;
    }

    private async Task<bool> ScanOnTryConnect()
    {
      var hosts = await Scan();

      if (hosts.Count == 1)
      {
        // Found a new CTRL-R host.
        client.Host = hosts[0];
        logger.Log($"Host found: {client.Host}");
        return true;
      }
      else if (hosts.Count < 1)
      {
        // CTRL-R is not running, or network/firewall/VPN issues prevent us
        // from reaching it.
        logger.LogWarning($"No hosts found!");
      }
      else if (hosts.Count > 1)
      {
        // Don't update CTRLClient when multiple hosts are found. If we have
        // multiple hosts that means the user needs to explicitly configure the
        // host using `LoadConfigFile`. Otherwise we get non-deterministic
        // behavior based on which host responds first.
        logger.LogError($"Multiple hosts found: {string.Join(", ", hosts)}");
      }

      return false;
    }
#endif
  }
}
