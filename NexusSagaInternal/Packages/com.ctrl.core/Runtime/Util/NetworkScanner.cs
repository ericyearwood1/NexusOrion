using System;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Net.NetworkInformation;
using System.Net.Sockets;

namespace CTRL.Utils
{
  public class NetworkScanner
  {
    private readonly int port;
    private readonly TaskRunner taskRunner;
    private readonly Action<string> resultCallback;
    private readonly IEnumerable<string> localIPAddresses;

    public NetworkScanner(
      int port,
      TimeSpan timeout,
      CancellationToken cancellationToken = default,
      Action<string> resultCallback = null)
    {
      this.port = port;
      this.taskRunner = new TaskRunner(timeout, cancellationToken);
      this.resultCallback = resultCallback;
      this.localIPAddresses = FindMyIPAddresses();
    }

    public async Task Scan()
    {
      // We need to check UNITY_EDITOR here as well because while running in editor, the define
      // symbols for the selected platform are also defined (e.g. both UNITY_EDITOR and UNITY_WEBGL)
#if (!UNITY_WEBGL) || (UNITY_EDITOR)
      // Run the implementation on a background thread. Even though socket connections are handled
      // asynchronously, we spawn a lot of them and this can block the UI momentarily.
      await Task.Run(ScanInternal);
#endif
    }

    private async Task ScanInternal()
    {
      // Expand each local IP address to include all hosts on the local subnet
      var hostsToTry = localIPAddresses.SelectMany(ipAddress =>
      {
        var prefix = ipAddress.Substring(0, ipAddress.LastIndexOf("."));
        return Enumerable.Range(0, 256).Select(suffix => $"{prefix}.{suffix}");
      }).ToList();

      // Always try localhost IP address for ctrl-r running on local machine with default args
      hostsToTry.Add("127.0.0.1");

      // Start all connection tasks and run them in parallel.
      // Each connection task adds its hosts to the results list if the connection succeeds.
      // We can't reliably query the result of the task directly due to bugs in .NET framework
      // (after Task.WaitAll(), each Task.IsCompleted is false and Task.Result blocks).
      var results = new List<string>();
      var tasks = hostsToTry.Select(async (host) =>
      {
        if (await CanConnectTCP(host))
        {
          lock (this)
          {
            results.Add(host);
            resultCallback?.Invoke(host);
          }
        }
      });

      // Wait for all connection tasks to complete.
      await Task.WhenAll(tasks);
    }

    private async Task<bool> CanConnectTCP(string host)
    {
      TcpClient client = new TcpClient();
      try
      {
        bool taskCompleted = await taskRunner.RunTask(client.ConnectAsync(host, port));
        if (taskCompleted && client.Connected)
        {
#if WINDOWS_UWP
          // On HoloLens the socket will appear to connect successfully for unreachable hosts.
          // To work around this we additionally try to write to the socket. It doesn't matter
          // what payload we send, we just expect any response before the timeout expires.
          Byte[] data = System.Text.Encoding.ASCII.GetBytes("anybody home?");
          return await taskRunner.RunTask(client.GetStream().WriteAsync(data, 0, data.Length));
#else
          return true;
#endif
        }
      }
      catch (System.Net.Sockets.SocketException)
      {
        // Ignore
      }
      finally
      {
        client.Close();
      }

      return false;
    }

    private static IReadOnlyList<string> FindMyIPAddresses()
    {
      List<string> results = new List<string>();

      foreach (NetworkInterface ni in NetworkInterface.GetAllNetworkInterfaces())
      {
        if (ni.OperationalStatus == OperationalStatus.Down)
        {
          continue;
        }

        if (ni.NetworkInterfaceType != NetworkInterfaceType.Wireless80211
            && ni.NetworkInterfaceType != NetworkInterfaceType.Ethernet)
        {
          continue;
        }

        foreach (UnicastIPAddressInformation ip in ni.GetIPProperties().UnicastAddresses)
        {
          if (ip != null &&
              ip.Address.AddressFamily == System.Net.Sockets.AddressFamily.InterNetwork)
          {
            results.Add(ip.Address.ToString());
          }
        }
      }

      return results;
    }

    public List<string> SortHosts(List<string> hosts, string recentHost)
    {
      // Return connected hosts, with higher priority hosts listed first.
      return new List<string>(hosts.OrderBy(host => GetHostOrder(host, recentHost)));
    }

    private int GetHostOrder(string host, string recentHost)
    {
      int weight = 0;
      // 1. prioritize recent hosts
      if (host == recentHost)
      {
        weight += 1 << 2;
      }
      // 2. prioritize hosts corresponding to the local device.
      if (localIPAddresses.Contains(host))
      {
        weight += 1 << 1;
      }
      // 3. prioritize hosts on the local network.
      if (host.StartsWith("192.168."))
      {
        weight += 1 << 0;
      }
      // Reverse weight to put higher weights first in the list.
      return -weight;
    }
  }

  public class TaskRunner
  {
    private readonly TimeSpan timeout;
    private readonly CancellationToken cancellationToken;

    public TaskRunner(TimeSpan timeout, CancellationToken cancellationToken)
    {
      this.timeout = timeout;
      this.cancellationToken = cancellationToken;
    }

    public async Task<bool> RunTask(Task task)
    {
      Task completedTask = await Task.WhenAny(
          task, Task.Delay(timeout), GetCancellationTask(cancellationToken));
      return task == completedTask && task.Status == TaskStatus.RanToCompletion;
    }

    private static Task GetCancellationTask(CancellationToken cancellationToken)
    {
      TaskCompletionSource<bool> tcs = new TaskCompletionSource<bool>();
      cancellationToken.Register(() => tcs.TrySetCanceled());
      return tcs.Task;
    }
  }

}
