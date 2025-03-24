// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static class LocalHostInfo
    {
        public static string[] GetIps()
        {
            // var localIpAddresses = LocalIpAddresses().Where(adr => adr.StartsWith("192")).ToArray();
            if (!System.Net.NetworkInformation.NetworkInterface.GetIsNetworkAvailable())
            {
                Debug.LogError("NetworkInformation.NetworkInterface.GetIsNetworkAvailable is false");
                return Array.Empty<string>();
            }

            IPHostEntry host = Dns.GetHostEntry(Dns.GetHostName());
            var addresses = host.AddressList.Where(ip => ip.AddressFamily == AddressFamily.InterNetwork).Select(ip => ip.ToString())
                .ToArray();
            Debug.Log($"localIpAddresses: {string.Join(", ", addresses)}");

            return addresses;
        }
    }
}
