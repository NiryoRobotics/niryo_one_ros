using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Runtime.InteropServices.ComTypes;
using System.Text;

namespace NiryoOneClient
{
  public class NiryoOneClient
  {
    private TcpClient _client;
    private int _port;
    private string _server;

    public NiryoOneClient(string server, int port)
    {
      _server = server;
      _port = port;
    }

    public void Connect()
    {
      if (_client != null)
      {
        _client.Close();
        _client.Dispose();
      }

      _client = new TcpClient(_server, _port);
    }


  }
}
