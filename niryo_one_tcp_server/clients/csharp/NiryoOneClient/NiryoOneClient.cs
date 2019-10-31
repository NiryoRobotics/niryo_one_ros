using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Runtime.InteropServices.ComTypes;
using System.Text;
using System.Threading.Tasks;

namespace NiryoOneClient
{
  public enum CalibrateMode
  {
    AUTO,
    MANUAL
  }

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

    private string ToSnakeCaseUpper(string s)
    {
        return string.Concat(s.Select((x, i) => i > 0 && char.IsUpper(x) ? "_" + x.ToString() : x.ToString())).ToUpper();
    }

    public async Task Calibrate(CalibrateMode mode)
    {
      await SendCommandAsync(nameof(Calibrate), mode.ToString());
      await ReceiveAnswerAsync(nameof(Calibrate));
    }

    public async Task SetLearningMode(bool mode)
    {
      await SendCommandAsync(nameof(SetLearningMode), mode.ToString().ToUpper());
      await ReceiveAnswerAsync(nameof(SetLearningMode));
    }

    public async Task SendCommandAsync(string command_type, params string[] args)
    {
      if (!_client.Connected)
        throw new SocketException();

      string cmd;
      if (args.Any())
        cmd = $"{ToSnakeCaseUpper(command_type)}:{string.Join(",", args)}";
      else
        cmd = ToSnakeCaseUpper(command_type);

      await _client.GetStream().WriteAsync(Encoding.ASCII.GetBytes(cmd + "\n"));
    }

    public async Task<string[]> ReceiveAnswerAsync(string command_type)
    {
      var sr = new StreamReader(_client.GetStream());
      var result = await sr.ReadLineAsync();
      result = result.TrimEnd('\n');
      var colonSplit = result.Split(':', 2);
      var cmd = colonSplit[0];
      if (cmd != ToSnakeCaseUpper(command_type))
        throw new NiryoOneException("Wrong command response received.");
      var commaSplit2 = colonSplit[1].Split(':', 2);
      var status = commaSplit2[0];
      if (status == "KO")
        throw new NiryoOneException(commaSplit2[1]);

      return commaSplit2[1].Split(',');
    }
  }

  public class NiryoOneException : Exception
  {
    public NiryoOneException(string reason)
    {
    }
  }
}
