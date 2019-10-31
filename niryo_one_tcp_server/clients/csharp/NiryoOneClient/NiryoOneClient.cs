using System;
using System.Net.Sockets;
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

        public async Task<NiryoOneConnection> Connect()
        {
            if (_client != null)
            {
                _client.Close();
                _client.Dispose();
                _client = null;
            }

            _client = new TcpClient();
            await _client.ConnectAsync(_server, _port);
            var stream = _client.GetStream();
            return new NiryoOneConnection(new System.IO.StreamReader(stream), new System.IO.StreamWriter(stream));
        }
    }
}
