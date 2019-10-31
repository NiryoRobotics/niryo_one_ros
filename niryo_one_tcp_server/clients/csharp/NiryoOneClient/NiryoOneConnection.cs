using System.IO;
using System.Threading.Tasks;
using System.Linq;

namespace NiryoOneClient
{
    public class NiryoOneConnection
    {
        private readonly TextWriter _textWriter;
        private readonly TextReader _textReader;

        public NiryoOneConnection(TextReader streamReader, TextWriter streamWriter)
        {
            _textWriter = streamWriter;
            _textReader = streamReader;
        }

        internal async Task WriteLineAsync(string s)
        {
            await _textWriter.WriteLineAsync(s);
        }

        internal async Task<string> ReadLineAsync()
        {
            return await _textReader.ReadLineAsync();
        }

        protected string ToSnakeCaseUpper(string s)
        {
            return string.Concat(s.Select((x, i) => i > 0 && char.IsUpper(x) ? "_" + x.ToString() : x.ToString())).ToUpper();
        }

        protected async Task SendCommandAsync(string command_type, params string[] args)
        {
            string cmd;
            if (args.Any())
                cmd = $"{ToSnakeCaseUpper(command_type)}:{string.Join(",", args)}";
            else
                cmd = ToSnakeCaseUpper(command_type);
            await WriteLineAsync(cmd);
        }

        protected async Task<string[]> ReceiveAnswerAsync(string command_type)
        {
            var result = await ReadLineAsync();
            result = result.TrimEnd('\n');
            var colonSplit = result.Split(':', 2);
            var cmd = colonSplit[0];
            if (cmd != ToSnakeCaseUpper(command_type))
                throw new NiryoOneException("Wrong command response received.");
            var commaSplit2 = colonSplit[1].Split(',', 2);
            var status = commaSplit2[0];
            if (status != "OK")
                throw new NiryoOneException(commaSplit2[1].TrimStart('"').TrimEnd('"'));

            if (commaSplit2.Length > 1)
                return commaSplit2[1].Split(',');
            else 
                return new string[0];
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
    }
}