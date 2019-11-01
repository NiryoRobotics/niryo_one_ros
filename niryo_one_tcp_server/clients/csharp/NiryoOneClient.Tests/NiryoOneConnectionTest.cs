using Microsoft.VisualStudio.TestTools.UnitTesting;
using NSubstitute;
using System.IO;
using System.Text;
using System.Threading.Tasks;

namespace NiryoOneClient.Tests
{
    [TestClass]
    public class NiryoOneConnectionTest
    {
        private TextReader _streamReader;
        private TextWriter _streamWriter;

        private NiryoOneConnection _connection;

        public NiryoOneConnectionTest()
        {
            _streamReader = Substitute.For<System.IO.TextReader>();
            _streamWriter = Substitute.For<System.IO.TextWriter>();
            _connection = new NiryoOneConnection(_streamReader, _streamWriter);
        }

        [TestMethod]
        public async Task Calibrate_SuccessfulAuto_Works()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult("CALIBRATE:OK\n"));
            await _connection.Calibrate(CalibrateMode.AUTO);
            await _streamWriter.Received().WriteLineAsync("CALIBRATE:AUTO");
        }

        [TestMethod]
        public async Task Calibrate_SuccessfulManual_Works()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult("CALIBRATE:OK\n"));
            await _connection.Calibrate(CalibrateMode.MANUAL);
            await _streamWriter.Received().WriteLineAsync("CALIBRATE:MANUAL");
        }

        [TestMethod]
        public async Task Calibrate_Failure_Throws()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult("CALIBRATE:KO,\"Sucks to be sucky\"\n"));
            var e = await Assert.ThrowsExceptionAsync<NiryoOneException>(async () => await _connection.Calibrate(CalibrateMode.MANUAL));
            Assert.AreEqual("Sucks to be sucky", e.Reason);
            await _streamWriter.Received().WriteLineAsync("CALIBRATE:MANUAL");
        }

        [TestMethod]
        public async Task SetLearningMode_True_Works()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult("SET_LEARNING_MODE:OK"));
            await _connection.SetLearningMode(true);
            await _streamWriter.Received().WriteLineAsync("SET_LEARNING_MODE:TRUE");
        }
    }
}
