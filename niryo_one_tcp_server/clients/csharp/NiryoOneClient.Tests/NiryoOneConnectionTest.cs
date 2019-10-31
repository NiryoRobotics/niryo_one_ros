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
        [TestMethod]
        public async Task Calibrate_SuccessfulAuto_Works()
        {   
            var streamReader = Substitute.For<System.IO.TextReader>();
            var streamWriter = Substitute.For<System.IO.TextWriter>();
            streamReader.ReadLineAsync().Returns(Task.FromResult("CALIBRATE:OK\n"));
            var connection = new NiryoOneConnection(streamReader, streamWriter);
            await connection.Calibrate(CalibrateMode.AUTO);
            await streamWriter.Received().WriteLineAsync("CALIBRATE:AUTO");
        }
        
        [TestMethod]
        public async Task Calibrate_SuccessfulManual_Works()
        {   
            var streamReader = Substitute.For<System.IO.TextReader>();
            var streamWriter = Substitute.For<System.IO.TextWriter>();
            streamReader.ReadLineAsync().Returns(Task.FromResult("CALIBRATE:OK\n"));
            var connection = new NiryoOneConnection(streamReader, streamWriter);
            await connection.Calibrate(CalibrateMode.MANUAL);
            await streamWriter.Received().WriteLineAsync("CALIBRATE:MANUAL");
        }        

        [TestMethod]
        public async Task Calibrate_Failure_Throws()
        {   
            var streamReader = Substitute.For<System.IO.TextReader>();
            var streamWriter = Substitute.For<System.IO.TextWriter>();
            streamReader.ReadLineAsync().Returns(Task.FromResult("CALIBRATE:KO,\"Sucks to be sucky\"\n"));
            var connection = new NiryoOneConnection(streamReader, streamWriter);
            var e = await Assert.ThrowsExceptionAsync<NiryoOneException>(async () => await connection.Calibrate(CalibrateMode.MANUAL));
            Assert.AreEqual("Sucks to be sucky", e.Reason);
            await streamWriter.Received().WriteLineAsync("CALIBRATE:MANUAL");
        }
    }
}
