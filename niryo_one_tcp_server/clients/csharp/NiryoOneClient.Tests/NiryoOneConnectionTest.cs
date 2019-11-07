using Microsoft.VisualStudio.TestTools.UnitTesting;
using NSubstitute;
using System.IO;
using System.Threading.Tasks;
using System.Linq;

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
            _streamReader.ReadLineAsync().Returns(Task.FromResult("CALIBRATE:KO,\"Sucks to be sucky\""));
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

        [TestMethod]
        public async Task SetLearningMode_False_Works()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult("SET_LEARNING_MODE:OK"));
            await _connection.SetLearningMode(false);
            await _streamWriter.Received().WriteLineAsync("SET_LEARNING_MODE:FALSE");
        }

        [TestMethod]
        public async Task SetLearningMode_WrongResponse_Throws()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult("COCO:OK"));
            var e = await Assert.ThrowsExceptionAsync<NiryoOneException>(async () => await _connection.SetLearningMode(false));
            Assert.AreEqual("Wrong command response received.", e.Reason);
            await _streamWriter.Received().WriteLineAsync("SET_LEARNING_MODE:FALSE");
        }

        [TestMethod]
        public async Task SetLearningMode_ErrorResponse_Throws()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult("SET_LEARNING_MODE:KO,\"No good\""));
            var e = await Assert.ThrowsExceptionAsync<NiryoOneException>(async () => await _connection.SetLearningMode(false));
            Assert.AreEqual("No good", e.Reason);
            await _streamWriter.Received().WriteLineAsync("SET_LEARNING_MODE:FALSE");
        }

        [TestMethod]
        public async Task MoveJoints_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult("MOVE_JOINTS:OK"));
            await _connection.MoveJoints(new RobotJoints(new[] {
                 0.03f, 0.0123f, 0.456f, 0.987f, 0.654f, 0.321f
                 }));
            await _streamWriter.Received().WriteLineAsync("MOVE_JOINTS:0.03,0.0123,0.456,0.987,0.654,0.321");
        }

        [TestMethod]
        public async Task MovePose_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult("MOVE_POSE:OK"));
            await _connection.MovePose(new PoseObject(new[] {
                 0.03f, 0.0123f, 0.456f, 0.987f, 0.654f, 0.321f
                 }));
            await _streamWriter.Received().WriteLineAsync("MOVE_POSE:0.03,0.0123,0.456,0.987,0.654,0.321");
        }

        [TestMethod]
        public async Task ShiftPose_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult("SHIFT_POSE:OK"));
            await _connection.ShiftPose(RobotAxis.ROLL, 0.03142f);
            await _streamWriter.Received().WriteLineAsync("SHIFT_POSE:ROLL,0.03142");
        }

        [TestMethod]
        public async Task SetArmMaxVelocity_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult("SET_ARM_MAX_VELOCITY:OK"));
            await _connection.SetArmMaxVelocity(50);
            await _streamWriter.Received().WriteLineAsync("SET_ARM_MAX_VELOCITY:50");
        }

        [TestMethod]
        public async Task EnableJoystick_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult("ENABLE_JOYSTICK:OK"));
            await _connection.EnableJoystick(false);
            await _streamWriter.Received().WriteLineAsync("ENABLE_JOYSTICK:FALSE");
        }

        [TestMethod]
        public async Task SetPinMode_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult("SET_PIN_MODE:OK"));
            await _connection.SetPinMode(RobotPin.GPIO_2B, PinMode.OUTPUT);
            await _streamWriter.Received().WriteLineAsync("SET_PIN_MODE:GPIO_2B,OUTPUT");
        }

        [TestMethod]
        public async Task DigitalWrite_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult(
                "DIGITAL_WRITE:OK"
            ));
            await _connection.DigitalWrite(RobotPin.GPIO_2A, DigitalState.LOW);
            await _streamWriter.Received().WriteLineAsync("DIGITAL_WRITE:GPIO_2A,LOW");
        }

        [TestMethod]
        public async Task DigitalRead_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult(
                "DIGITAL_READ:OK,HIGH"
            ));
            await _connection.DigitalRead(RobotPin.GPIO_1A);
            await _streamWriter.Received().WriteLineAsync("DIGITAL_READ:GPIO_1A");
        }

        [TestMethod]
        public async Task ChangeTool_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult(
                "CHANGE_TOOL:OK"
            ));
            await _connection.ChangeTool(RobotTool.GRIPPER_2);
            await _streamWriter.Received().WriteLineAsync("CHANGE_TOOL:GRIPPER_2");
        }

        [TestMethod]
        public async Task OpenGripper_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult(
                "OPEN_GRIPPER:OK"
            ));
            await _connection.OpenGripper(RobotTool.GRIPPER_1, 200);
            await _streamWriter.Received().WriteLineAsync("OPEN_GRIPPER:GRIPPER_1,200");
        }

        [TestMethod]
        public async Task CloseGripper_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult(
                "CLOSE_GRIPPER:OK"
            ));
            await _connection.CloseGripper(RobotTool.GRIPPER_1, 200);
            await _streamWriter.Received().WriteLineAsync("CLOSE_GRIPPER:GRIPPER_1,200");
        }

        [TestMethod]
        public async Task PullAirVacuumPump_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult(
                "PULL_AIR_VACUUM_PUMP:OK"
            ));
            await _connection.PullAirVacuumPump(RobotTool.VACUUM_PUMP_1);
            await _streamWriter.Received().WriteLineAsync("PULL_AIR_VACUUM_PUMP:VACUUM_PUMP_1");
        }

        [TestMethod]
        public async Task PushAirVacuumPump_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult(
                "PUSH_AIR_VACUUM_PUMP:OK"
            ));
            await _connection.PushAirVacuumPump(RobotTool.VACUUM_PUMP_1);
            await _streamWriter.Received().WriteLineAsync("PUSH_AIR_VACUUM_PUMP:VACUUM_PUMP_1");
        }

        [TestMethod]
        public async Task SetupElectromagnet_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult(
                "SETUP_ELECTROMAGNET:OK"
            ));
            await _connection.SetupElectromagnet(RobotTool.ELECTROMAGNET_1, RobotPin.GPIO_2B);
            await _streamWriter.Received().WriteLineAsync("SETUP_ELECTROMAGNET:ELECTROMAGNET_1,GPIO_2B");
        }

        [TestMethod]
        public async Task ActivateElectromagnet_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult(
                "ACTIVATE_ELECTROMAGNET:OK"
            ));
            await _connection.ActivateElectromagnet(RobotTool.ELECTROMAGNET_1, RobotPin.GPIO_2B);
            await _streamWriter.Received().WriteLineAsync("ACTIVATE_ELECTROMAGNET:ELECTROMAGNET_1,GPIO_2B");
        }

        [TestMethod]
        public async Task DeactivateElectromagnet_Sample_SendsCorrectly()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult(
                "DEACTIVATE_ELECTROMAGNET:OK"
            ));
            await _connection.DeactivateElectromagnet(RobotTool.ELECTROMAGNET_1, RobotPin.GPIO_2C);
            await _streamWriter.Received().WriteLineAsync("DEACTIVATE_ELECTROMAGNET:ELECTROMAGNET_1,GPIO_2C");
        }

        [TestMethod]
        public async Task GetJoints_Sample_Works()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult(
                "GET_JOINTS:OK,0.0,0.640187,-1.397485,0.0,0.0,0.0"
            ));
            var joints = await _connection.GetJoints();
            CollectionAssert.AreEqual(new[] { 0.0f, 0.640187f, -1.397485f, 0.0f, 0.0f, 0.0f }, joints.ToArray());
        }

        [TestMethod]
        public async Task GetPose_Sample_Works()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult(
                "GET_POSE:OK,0.0695735635306,1.31094787803e-12,0.200777981243,-5.10302119597e-12,0.757298,5.10351727471e-12"
            ));
            var pose = await _connection.GetPose();
            CollectionAssert.AreEqual(new[] { 0.0695735635306f, 1.31094787803e-12f, 0.200777981243f, -5.10302119597e-12f, 0.757298f, 5.10351727471e-12f },
            pose.ToArray());
        }

        [TestMethod]
        public async Task GetHardwareStatus_Sample_Works()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult(
                "GET_HARDWARE_STATUS:OK,59,2,True,'',0,False,['Stepper Axis 1', 'Stepper Axis 2', 'Stepper Axis 3', 'Servo Axis 4', 'Servo Axis 5', 'Servo Axis 6'],['Niryo Stepper', 'Niryo Stepper', 'Niryo Stepper', 'DXL XL-430', 'DXL XL-430', 'DXL XL-320'],(34, 34, 37, 43, 45, 37),(0.0, 0.0, 0.0, 11.3, 11.2, 7.9),(0, 0, 0, 0, 0, 0)"
            ));
            var status = await _connection.GetHardwareStatus();
            Assert.AreEqual(59, status.RpiTemperature);
            Assert.AreEqual(2, status.HardwareVersion);
            Assert.AreEqual(true, status.ConnectionUp);
            Assert.AreEqual("", status.ErrorMessage);
            Assert.AreEqual(0, status.CalibrationNeeded);
            Assert.AreEqual(false, status.CalibrationInProgress);
            CollectionAssert.AreEqual(new[] { "Stepper Axis 1", "Stepper Axis 2", "Stepper Axis 3", "Servo Axis 4", "Servo Axis 5", "Servo Axis 6" }, status.MotorNames);
            CollectionAssert.AreEqual(new[] { "Niryo Stepper", "Niryo Stepper", "Niryo Stepper", "DXL XL-430", "DXL XL-430", "DXL XL-320" }, status.MotorTypes);
            CollectionAssert.AreEqual(new[] { 34, 34, 37, 43, 45, 37 }, status.Temperatures);
            CollectionAssert.AreEqual(new[] { 0.0m, 0.0m, 0.0m, 11.3m, 11.2m, 7.9m }, status.Voltages);
            CollectionAssert.AreEqual(new[] { 0, 0, 0, 0, 0, 0 }, status.HardwareErrors);
        }

        [TestMethod]
        public async Task GetLearningMode_Sample_Works()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult("GET_LEARNING_MODE:OK,FALSE"));
            var mode = await _connection.GetLearningMode();
            await _streamWriter.Received().WriteLineAsync("GET_LEARNING_MODE");
            Assert.AreEqual(false, mode);
        }

        [TestMethod]
        public async Task GetDigitalIoState_Sample_Works()
        {
            _streamReader.ReadLineAsync().Returns(Task.FromResult(
                "GET_DIGITAL_IO_STATE:OK,[2, '1A', 1, 1],[3, '1B', 1, 1],[16, '1C', 1, 1],[26, '2A', 1, 1],[19, '2B', 1, 1],[6, '2C', 1, 1],[12, 'SW1', 0, 0],[13, 'SW2', 0, 0]"
                ));
            var state = await _connection.GetDigitalIoState();
            Assert.AreEqual(2, state[0].PinId);
            Assert.AreEqual("1A", state[0].Name);
            Assert.AreEqual(PinMode.INPUT, state[0].Mode);
            Assert.AreEqual(DigitalState.HIGH, state[0].State);

            Assert.AreEqual(3, state[1].PinId);
            Assert.AreEqual("1B", state[1].Name);
            Assert.AreEqual(PinMode.INPUT, state[1].Mode);
            Assert.AreEqual(DigitalState.HIGH, state[1].State);

            Assert.AreEqual(16, state[2].PinId);
            Assert.AreEqual("1C", state[2].Name);
            Assert.AreEqual(PinMode.INPUT, state[2].Mode);
            Assert.AreEqual(DigitalState.HIGH, state[2].State);

            Assert.AreEqual(26, state[3].PinId);
            Assert.AreEqual("2A", state[3].Name);
            Assert.AreEqual(PinMode.INPUT, state[3].Mode);
            Assert.AreEqual(DigitalState.HIGH, state[3].State);

            Assert.AreEqual(19, state[4].PinId);
            Assert.AreEqual("2B", state[4].Name);
            Assert.AreEqual(PinMode.INPUT, state[4].Mode);
            Assert.AreEqual(DigitalState.HIGH, state[4].State);

            Assert.AreEqual(6, state[5].PinId);
            Assert.AreEqual("2C", state[5].Name);
            Assert.AreEqual(PinMode.INPUT, state[5].Mode);
            Assert.AreEqual(DigitalState.HIGH, state[5].State);

            Assert.AreEqual(12, state[6].PinId);
            Assert.AreEqual("SW1", state[6].Name);
            Assert.AreEqual(PinMode.OUTPUT, state[6].Mode);
            Assert.AreEqual(DigitalState.LOW, state[6].State);

            Assert.AreEqual(13, state[7].PinId);
            Assert.AreEqual("SW2", state[7].Name);
            Assert.AreEqual(PinMode.OUTPUT, state[7].Mode);
            Assert.AreEqual(DigitalState.LOW, state[7].State);
        }
    }
}
