using System.IO;
using System.Threading.Tasks;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Text.RegularExpressions;

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

        protected async Task<string> ReceiveAnswerAsync(string command_type)
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
                return commaSplit2[1];
            else
                return string.Empty;
        }

        /// <summary>
        /// Request calibration.
        /// <param name="mode">Whether to request automatic or manual calibration<param>
        /// </summary>
        public async Task Calibrate(CalibrateMode mode)
        {
            await SendCommandAsync(nameof(Calibrate), mode.ToString());
            await ReceiveAnswerAsync(nameof(Calibrate));
        }

        /// <summary>
        /// Set whether the robot should be in learning mode or not.
        /// <param name="mode">Activate learning mode or not</param>
        /// </summary>
        public async Task SetLearningMode(bool mode)
        {
            await SendCommandAsync(nameof(SetLearningMode), mode.ToString().ToUpper());
            await ReceiveAnswerAsync(nameof(SetLearningMode));
        }

        /// <summary>
        /// Move joints to specified configuration.
        /// <param name="joints">The desired destination joint configuration</param>
        /// </summary>
        public async Task MoveJoints(RobotJoints joints)
        {
            await SendCommandAsync(nameof(MoveJoints), string.Join(',', joints));
            await ReceiveAnswerAsync(nameof(MoveJoints));
        }

        /// <summary>
        /// Move joints to specified pose.
        /// <param name="pose">The desired destination pose</param>
        /// </summary>
        public async Task MovePose(PoseObject pose)
        {
            await SendCommandAsync(nameof(MovePose), string.Join(',', pose));
            await ReceiveAnswerAsync(nameof(MovePose));
        }

        /// <summary>
        /// Shift the pose along one axis.
        /// <param name="axis">Which axis to shift</param>
        /// <param name="value">The amount to shift (meters or radians)</param>
        /// </summary>
        public async Task ShiftPose(RobotAxis axis, float value)
        {
            await SendCommandAsync(nameof(ShiftPose), axis.ToString(), value.ToString());
            await ReceiveAnswerAsync(nameof(ShiftPose));
        }

        /// <summary>
        /// Set the maximum arm velocity.false
        /// <param name="velocity">The maximum velocity in percent of maximum velocity.</param>
        /// </summary>
        public async Task SetArmMaxVelocity(int velocity)
        {
            await SendCommandAsync(nameof(SetArmMaxVelocity), velocity.ToString());
            await ReceiveAnswerAsync(nameof(SetArmMaxVelocity));
        }

        /// <summary>
        /// Enable or disable joystick control.false
        /// </summary>
        public async Task EnableJoystick(bool mode)
        {
            await SendCommandAsync(nameof(EnableJoystick), mode.ToString().ToUpper());
            await ReceiveAnswerAsync(nameof(EnableJoystick));
        }

        /// <summary>
        /// Configure a GPIO pin for input or output.
        /// <summary>
        public async Task SetPinMode(RobotPin pin, PinMode mode)
        {
            await SendCommandAsync(nameof(SetPinMode), pin.ToString(), mode.ToString());
            await ReceiveAnswerAsync(nameof(SetPinMode));
        }

        /// <summary>
        /// Write to a digital pin configured as output.
        /// </summary>
        public async Task DigitalWrite(RobotPin pin, DigitalState state)
        {
            await SendCommandAsync(nameof(DigitalWrite), pin.ToString(), state.ToString());
            await ReceiveAnswerAsync(nameof(DigitalWrite));
        }

        /// <summary>
        /// Read from a digital pin configured as input.
        /// </summary>
        public async Task<DigitalState> DigitalRead(RobotPin pin)
        {
            await SendCommandAsync(nameof(DigitalRead), pin.ToString());
            var state = await ReceiveAnswerAsync(nameof(DigitalRead));
            return (DigitalState)Enum.Parse(typeof(DigitalState), state);
        }

        /// <summary>
        /// Select which tool is connected to the robot.
        /// </summary>
        public async Task ChangeTool(RobotTool tool)
        {
            await SendCommandAsync(nameof(ChangeTool), tool.ToString());
            await ReceiveAnswerAsync(nameof(ChangeTool));
        }

        /// <summary>
        /// Open the gripper.
        /// <param name="gripper">Which gripper to open</param>
        /// <param name="speed">The speed to use. Must be between 0 and 1000, recommended values between 100 and 500.</param>
        /// </summary>
        public async Task OpenGripper(RobotTool gripper, int speed)
        {
            await SendCommandAsync(nameof(OpenGripper), gripper.ToString(), speed.ToString());
            await ReceiveAnswerAsync(nameof(OpenGripper));
        }

        /// <summary>
        /// Close the gripper.
        /// <param name="gripper">Which gripper to close</param>
        /// <param name="speed">The speed to use. Must be between 0 and 1000, recommended values between 100 and 500.</param>
        /// </summary>
        public async Task CloseGripper(RobotTool gripper, int speed)
        {
            await SendCommandAsync(nameof(CloseGripper), gripper.ToString(), speed.ToString());
            await ReceiveAnswerAsync(nameof(CloseGripper));
        }

        /// <summary>
        /// Pull air using the vacuum pump.
        /// <param name="vacuumPump">Must be VACUUM_PUMP_1. Only one type available for now.</param>
        /// </summary>
        public async Task PullAirVacuumPump(RobotTool vacuumPump)
        {
            await SendCommandAsync(nameof(PullAirVacuumPump), vacuumPump.ToString());
            await ReceiveAnswerAsync(nameof(PullAirVacuumPump));
        }

        /// <summary>
        /// Push air using the vacuum pump.
        /// <param name="vacuumPump">Must be VACUUM_PUMP_1. Only one type available for now.</param>
        /// </summary>
        public async Task PushAirVacuumPump(RobotTool vacuumPump)
        {
            await SendCommandAsync(nameof(PushAirVacuumPump), vacuumPump.ToString());
            await ReceiveAnswerAsync(nameof(PushAirVacuumPump));
        }

        /// <summary>
        /// Setup the electromagnet.
        /// <param name="tool">Must be ELECTROMAGNET_1. Only one type available for now.</param>
        /// <param name="pin">The pin to which the magnet is connected.</param>
        /// </summary>
        public async Task SetupElectromagnet(RobotTool tool, RobotPin pin)
        {
            await SendCommandAsync(nameof(SetupElectromagnet), tool.ToString(), pin.ToString());
            await ReceiveAnswerAsync(nameof(SetupElectromagnet));
        }

        /// <summary>
        /// Activate the electromagnet.
        /// <param name="tool">Must be ELECTROMAGNET_1. Only one type available for now.</param>
        /// <param name="pin">The pin to which the magnet is connected.</param>
        /// </summary>
        public async Task ActivateElectromagnet(RobotTool tool, RobotPin pin)
        {
            await SendCommandAsync(nameof(ActivateElectromagnet), tool.ToString(), pin.ToString());
            await ReceiveAnswerAsync(nameof(ActivateElectromagnet));
        }

        /// <summary>
        /// Deactivate the electromagnet.
        /// <param name="tool">Must be ELECTROMAGNET_1. Only one type available for now.</param>
        /// <param name="pin">The pin to which the magnet is connected.</param>
        /// </summary>
        public async Task DeactivateElectromagnet(RobotTool tool, RobotPin pin)
        {
            await SendCommandAsync(nameof(DeactivateElectromagnet), tool.ToString(), pin.ToString());
            await ReceiveAnswerAsync(nameof(DeactivateElectromagnet));
        }

        /// <summary>
        /// Get the current joint configuration.
        /// </summary>
        public async Task<RobotJoints> GetJoints()
        {
            await SendCommandAsync(nameof(GetJoints));
            var joints = await ReceiveAnswerAsync(nameof(GetJoints));
            return RobotJoints.Parse(joints);
        }

        /// <summary>
        /// Get the current pose.
        /// </summary>
        public async Task<PoseObject> GetPose()
        {
            await SendCommandAsync(nameof(GetPose));
            var pose = await ReceiveAnswerAsync(nameof(GetPose));
            return PoseObject.Parse(pose);
        }

        /// <summary>
        /// Get the current hardware status.
        /// </summary>
        public async Task<HardwareStatus> GetHardwareStatus()
        {
            await SendCommandAsync(nameof(GetHardwareStatus));
            var status = await ReceiveAnswerAsync(nameof(GetHardwareStatus));
            return HardwareStatus.Parse(status);
        }

        /// <summary>
        /// Get whether the robot is in learning mode.
        /// </summary>
        public async Task<bool> GetLearningMode()
        {
            await SendCommandAsync(nameof(GetLearningMode));
            var mode = await ReceiveAnswerAsync(nameof(GetLearningMode));
            return bool.Parse(mode);
        }

        /// <summary>
        /// Get the current state of the digital io pins.
        /// </summary>
        public async Task<DigitalPinObject[]> GetDigitalIoState()
        {
            await SendCommandAsync(nameof(GetDigitalIoState));
            var state = await ReceiveAnswerAsync(nameof(GetDigitalIoState));

            var regex = new Regex("\\[[0-9]+, '[^']*', [0-9]+, [0-9+]\\]");
            var matches = regex.Matches(state);

            return matches.Select(m => DigitalPinObject.Parse(m.Value)).ToArray();
        }

    }
}