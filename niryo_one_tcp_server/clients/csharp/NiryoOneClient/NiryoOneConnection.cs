
/*  MIT License

    Copyright (c) 2019 Niryo

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
 */

using System;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

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
            await _textWriter.FlushAsync();
        }

        internal async Task<string> ReadLineAsync()
        {
            return await _textReader.ReadLineAsync();
        }

        protected async Task SendCommandAsync(string command_type, params string[] args)
        {
            string cmd;
            if (args.Any())
                cmd = $"{command_type}:{string.Join(",", args)}";
            else
                cmd = command_type;
            await WriteLineAsync(cmd);
        }

        protected async Task<string> ReceiveAnswerAsync(string command_type)
        {
            var result = await ReadLineAsync();
            result = result.TrimEnd('\n');
            var colonSplit = result.Split(':', 2);
            var cmd = colonSplit[0];
            if (cmd != command_type)
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
        /// <param name="mode">Whether to request automatic or manual calibration</param>
        /// </summary>
        public async Task Calibrate(CalibrateMode mode)
        {
            await SendCommandAsync("CALIBRATE", mode.ToString());
            await ReceiveAnswerAsync("CALIBRATE");
        }

        /// <summary>
        /// Set whether the robot should be in learning mode or not.
        /// <param name="mode">Activate learning mode or not</param>
        /// </summary>
        public async Task SetLearningMode(bool mode)
        {
            await SendCommandAsync("SET_LEARNING_MODE", mode.ToString().ToUpper());
            await ReceiveAnswerAsync("SET_LEARNING_MODE");
        }

        /// <summary>
        /// Move joints to specified configuration.
        /// <param name="joints">The desired destination joint configuration</param>
        /// </summary>
        public async Task MoveJoints(RobotJoints joints)
        {
            await SendCommandAsync("MOVE_JOINTS", string.Join(',', joints));
            await ReceiveAnswerAsync("MOVE_JOINTS");
        }

        /// <summary>
        /// Move joints to specified pose.
        /// <param name="pose">The desired destination pose</param>
        /// </summary>
        public async Task MovePose(PoseObject pose)
        {
            await SendCommandAsync("MOVE_POSE", string.Join(',', pose));
            await ReceiveAnswerAsync("MOVE_POSE");
        }

        /// <summary>
        /// Shift the pose along one axis.
        /// <param name="axis">Which axis to shift</param>
        /// <param name="value">The amount to shift (meters or radians)</param>
        /// </summary>
        public async Task ShiftPose(RobotAxis axis, float value)
        {
            await SendCommandAsync("SHIFT_POSE", axis.ToString(), value.ToString());
            await ReceiveAnswerAsync("SHIFT_POSE");
        }

        /// <summary>
        /// Set the maximum arm velocity.false
        /// <param name="velocity">The maximum velocity in percent of maximum velocity.</param>
        /// </summary>
        public async Task SetArmMaxVelocity(int velocity)
        {
            await SendCommandAsync("SET_ARM_MAX_VELOCITY", velocity.ToString());
            await ReceiveAnswerAsync("SET_ARM_MAX_VELOCITY");
        }

        /// <summary>
        /// Enable or disable joystick control.false
        /// </summary>
        public async Task EnableJoystick(bool mode)
        {
            await SendCommandAsync("ENABLE_JOYSTICK", mode.ToString().ToUpper());
            await ReceiveAnswerAsync("ENABLE_JOYSTICK");
        }

        /// <summary>
        /// Configure a GPIO pin for input or output.
        /// </summary>
        public async Task SetPinMode(RobotPin pin, PinMode mode)
        {
            await SendCommandAsync("SET_PIN_MODE", pin.ToString(), mode.ToString());
            await ReceiveAnswerAsync("SET_PIN_MODE");
        }

        /// <summary>
        /// Write to a digital pin configured as output.
        /// </summary>
        public async Task DigitalWrite(RobotPin pin, DigitalState state)
        {
            await SendCommandAsync("DIGITAL_WRITE", pin.ToString(), state.ToString());
            await ReceiveAnswerAsync("DIGITAL_WRITE");
        }

        /// <summary>
        /// Read from a digital pin configured as input.
        /// </summary>
        public async Task<DigitalState> DigitalRead(RobotPin pin)
        {
            await SendCommandAsync("DIGITAL_READ", pin.ToString());
            var state = await ReceiveAnswerAsync("DIGITAL_READ");
            return (DigitalState)Enum.Parse(typeof(DigitalState), state);
        }

        /// <summary>
        /// Select which tool is connected to the robot.
        /// </summary>
        public async Task ChangeTool(RobotTool tool)
        {
            await SendCommandAsync("CHANGE_TOOL", tool.ToString());
            await ReceiveAnswerAsync("CHANGE_TOOL");
        }

        /// <summary>
        /// Open the gripper.
        /// <param name="gripper">Which gripper to open</param>
        /// <param name="speed">The speed to use. Must be between 0 and 1000, recommended values between 100 and 500.</param>
        /// </summary>
        public async Task OpenGripper(RobotTool gripper, int speed)
        {
            await SendCommandAsync("OPEN_GRIPPER", gripper.ToString(), speed.ToString());
            await ReceiveAnswerAsync("OPEN_GRIPPER");
        }

        /// <summary>
        /// Close the gripper.
        /// <param name="gripper">Which gripper to close</param>
        /// <param name="speed">The speed to use. Must be between 0 and 1000, recommended values between 100 and 500.</param>
        /// </summary>
        public async Task CloseGripper(RobotTool gripper, int speed)
        {
            await SendCommandAsync("CLOSE_GRIPPER", gripper.ToString(), speed.ToString());
            await ReceiveAnswerAsync("CLOSE_GRIPPER");
        }

        /// <summary>
        /// Pull air using the vacuum pump.
        /// <param name="vacuumPump">Must be VACUUM_PUMP_1. Only one type available for now.</param>
        /// </summary>
        public async Task PullAirVacuumPump(RobotTool vacuumPump)
        {
            await SendCommandAsync("PULL_AIR_VACUUM_PUMP", vacuumPump.ToString());
            await ReceiveAnswerAsync("PULL_AIR_VACUUM_PUMP");
        }

        /// <summary>
        /// Push air using the vacuum pump.
        /// <param name="vacuumPump">Must be VACUUM_PUMP_1. Only one type available for now.</param>
        /// </summary>
        public async Task PushAirVacuumPump(RobotTool vacuumPump)
        {
            await SendCommandAsync("PUSH_AIR_VACUUM_PUMP", vacuumPump.ToString());
            await ReceiveAnswerAsync("PUSH_AIR_VACUUM_PUMP");
        }

        /// <summary>
        /// Setup the electromagnet.
        /// <param name="tool">Must be ELECTROMAGNET_1. Only one type available for now.</param>
        /// <param name="pin">The pin to which the magnet is connected.</param>
        /// </summary>
        public async Task SetupElectromagnet(RobotTool tool, RobotPin pin)
        {
            await SendCommandAsync("SETUP_ELECTROMAGNET", tool.ToString(), pin.ToString());
            await ReceiveAnswerAsync("SETUP_ELECTROMAGNET");
        }

        /// <summary>
        /// Activate the electromagnet.
        /// <param name="tool">Must be ELECTROMAGNET_1. Only one type available for now.</param>
        /// <param name="pin">The pin to which the magnet is connected.</param>
        /// </summary>
        public async Task ActivateElectromagnet(RobotTool tool, RobotPin pin)
        {
            await SendCommandAsync("ACTIVATE_ELECTROMAGNET", tool.ToString(), pin.ToString());
            await ReceiveAnswerAsync("ACTIVATE_ELECTROMAGNET");
        }

        /// <summary>
        /// Deactivate the electromagnet.
        /// <param name="tool">Must be ELECTROMAGNET_1. Only one type available for now.</param>
        /// <param name="pin">The pin to which the magnet is connected.</param>
        /// </summary>
        public async Task DeactivateElectromagnet(RobotTool tool, RobotPin pin)
        {
            await SendCommandAsync("DEACTIVATE_ELECTROMAGNET", tool.ToString(), pin.ToString());
            await ReceiveAnswerAsync("DEACTIVATE_ELECTROMAGNET");
        }

        /// <summary>
        /// Get the current joint configuration.
        /// </summary>
        public async Task<RobotJoints> GetJoints()
        {
            await SendCommandAsync("GET_JOINTS");
            var joints = await ReceiveAnswerAsync("GET_JOINTS");
            return RobotJoints.Parse(joints);
        }

        /// <summary>
        /// Get the current pose.
        /// </summary>
        public async Task<PoseObject> GetPose()
        {
            await SendCommandAsync("GET_POSE");
            var pose = await ReceiveAnswerAsync("GET_POSE");
            return PoseObject.Parse(pose);
        }

        /// <summary>
        /// Get the current hardware status.
        /// </summary>
        public async Task<HardwareStatus> GetHardwareStatus()
        {
            await SendCommandAsync("GET_HARDWARE_STATUS");
            var status = await ReceiveAnswerAsync("GET_HARDWARE_STATUS");
            return HardwareStatus.Parse(status);
        }

        /// <summary>
        /// Get whether the robot is in learning mode.
        /// </summary>
        public async Task<bool> GetLearningMode()
        {
            await SendCommandAsync("GET_LEARNING_MODE");
            var mode = await ReceiveAnswerAsync("GET_LEARNING_MODE");
            return bool.Parse(mode);
        }

        /// <summary>
        /// Get the current state of the digital io pins.
        /// </summary>
        public async Task<DigitalPinObject[]> GetDigitalIoState()
        {
            await SendCommandAsync("GET_DIGITAL_IO_STATE");
            var state = await ReceiveAnswerAsync("GET_DIGITAL_IO_STATE");

            var regex = new Regex("\\[[0-9]+, '[^']*', [0-9]+, [0-9+]\\]");
            var matches = regex.Matches(state);

            return matches.Select(m => DigitalPinObject.Parse(m.Value)).ToArray();
        }
    }
}