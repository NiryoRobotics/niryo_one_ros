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

using System.Threading.Tasks;

namespace NiryoOneClient
{
    /// <summary>
    /// A connection object allowing sending commands to the tcp server on a Niryo One robotic arm
    /// </summary>
    public interface INiryoOneConnection
    {
        /// <summary>
        /// Request calibration.
        /// <param name="mode">Whether to request automatic or manual calibration</param>
        /// </summary>
        Task Calibrate(CalibrateMode mode);

        /// <summary>
        /// Set whether the robot should be in learning mode or not.
        /// <param name="mode">Activate learning mode or not</param>
        /// </summary>
        Task SetLearningMode(bool mode);

        /// <summary>
        /// Move joints to specified configuration.
        /// <param name="joints">The desired destination joint configuration</param>
        /// </summary>
        Task MoveJoints(RobotJoints joints);

        /// <summary>
        /// Move joints to specified pose.
        /// <param name="pose">The desired destination pose</param>
        /// </summary>
        Task MovePose(PoseObject pose);

        /// <summary>
        /// Shift the pose along one axis.
        /// <param name="axis">Which axis to shift</param>
        /// <param name="value">The amount to shift (meters or radians)</param>
        /// </summary>
        Task ShiftPose(RobotAxis axis, float value);

        /// <summary>
        /// Set the maximum arm velocity.false
        /// <param name="velocity">The maximum velocity in percent of maximum velocity.</param>
        /// </summary>
        Task SetArmMaxVelocity(int velocity);

        /// <summary>
        /// Enable or disable joystick control.false
        /// </summary>
        Task EnableJoystick(bool mode);

        /// <summary>
        /// Configure a GPIO pin for input or output.
        /// </summary>
        Task SetPinMode(RobotPin pin, PinMode mode);

        /// <summary>
        /// Write to a digital pin configured as output.
        /// </summary>
        Task DigitalWrite(RobotPin pin, DigitalState state);

        /// <summary>
        /// Read from a digital pin configured as input.
        /// </summary>
        Task<DigitalState> DigitalRead(RobotPin pin);

        /// <summary>
        /// Select which tool is connected to the robot.
        /// </summary>
        Task ChangeTool(RobotTool tool);

        /// <summary>
        /// Open the gripper.
        /// <param name="gripper">Which gripper to open</param>
        /// <param name="speed">The speed to use. Must be between 0 and 1000, recommended values between 100 and 500.</param>
        /// </summary>
        Task OpenGripper(RobotTool gripper, int speed);

        /// <summary>
        /// Close the gripper.
        /// <param name="gripper">Which gripper to close</param>
        /// <param name="speed">The speed to use. Must be between 0 and 1000, recommended values between 100 and 500.</param>
        /// </summary>
        Task CloseGripper(RobotTool gripper, int speed);

        /// <summary>
        /// Pull air using the vacuum pump.
        /// <param name="vacuumPump">Must be VACUUM_PUMP_1. Only one type available for now.</param>
        /// </summary>
        Task PullAirVacuumPump(RobotTool vacuumPump);

        /// <summary>
        /// Push air using the vacuum pump.
        /// <param name="vacuumPump">Must be VACUUM_PUMP_1. Only one type available for now.</param>
        /// </summary>
        Task PushAirVacuumPump(RobotTool vacuumPump);

        /// <summary>
        /// Setup the electromagnet.
        /// <param name="tool">Must be ELECTROMAGNET_1. Only one type available for now.</param>
        /// <param name="pin">The pin to which the magnet is connected.</param>
        /// </summary>
        Task SetupElectromagnet(RobotTool tool, RobotPin pin);

        /// <summary>
        /// Activate the electromagnet.
        /// <param name="tool">Must be ELECTROMAGNET_1. Only one type available for now.</param>
        /// <param name="pin">The pin to which the magnet is connected.</param>
        /// </summary>
        Task ActivateElectromagnet(RobotTool tool, RobotPin pin);

        /// <summary>
        /// Deactivate the electromagnet.
        /// <param name="tool">Must be ELECTROMAGNET_1. Only one type available for now.</param>
        /// <param name="pin">The pin to which the magnet is connected.</param>
        /// </summary>
        Task DeactivateElectromagnet(RobotTool tool, RobotPin pin);

        /// <summary>
        /// Get the current joint configuration.
        /// </summary>
        Task<RobotJoints> GetJoints();

        /// <summary>
        /// Get the current pose.
        /// </summary>
        Task<PoseObject> GetPose();

        /// <summary>
        /// Get the current hardware status.
        /// </summary>
        Task<HardwareStatus> GetHardwareStatus();

        /// <summary>
        /// Get whether the robot is in learning mode.
        /// </summary>
        Task<bool> GetLearningMode();

        /// <summary>
        /// Get the current state of the digital io pins.
        /// </summary>
        Task<DigitalPinObject[]> GetDigitalIOState();
    }
}