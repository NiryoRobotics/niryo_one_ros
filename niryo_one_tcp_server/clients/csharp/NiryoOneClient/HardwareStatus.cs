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
using System.Globalization;
using System.Linq;

namespace NiryoOneClient
{
    /// <summary>
    /// A representation of the status of several system components in the Niryo One.
    /// </summary>
    public class HardwareStatus
    {
        /// <summary>
        /// The core temperature of the Raspberry Pi
        /// </summary>
        public int RpiTemperature;

        /// <summary>
        /// The hardware version
        /// </summary>
        public int HardwareVersion;

        /// <summary>
        /// Whether a connection to the robot is up
        /// </summary>
        public bool ConnectionUp;

        /// <summary>
        /// The current error message
        /// </summary>
        public string ErrorMessage;

        /// <summary>
        /// Whether the robot needs a calibartion to perform moves
        /// </summary>
        public int CalibrationNeeded;

        /// <summary>
        /// Whether a calibration is in progress
        /// </summary>
        public bool CalibrationInProgress;

        /// <summary>
        /// The names of the connected motors
        /// </summary>
        public string[] MotorNames;

        /// <summary>
        /// The model names of the connected motors
        /// </summary>
        public string[] MotorTypes;

        /// <summary>
        /// The temperatures in degrees celcius of the connected motors
        /// </summary>
        public int[] Temperatures;

        /// <summary>
        /// The voltages applied to the connected motors
        /// </summary>
        public decimal[] Voltages;

        /// <summary>
        /// The number of hardware errors on the respective motors
        /// </summary>
        public int[] HardwareErrors;
    }
}