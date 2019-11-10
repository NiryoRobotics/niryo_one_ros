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
        private static string Strip_(string s, char prefix, char suffix)
        {
            if (!s.StartsWith(prefix))
                throw new ArgumentException();
            if (!s.EndsWith(suffix))
                throw new ArgumentException();
            return s.Substring(1, s.Length - 2);
        }

        private static string[] ParseStrings_(string s)
        {
            var regex = new System.Text.RegularExpressions.Regex(@"'[^']*'");
            return regex.Matches(s).Select(m => Strip_(m.Value, '\'', '\'')).ToArray();
        }

        private static T[] ParseNumbers_<T>(string s, Func<string, T> parser)
        {
            var regex = new System.Text.RegularExpressions.Regex(@"[0-9]+(\.[0-9]*)?");
            return regex.Matches(s).Select(m => parser(m.Value)).ToArray();
        }

        /// <summary>
        /// Parse a string representation of the hardware status in the format of the tcp server
        /// </summary>
        /// <param name="data">The string representation</param>
        /// <returns>A parsed object</returns>
        public static HardwareStatus Parse(string data)
        {
            var regex = new System.Text.RegularExpressions.Regex(@"((?:\[[^[\]]+\])|(?:\([^\)]+\))|True|False|\d+|'\w*')");
            var matches = regex.Matches(data);

            if (matches.Count != 11)
            {
                throw new NiryoOneException("Incorrect answer received, cannot understand received format.");
            }

            var rpiTemperature = int.Parse(matches[0].Value);
            var hardwareVersion = int.Parse(matches[1].Value);
            var connectionUp = bool.Parse(matches[2].Value);
            var errorMessage = Strip_(matches[3].Value, '\'', '\'');
            var calibrationNeeded = int.Parse(matches[4].Value);
            var calibrationInProgress = bool.Parse(matches[5].Value);

            var motorNames = ParseStrings_(matches[6].Value);
            var motorTypes = ParseStrings_(matches[7].Value);

            var temperatures = ParseNumbers_(matches[8].Value, int.Parse);
            var voltages = ParseNumbers_(matches[9].Value, x => decimal.Parse(x, CultureInfo.InvariantCulture));
            var hardwareErrors = ParseNumbers_(matches[10].Value, int.Parse);

            var hardwareStatus = new HardwareStatus()
            {
                RpiTemperature = rpiTemperature,
                HardwareVersion = hardwareVersion,
                ConnectionUp = connectionUp,
                ErrorMessage = errorMessage,
                CalibrationNeeded = calibrationNeeded,
                CalibrationInProgress = calibrationInProgress,
                MotorNames = motorNames,
                MotorTypes = motorTypes,
                Temperatures = temperatures,
                Voltages = voltages,
                HardwareErrors = hardwareErrors
            };
            return hardwareStatus;
        }

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
        /// The temperatures in degrees celsius of the connected motors
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
