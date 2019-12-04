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

namespace NiryoOneClient
{
    /// <summary>
    /// An enumeration of the digital GPIO pins on the Niryo One robotic arm
    /// </summary>
    public enum RobotPin
    {
        /// <summary>The pin labeled 1A</summary>
        GPIO_1A,
        /// <summary>The pin labeled 1B</summary>
        GPIO_1B,
        /// <summary>The pin labeled 1C</summary>
        GPIO_1C,
        /// <summary>The pin labeled 2A</summary>
        GPIO_2A,
        /// <summary>The pin labeled 2B</summary>
        GPIO_2B,
        /// <summary>The pin labeled 2C</summary>
        GPIO_2C
    }

    /// <summary>
    /// The configuration mode of a GPIO pin - input or output
    /// </summary>
    public enum PinMode
    {
        /// <summary>
        /// The pin is configured as an output
        /// </summary>
        OUTPUT = 0,
        /// <summary>
        /// The pin is configured as an input
        /// </summary>
        INPUT = 1
    }

    /// <summary>
    /// The state of a pin - high or low
    /// </summary>
    public enum DigitalState
    {
        /// <summary>
        /// The pin is low
        /// </summary>
        LOW = 0, 
        /// <summary>
        /// The pin is high
        /// </summary>
        HIGH = 1
    }

    /// <summary>
    /// A representation of the state of a pin
    /// </summary>
    public class DigitalPinObject
    {
        /// <summary>
        /// The internal pin id
        /// </summary>
        public int PinId;
        /// <summary>
        /// The user-readable name of a pin
        /// </summary>
        public string Name;
        /// <summary>
        /// Whether the pin is configured for output or input
        /// </summary>
        public PinMode Mode;
        /// <summary>
        /// Whether the pin is low or high
        /// </summary>
        public DigitalState State;
    }
}