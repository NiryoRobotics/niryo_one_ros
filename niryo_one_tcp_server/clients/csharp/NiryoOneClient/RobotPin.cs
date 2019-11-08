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
using System.Collections;
using System.Collections.Generic;

namespace NiryoOneClient
{
    public enum RobotPin
    {
        GPIO_1A,
        GPIO_1B,
        GPIO_1C,
        GPIO_2A,
        GPIO_2B,
        GPIO_2C
    }

    public enum PinMode
    {
        OUTPUT = 0, INPUT = 1
    }

    public enum DigitalState
    {
        LOW = 0, HIGH = 1
    }

    public class DigitalPinObject
    {
        public int PinId;
        public string Name;
        public PinMode Mode;
        public DigitalState State;

        public static DigitalPinObject Parse(string s)
        {
            if (!s.StartsWith('[') || !s.EndsWith(']'))
                throw new ArgumentException();

            var ss = s.Substring(1, s.Length - 2).Split(", ");

            return new DigitalPinObject
            {
                PinId = int.Parse(ss[0]),
                Name = ss[1].Trim().Substring(1, ss[1].Length - 2),
                Mode = (PinMode)int.Parse(ss[2]),
                State = (DigitalState)int.Parse(ss[3])
            };
        }
    }
}