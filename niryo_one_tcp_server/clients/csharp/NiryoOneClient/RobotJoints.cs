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
using System.Globalization;
using System.Linq;

namespace NiryoOneClient
{
    /// <summary>
    /// A representation of the position of the joints in a Niryo One robotic arm
    /// </summary>
    public class RobotJoints : IEnumerable<float>
    {
        private float[] _j = new float[6];

        /// <summary>
        /// Construct an object from the 6 joint values
        /// </summary>
        /// <param name="j1">The first joint rotation in radians</param>
        /// <param name="j2">The second joint rotation in radians</param>
        /// <param name="j3">The third joint rotation in radians</param>
        /// <param name="j4">The fourth joint rotation in radians</param>
        /// <param name="j5">The fifth joint rotation in radians</param>
        /// <param name="j6">The sixth joint rotation in radians</param>
        public RobotJoints(float j1, float j2, float j3, float j4, float j5, float j6)
        {
            _j = new[] { j1, j2, j3, j4, j5, j6 };
        }

        /// <summary>
        /// Construct an object from 6 joint values
        /// </summary>
        /// <param name="j">An array of the 6 joint rotations in radians</param>
        public RobotJoints(float[] j)
        {
            if (j.Length != 6)
                throw new ArgumentException("Joints must be constructed from 6 values.", nameof(j));

            _j = j;
        }


        /// <summary>
        /// Parse a string representation of a joint configuration in the format of the tcp server
        /// </summary>
        /// <param name="s">The string representation</param>
        /// <returns>A parsed object</returns> 
        public static RobotJoints Parse(string s)
        {
            return new RobotJoints(s.Split(",").Select(x => float.Parse(x, CultureInfo.InvariantCulture)).ToArray());
        }

        /// <summary>The value of the first joint, in radians</summary>
        public float J1 { get => _j[0]; set => _j[0] = value; }
        /// <summary>The value of the second joint, in radians</summary>
        public float J2 { get => _j[1]; set => _j[1] = value; }
        /// <summary>The value of the third joint, in radians</summary>
        public float J3 { get => _j[2]; set => _j[2] = value; }
        /// <summary>The value of the fourth joint, in radians</summary>
        public float J4 { get => _j[3]; set => _j[3] = value; }
        /// <summary>The value of the fifth joint, in radians</summary>
        public float J5 { get => _j[4]; set => _j[4] = value; }
        /// <summary>The value of the sixth joint, in radians</summary>
        public float J6 { get => _j[5]; set => _j[5] = value; }

        /// <summary>
        /// Returns an enumerator that iterates through the colleection
        /// </summary>
        public IEnumerator<float> GetEnumerator()
        {
            return ((IEnumerable<float>)_j).GetEnumerator();
        }

        /// <summary>
        /// Returns an enumerator that iterates through the colleection
        /// </summary>
        IEnumerator IEnumerable.GetEnumerator()
        {
            return _j.GetEnumerator();
        }
    }
}