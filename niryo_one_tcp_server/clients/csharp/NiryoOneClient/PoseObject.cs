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
    /// A representation of a cartesian position of the robotic arm in 6 dimensions.
    /// For a description of the coordinate system, see <a href="https://www.ros.org/reps/rep-0103.html">REP-0103</a>.
    /// </summary>
    public class PoseObject : IEnumerable<float>
    {
        private float[] _j = new float[6];

        /// <summary>
        /// Construct a PoseObject from explicit coordinates.
        /// For a description of the coordinate system, see <a href="https://www.ros.org/reps/rep-0103.html">REP-0103</a>.
        /// </summary>
        /// <param name="x">X position in meters</param>
        /// <param name="y">Y position in meters</param>
        /// <param name="z">Z position in meters</param>
        /// <param name="roll">Rotation about the fixed X axis in radians</param>
        /// <param name="pitch">Rotation about the fixed Y axis in radians</param>
        /// <param name="yaw">Rotation about the fixed Z axis in radians</param>
        public PoseObject(float x, float y, float z, float roll, float pitch, float yaw)
        {
            _j = new[] { x, y, z, roll, pitch, yaw };
        }

        /// <summary>
        /// Construct a PoseObject from a coordinate array.
        /// For a description of the coordinate system, see <a href="https://www.ros.org/reps/rep-0103.html">REP-0103</a>.
        /// </summary>
        /// <param name="j">An array of the coordinates, in order X, Y, Z, roll, pitch, yaw</param>
        public PoseObject(float[] j)
        {
            if (j.Length != 6)
                throw new ArgumentException("Joints must be constructed from 6 values.", nameof(j));

            _j = j;
        }

        /// <summary>
        /// The X position in meters.
        /// </summary>
        public float X { get => _j[0]; set => _j[0] = value; }

        /// <summary>
        /// The Y position in meters.
        /// </summary>
        public float Y { get => _j[1]; set => _j[1] = value; }

        /// <summary>
        /// The Z position in meters.
        /// </summary>
        public float Z { get => _j[2]; set => _j[2] = value; }

        /// <summary>
        /// The roll, i.e. the rotation around the fixed X axis in radians.
        /// </summary>
        public float Roll { get => _j[3]; set => _j[3] = value; }

        /// <summary>
        /// The pitch, i.e. the rotation around the fixed Y axis in radians.
        /// </summary>
        public float Pitch { get => _j[4]; set => _j[4] = value; }

        /// <summary>
        /// The yaw, i.e. the rotation around the fixed Z axis in radians.
        /// </summary>
        public float Yaw { get => _j[5]; set => _j[5] = value; }

        /// <summary>
        /// Returns an enumerator which iterates through the collection
        /// </summary>
        public IEnumerator<float> GetEnumerator()
        {
            return ((IEnumerable<float>)_j).GetEnumerator();
        }

        /// <summary>
        /// Returns an enumerator which iterates through the collection
        /// </summary>
        IEnumerator IEnumerable.GetEnumerator()
        {
            return _j.GetEnumerator();
        }
    }

    /// <summary>
    /// One of the 6 axes.
    /// For a description of the coordinate system, see <a href="https://www.ros.org/reps/rep-0103.html">REP-0103</a>.
    /// </summary>
    public enum RobotAxis
    {
        /// <summary>The X axis</summary>
        X,
        /// <summary>The Y axis</summary>
        Y,
        /// <summary>The Z axis</summary>
        Z,
        /// <summary>The roll, i.e. the rotation around the fixed X axis in radians.</summary>
        ROLL,
        /// <summary>The pitch, i.e. the rotation around the fixed Y axis in radians.</summary>
        PITCH,
        /// <summary>The yaw, i.e. the rotation around the fixed Z axis in radians.</summary>
        YAW
    }
}
