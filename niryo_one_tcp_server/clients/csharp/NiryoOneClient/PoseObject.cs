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
using System.Linq;

namespace NiryoOneClient
{
    public class PoseObject : IEnumerable<float>
    {
        private float[] _j = new float[6];

        public PoseObject(float x, float y, float z, float roll, float pitch, float yaw)
        {
            _j = new[] { x, y, z, roll, pitch, yaw };
        }

        public PoseObject(float[] j)
        {
            if (j.Length != 6)
                throw new ArgumentException("Joints must be constructed from 6 values.", nameof(j));

            _j = j;
        }

        public static PoseObject Parse(string s)
        {
            return new PoseObject(s.Split(",").Select(float.Parse).ToArray());
        }

        public float X { get => _j[0]; set => _j[0] = value; }
        public float Y { get => _j[1]; set => _j[1] = value; }
        public float Z { get => _j[2]; set => _j[2] = value; }
        public float Roll { get => _j[3]; set => _j[3] = value; }
        public float Pitch { get => _j[4]; set => _j[4] = value; }
        public float Yaw { get => _j[5]; set => _j[5] = value; }

        public IEnumerator<float> GetEnumerator()
        {
            return ((IEnumerable<float>)_j).GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return _j.GetEnumerator();
        }
    }

    public enum RobotAxis
    {
        X,
        Y,
        Z,
        ROLL,
        PITCH,
        YAW
    }
}