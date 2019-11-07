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

    public enum RobotAxis {
        X, Y, Z, ROLL, PITCH, YAW
    }
}