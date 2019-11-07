using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace NiryoOneClient
{
    public class RobotJoints : IEnumerable<float>
    {
        private float[] _j = new float[6];

        public RobotJoints(float j1, float j2, float j3, float j4, float j5, float j6)
        {
            _j = new[] { j1, j2, j3, j4, j5, j6 };
        }

        public RobotJoints(float[] j)
        {
            if (j.Length != 6)
                throw new ArgumentException("Joints must be constructed from 6 values.", nameof(j));

            _j = j;
        }

        public static RobotJoints Parse(string s)
        {
            return new RobotJoints(s.Split(",").Select(float.Parse).ToArray());
        }

        public float J1 { get => _j[0]; set => _j[0] = value; }
        public float J2 { get => _j[1]; set => _j[1] = value; }
        public float J3 { get => _j[2]; set => _j[2] = value; }
        public float J4 { get => _j[3]; set => _j[3] = value; }
        public float J5 { get => _j[4]; set => _j[4] = value; }
        public float J6 { get => _j[5]; set => _j[5] = value; }

        public IEnumerator<float> GetEnumerator()
        {
            return ((IEnumerable<float>)_j).GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return _j.GetEnumerator();
        }
    }
}