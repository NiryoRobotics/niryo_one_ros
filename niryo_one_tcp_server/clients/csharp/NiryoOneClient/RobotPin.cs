using System;
using System.Collections;
using System.Collections.Generic;

namespace NiryoOneClient
{
    public enum RobotPin
    {
        GPIO_1A, GPIO_1B, GPIO_1C, GPIO_2A, GPIO_2B, GPIO_2C
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