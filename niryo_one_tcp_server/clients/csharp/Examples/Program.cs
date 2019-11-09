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
using System.Linq;
using System.Threading.Tasks;
using NiryoOneClient;

namespace Examples
{
    class Program
    {
        public static async Task Main(string[] args)
        {
            string server = "10.10.10.10";

            if (args.Length == 1 || args.Length == 7)
            {
                server = args.FirstOrDefault();
                args = args.Skip(1).ToArray();
            }

            using (var niryoOneClient = new NiryoOneClient.NiryoOneClient(server))
            {
                Console.WriteLine($"Connecting to {server}:40001");
                var niryo = await niryoOneClient.Connect();
                Console.WriteLine($"Connected!");

                PoseObject initialPose = null;
                if (args.Length == 6)
                    initialPose = new PoseObject(args.Select(f => float.Parse(f)).ToArray());

                Console.WriteLine("Calibrating...");
                await niryo.Calibrate(CalibrateMode.AUTO);
                Console.WriteLine("Done!");

                var pose = await niryo.GetPose();
                await niryo.MoveJoints(new RobotJoints(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
                await niryo.ShiftPose(RobotAxis.Y, 0.15f);

                if (initialPose != null)
                {
                    await niryo.MovePose(initialPose);
                }

                var digitalIOPins = await niryo.GetDigitalIOState();

                foreach (var pin in digitalIOPins)
                    Console.WriteLine($"Pin: {pin.PinId}, name: {pin.Name}, mode: {pin.Mode}, state: {pin.State}");

                await niryo.SetLearningMode(true);
            }
        }
    }
}
