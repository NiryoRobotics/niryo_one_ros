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
            string server = args.FirstOrDefault() ?? "10.10.10.10";

            using (var niryoOneClient = new NiryoOneClient.NiryoOneClient(server))
            {
                Console.WriteLine($"Connecting to {server}:40001");
                var niryo = await niryoOneClient.Connect();
                Console.WriteLine($"Connected!");

                PoseObject initialPose = null;

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

                var digitalIoPins = await niryo.GetDigitalIoState();

                foreach (var pin in digitalIoPins)
                    Console.WriteLine($"Pin: {pin.PinId}, name: {pin.Name}, mode: {pin.Mode}, state: {pin.State}");

                await niryo.SetLearningMode(true);
            }
        }
    }
}
