using System;
using System.Runtime.InteropServices;


namespace Dynaman
{
    class Program
    {

        [DllImport("winmm.dll", EntryPoint = "timeGetTime")]
        public static extern uint GetTime();

        static int Main()
        {  
            FloatingObject.SetWorkspece(-800.0f, 0, 500.0f, 800.0f, 1000.0f, 1570.0f);
            FloatingObject.SetSensorGeometry(35.1867f, -1242.32f, 1085.62f, 1.57172f, 1.60631f, 1.57601f);
            
            FloatingObject.AddDevice(992.5f, 270.0f, 1931.0f, 0.0f, (float)Math.PI, 0.0f);
            FloatingObject.AddDevice(992.5f, 790.0f, 1931.0f, 0.0f, (float)Math.PI, 0.0f);
            FloatingObject.AddDevice(542.5f, 10.0f, 1931.0f, 0.0f, (float)Math.PI, 0.0f);
            FloatingObject.AddDevice(542.5f, 530.0f, 1931.0f, 0.0f, (float)Math.PI, 0.0f);
            FloatingObject.AddDevice(542.5f, 1050.0f, 1931.0f, 0.0f, (float)Math.PI, 0.0f);
            FloatingObject.AddDevice(92.5f, 270.0f, 1931.0f, 0.0f, (float)Math.PI, 0.0f);
            FloatingObject.AddDevice(92.5f, 790.0f, 1931.0f, 0.0f, (float)Math.PI, 0.0f);
            FloatingObject.AddDevice(-357.5f, 10.0f, 1931.0f, 0.0f, (float)Math.PI, 0.0f);
            FloatingObject.AddDevice(-357.5f, 530.0f, 1931.0f, 0.0f, (float)Math.PI, 0.0f);
            FloatingObject.AddDevice(-357.5f, 1050.0f, 1931.0f, 0.0f, (float)Math.PI, 0.0f);
            FloatingObject.AddDevice(-807.5f, 270.0f, 1931.0f, 0.0f, (float)Math.PI, 0.0f);
            FloatingObject.AddDevice(-807.5f, 790.0f, 1931.0f, 0.0f, (float)Math.PI, 0.0f);
            Console.WriteLine("AUTDs added");

            var obj = new FloatingObject(0, 596.04f, 1331.0f, -0.0001f, 90.0f);
            Console.WriteLine("Floating object created.");
            float x, y, z;
            unsafe { obj.PositionTarget(&x, &y, &z); }
            Console.WriteLine("the target position is: {0}, {1}, {2}", x, y, z);
            obj.Register();
            Console.WriteLine("Object registered.");
            FloatingObject.StartControl();

            System.Threading.Thread.Sleep(20000);

            obj.MoveTo(500.0f, 596.6f, 1331.0f, 5.0f, GetTime() / 1000.0f);
            System.Threading.Thread.Sleep(10000);

            FloatingObject.StopControl();
            FloatingObject.FinalizeDynaman();

            return 0;
        }
    }
}