using System;
using System.Runtime.InteropServices;

namespace Dynaman
{
    class Api {
        [DllImport("dynaman_dll.dll")]
        public static extern IntPtr InitializeDynaman();
        [DllImport("dynaman_dll.dll")]
        public static extern void DeleteDynaman(IntPtr dynamanPtr);
        [DllImport("dynaman_dll.dll")]
        public static extern void CloseDynaman(IntPtr dynamanPtr);
        [DllImport("dynaman_dll.dll")]
        public static extern void SetWorkSpace(IntPtr dynamanPtr, float x0, float y0, float z0, float x1, float y1, float z1);
        [DllImport("dynaman_dll.dll")]
        public static extern void SetSensorGeometry(IntPtr dynamanPtr, float x, float y, float z, float angle_z1, float angle_y2, float angle_z3);
        [DllImport("dynaman_dll.dll")]
        public static extern void AddDevice(IntPtr dynamanPtr, float x, float y, float z, float angle_z1, float angle_y2, float angle_z3);
        [DllImport("dynaman_dll.dll")]
        public static extern void StartControl(IntPtr dynamanPtr);
        [DllImport("dynaman_dll.dll")]
        public static extern IntPtr CreateFloatingObject(float x, float y, float z, float weight, float radius);
        [DllImport("dynaman_dll.dll")]
        public static extern void DeleteFloatingObject(IntPtr FloatingObjectPtr);
        [DllImport("dynaman_dll.dll")]
        public static extern void RegisterFloatingObject(IntPtr FloatingObjectPtr, IntPtr dynamanPtr);
        [DllImport("dynaman_dll.dll")]
        public static extern void MoveObjectFromTo(IntPtr FloatingObjectPtr, float x0, float y0, float z0,
            float x1, float y1, float z1, float timeToGo, float timeInit);
        [DllImport("dynaman_dll.dll")]
        public static extern void KeepObjectAt(IntPtr FloatingObjtctPtr, float x, float y, float z);
        [DllImport("dynaman_dll.dll")]
        public static extern unsafe void GetObjectPosition(IntPtr FloatingObjectPtr, float* x, float* y, float* z);
        [DllImport("dynaman_dll.dll")]
        public static extern unsafe void GetObjectPositionTarget(IntPtr FloatingObjectPtr, float* x, float* y, float* z);
    }

    class FloatingObject
    {
        private IntPtr floatingObjectPtr;
        private static IntPtr dynamanPtr;
        static FloatingObject()
        {
            dynamanPtr = Api.InitializeDynaman();
        }

        public static void AddDevice(float x, float y, float z, float angle1_z, float angle2_y, float angle3_z)
        {
            Api.AddDevice(dynamanPtr, x, y, z, angle1_z, angle2_y, angle3_z);
        }

        public static void SetSensorGeometry(float x, float y, float z, float angle1_z, float angle2_y, float angle3_z)
        {
            Api.SetSensorGeometry(dynamanPtr, x, y, z, angle1_z, angle2_y, angle3_z);
        }

        public static void SetWorkspece(float x0, float y0, float z0, float x1, float y1, float z1)
        {
            Api.SetWorkSpace(dynamanPtr, x0, y0, z0, x1, y1, z1);
        }

        public FloatingObject(float x, float y, float z, float weight, float radius)
        {
            Api.CreateFloatingObject(x, y, z, weight, radius);
        }

        public void Register()
        {
            Api.RegisterFloatingObject(floatingObjectPtr, dynamanPtr);
        }

        public static void StartControl()
        {
            Dynaman.Api.StartControl(dynamanPtr);
        }

        public static void StopControl()
        {
            Dynaman.Api.CloseDynaman(dynamanPtr);
        }

        public static void FinalizeDynaman()
        {
            Dynaman.Api.DeleteDynaman(dynamanPtr);
        }
        
        public void StayAt(float x, float y, float z)
        {
            Dynaman.Api.KeepObjectAt(floatingObjectPtr, x, y, z);
        }

        public unsafe void Position(float* px, float* py, float* pz)
        {
            Dynaman.Api.GetObjectPosition(floatingObjectPtr, px, py, pz);
        }

        public unsafe void PositionTarget(float* px, float* py, float* pz)
        {
            Api.GetObjectPositionTarget(floatingObjectPtr, px, py, pz);
        }

        public unsafe void MoveTo(float x, float y, float z, float timeToGo, float timeInit)
        {
            float x0, y0, z0;
            Position(&x0, &y0, &z0);
            Dynaman.Api.MoveObjectFromTo(floatingObjectPtr, x0, y0, z0, x, y, z, timeToGo, timeInit);
        }

    }
}