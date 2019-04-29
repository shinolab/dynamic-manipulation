using System;
using System.Runtime.InteropServices;
#if ENABLE_MATHNET
using MathNet.Numerics.LinearAlgebra;
#endif

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
            floatingObjectPtr = Api.CreateFloatingObject(x, y, z, weight, radius);
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

#if ENABLE_MATHNET
        public static void AddDevice(Vector<float> pos, Vector<float> eulerAngles)
        {
            Api.AddDevice(dynamanPtr, pos[0], pos[1], pos[2], eulerAngles[0], eulerAngles[1], eulerAngles[2]);
        }

        public static void SetSensorGeometry(Vector<float> pos, Vector<float> eulerAngles)
        {
            Api.SetSensorGeometry(dynamanPtr, pos[0], pos[1], pos[2], eulerAngles[0], eulerAngles[1], eulerAngles[2]);
        }

        public static void SetWorkspece(Vector<float> corner0, Vector<float> corner1)
        {
            Api.SetWorkSpace(dynamanPtr, corner0[0], corner0[1], corner0[2], corner1[0], corner1[1], corner1[2]);
        }

        public FloatingObject(Vector<float> posTgt, float weight, float radius)
        {
            Api.CreateFloatingObject(posTgt[0], posTgt[1], posTgt[2], weight, radius);
        }

        public void StayAt(Vector<float> pos)
        {
            Dynaman.Api.KeepObjectAt(floatingObjectPtr, pos[0], pos[1], pos[2]);
        }

        public Vector<float> Position()
        {
            var pos = new float[3];
            unsafe
            {
                fixed (float* pPos = &pos[0])
                {
                    Dynaman.Api.GetObjectPosition(floatingObjectPtr, pPos, pPos+1, pPos+2);
                }
            }
            return Vector<float>.Build.DenseOfArray(pos);
        }

        public Vector<float> PositionTarget()
        {
            var pos = new float[3];
            unsafe
            {
                fixed (float* pPos = &pos[0])
                {
                    Dynaman.Api.GetObjectPositionTarget(floatingObjectPtr, pPos, pPos + 1, pPos + 2);
                }
            }
            return Vector<float>.Build.DenseOfArray(pos);
        }

        public unsafe void MoveTo(Vector<float> pos, float timeToGo, float timeInit)
        {
            float x0, y0, z0;
            PositionTarget(&x0, &y0, &z0);
            Dynaman.Api.MoveObjectFromTo(floatingObjectPtr, x0, y0, z0, pos[0], pos[1], pos[2], timeToGo, timeInit);
        }
#endif
    }
}