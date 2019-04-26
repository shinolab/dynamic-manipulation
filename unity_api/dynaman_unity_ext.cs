#if UNITY_2019_1_OR_NEWER
using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

namespace Dynaman
{
    public static class GeometryTransform
    {
        public static Vector3 PosDynaman2Unity(Vector<float> pos)
        {
            return new Vector3(pos[0], pos[2], pos[1]);
        }

        public static Vector<float> PosUnity2Dynaman(Vector3 pos)
        {
            var r = Vector<float>.Build.DenseOfArray(new float[] { pos.x, pos.z, pos.y });
            return r;
        } 
    }
    public class FloatingObjectUnity : FloatingObject
    {
        public FloatingObjectUnity(Vector3 pos, float weight, float radius)
            : base(1000 * pos.x, 1000 * pos.z, 1000 * pos.y, weight, radius){}

        //the rotation order of AUTD in Unity is [y-z-y]
        public void AddDevice(Vector3 pos, Vector3 eulerAngles)
        {
            var angles = Vector<float>.Build.DenseOfArray(new float[] { eulerAngles.x, eulerAngles.y, eulerAngles.z });
            AddDevice(GeometryTransform.PosUnity2Dynaman(pos), angles);
        }

        public static void SetSensorGeometry(Vector3 pos, Vector3 eulerAngles)
        {
            var angles = Vector<float>.Build.DenseOfArray(new float[] { eulerAngles.x, eulerAngles.y, eulerAngles.z });
            SetSensorGeometry(GeometryTransform.PosUnity2Dynaman(pos), angles);
        }

        public static void SetWorkspece(Vector3 corner0, Vector3 corner1)
        {
            SetWorkspece(GeometryTransform.PosUnity2Dynaman(corner0), GeometryTransform.PosUnity2Dynaman(corner1));
        }

        public void StayAt(Vector3 pos)
        {
            StayAt(GeometryTransform.PosUnity2Dynaman(pos));
        }

        public Vector3 PositionUnity()
        {
            return GeometryTransform.PosDynaman2Unity(Position());
        }

        public Vector3 PositionTargetUnity()
        {
            return GeometryTransform.PosDynaman2Unity(PositionTarget());
        }

        public void MoveTo(Vector3 destination, float timeToGo, float timeInit)
        {
            MoveTo(destination, timeToGo, timeInit);
        }
    }

}

#endif