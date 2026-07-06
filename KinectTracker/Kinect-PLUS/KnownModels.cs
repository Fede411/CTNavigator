using System;
using System.Numerics;

namespace KinectTracker
{
	public static class KnownModels
	{

        public static RigidBodyModel CreateInstrument()
        {
            // T (tooltip) = origen local
            Vector3 pointA = new Vector3(-35.566f, 19.285f, 142.629f);
            Vector3 pointB = new Vector3(6.796f, -48.765f, 162.253f);
            Vector3 pointC = new Vector3(6.460f, 54.185f, 183.059f);
            Vector3 pointD = new Vector3(48.674f, -45.475f, 137.801f);

            Vector3[] insPoints = { pointA, pointB, pointC, pointD };
            return new RigidBodyModel("Instrument", insPoints, Vector3.Zero);
        }

        public static RigidBodyModel CreateMarker() {
            // new NotImplementedException("Aún falta por diseñar este modelo");
            Vector3 pointA1 = new Vector3(12.3193f, 3.206f, 0);
            Vector3 pointB1 = new Vector3(-13.6596f, 4.397f, 15);
            Vector3 pointC1 = new Vector3(1.3403f, -7.603f, -15);

            Vector3[] markPoints =  { pointA1, pointB1, pointC1};

            return new RigidBodyModel("Reference Marker", markPoints, null);

        }
    }
}
