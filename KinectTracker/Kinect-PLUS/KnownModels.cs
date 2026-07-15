using System;
using System.Numerics;

namespace KinectTracker
{
	public static class KnownModels
	{

        public static RigidBodyModel CreateInstrument()
        {
            // T (tooltip) = origen local
            Vector3 pointA = new Vector3(-45.226f, 21.337f, 160.149f);
            Vector3 pointB = new Vector3(44.870f, 37.027f, 176.869f);
            Vector3 pointC = new Vector3(-17.633f, -49.408f, 194.446f);
            Vector3 pointD = new Vector3(-24.659f, -33.217f, 163.205f);

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
