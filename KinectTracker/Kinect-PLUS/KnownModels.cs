using System;
using System.Numerics;

namespace KinectTracker
{
	public static class KnownModels
	{

        public static RigidBodyModel CreateInstrument()
        {
            // T (tooltip) = origen local. B, C, D conservan las coords del CAD (casan con calibre <0,6mm).
            // A reubicada por trilateración a las distancias de calibre (Anexo C): A-B 71,616 | A-C 54,74 | A-D 100,762
            Vector3 pointA = new Vector3(-6.290f, -61.800f, -21.503f);   // corregida (sesgo heat-set ~1,8mm)
            Vector3 pointB = new Vector3(17.930f, 4.884f, -36.219f);
            Vector3 pointC = new Vector3(22.155f, -106.199f, -36.219f);
            Vector3 pointD = new Vector3(-23.421f, -158.495f, -36.219f);

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
