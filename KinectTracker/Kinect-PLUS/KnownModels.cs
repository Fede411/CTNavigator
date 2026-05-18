using System;
using System.Numerics;

namespace KinectTracker
{
	public static class KnownModels
	{

        public static RigidBodyModel CreateInstrument() { //Medidas teóricas del CAD, en el Anexo C están las medidas reales tomadas con calibre
			Vector3 pointA = new Vector3(-7.0760f, -60.181f, -21.219f);
            Vector3 pointB = new Vector3(17.93f, 4.884f, -36.219f);
            Vector3 pointC = new Vector3(22.155f, -106.199f, -36.219f);
            Vector3 pointD = new Vector3(-23.421f, -158.495f, -36.219f);

            Vector3[] insPoints =  {pointA, pointB, pointC, pointD};

            return new RigidBodyModel("Instrument", insPoints, Vector3.Zero);
        }

        public static RigidBodyModel CreateMarker() {
            throw new NotImplementedException("Aún falta por diseñar este modelo");
            //Vector3 pointA1 = new Vector3(0,0,0);
            //Vector3 pointB1 = new Vector3(0,0,0);
            //Vector3 pointC1 = new Vector3(0, 0, 0);

            //Vector3[] markPoints =  { pointA1, pointB1, pointC1};

            //return new RigidBodyModel("Reference Marker", markPoints, null);

        }
    }
}
