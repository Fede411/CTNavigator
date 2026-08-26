using System;
using System.Numerics;

namespace KinectTracker
{//Almacenamiento puntos en 3D de las bolas del instrumento y marcador, con métodos para crear cada uno.
	public static class KnownModels
	{

        public static RigidBodyModel CreateInstrument() //v3.5, X invertidas para encontrar bien la punta
        {
            //T (tooltip) = origen local
            Vector3 pointA = new Vector3(45.225f, 21.337f, 126.125f);
            Vector3 pointB = new Vector3(-44.871f, 37.027f, 142.845f);
            Vector3 pointC = new Vector3(17.632f, -49.408f, 160.422f);
            Vector3 pointD = new Vector3(25.036f, -34.428f, 125.966f);

            Vector3[] insPoints = { pointA, pointB, pointC, pointD };
            return new RigidBodyModel("Instrument", insPoints, Vector3.Zero);
        }

        public static RigidBodyModel CreateMarker() //v2
        {
            //Origen = centroide de los 3 puntos
            Vector3 pointA = new Vector3(-65.913f, -10.930f, -8.333f);
            Vector3 pointB = new Vector3(49.087f, -10.930f, -8.333f);
            Vector3 pointC = new Vector3(16.827f, 21.860f, 16.667f);

            Vector3[] markerPoints = { pointA, pointB, pointC };
            return new RigidBodyModel("Marker", markerPoints, /* origen */ Vector3.Zero);
        }
    }
}
