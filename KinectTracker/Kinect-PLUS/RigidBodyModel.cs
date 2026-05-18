using System;
using System.Collections.Generic;
using System.Numerics;

namespace KinectTracker {
    public class RigidBodyModel
    {
        public readonly string Name;
        public readonly Vector3[] LocalSpheres;
        public readonly Vector3? LocalTooltip;
        public readonly SphereDistance[] Distances;
        public int SphereCount => LocalSpheres.Length;

        public RigidBodyModel(string name, Vector3[] localSpheres, Vector3? localTooltip) //constructor
        {
            this.Name = name;
            this.LocalSpheres = localSpheres;
            this.LocalTooltip = localTooltip;
            var lista = new List<SphereDistance>();

            for (int i = 0; i < SphereCount; i++) { 
                for (int j = i+1; j < SphereCount; j++)
                {
                    lista.Add(new SphereDistance(i, j, Vector3.Distance(localSpheres[i],localSpheres[j])));
                    
                }             
            }

            Distances = lista.ToArray();
        }


    }

    public readonly struct SphereDistance
    {
        public readonly int IndexA;
        public readonly int IndexB;
        public readonly float DistanceMm;

        public SphereDistance(int indexA, int indexB, float distanceMm)
        {
            if (indexA == indexB)
            {
                throw new ArgumentException("Una esfera no conecta con sí misma.");
            }

            this.IndexA = indexA;
            this.IndexB = indexB;

            //Normalizamos el orden de los índices para que IndexA siempre sea el menor (y no tener dos objetos iguales con índices invertidos)
            if (indexA < indexB)
            {
                this.IndexA = indexA;
                this.IndexB = indexB;
            }
            else
            {
                this.IndexA = indexB;
                this.IndexB = indexA;
            }

            this.DistanceMm = distanceMm;


        }
    }
        
}


