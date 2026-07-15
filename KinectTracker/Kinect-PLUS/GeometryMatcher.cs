using System;
using System.Collections.Generic;
using System.Numerics;

namespace KinectTracker
{

    public readonly struct MatchResult
    {
        public readonly bool Success;
        public readonly int[] Correspondences; //model.LocalSpheres[i] corresponde a detections[Correspondences[i]]
        public readonly float Residual;
        public readonly int Matches;
        public readonly int[] ModelSpheres;

        public MatchResult(bool success, int[] correspondences, float residual, int matches, int[] modelSpheres)
        {
            this.Success = success;
            this.Correspondences = correspondences;
            this.Residual = residual;
            this.Matches = matches;
            this.ModelSpheres = modelSpheres;
        }
    }

    public class GeometryMatcher
    {
        public static int RejectLeve = 0, RejectMedio = 0, RejectGrave = 0;

        // Explora TODOS los grupos de detecciones y se queda con el de menor residual.
        // Antes cortaba en el primer grupo que cuadraba: con ruido en escena, un fantasma que
        // "casi" cuadra podia ganar a las esferas reales solo por explorarse antes.
        private static bool Combine(int[] grupo, int grupoLleno, int desde,
    Vector3[] detections, int[] modelSubset, SphereDistance[] distLocal, float tolerance, out MatchResult result)
        {
            result = new MatchResult(false, new int[0], float.NaN, 0, new int[0]);
            float bestResidual = float.MaxValue;

            CombineRec(grupo, grupoLleno, desde, detections, modelSubset, distLocal, tolerance,
                       ref result, ref bestResidual);

            return result.Success;
        }

        private static void CombineRec(int[] grupo, int grupoLleno, int desde,
            Vector3[] detections, int[] modelSubset, SphereDistance[] distLocal, float tolerance,
            ref MatchResult best, ref float bestResidual)
        {
            int k = modelSubset.Length;   // tamaño del subconjunto (4 = completo, 3 = trío)

            if (grupoLleno == k)
            {
                // Construir sub-array con las detecciones del grupo (tamaño variable según modelo)
                Vector3[] subDetections = new Vector3[k];
                for (int m = 0; m < k; m++)
                    subDetections[m] = detections[grupo[m]];

                // Permutar el orden dentro del sub-array (Permute prueba todos los órdenes)
                int[] perm = new int[k];
                for (int m = 0; m < perm.Length; m++)
                    perm[m] = m;

                if (Permute(perm, 0, subDetections, distLocal, tolerance, out MatchResult subResult))
                {
                    if (subResult.Residual < bestResidual)
                    {
                        bestResidual = subResult.Residual;

                        // subResult.Correspondences tiene índices 0..N-1 del sub-array.
                        // Traducir a índices originales de detections[] usando grupo[].
                        int[] traducido = new int[k];
                        for (int m = 0; m < k; m++)
                            traducido[m] = grupo[subResult.Correspondences[m]];

                        // modelSubset = qué esferas del modelo casaron (necesario para el tooltip en match parcial)
                        best = new MatchResult(true, traducido, subResult.Residual, k, (int[])modelSubset.Clone());
                    }
                }

                return;   // ya no se corta: se siguen probando el resto de grupos
            }

            for (int i = desde; i < detections.Length; i++)
            {
                grupo[grupoLleno] = i;
                CombineRec(grupo, grupoLleno + 1, i + 1, detections, modelSubset, distLocal, tolerance,
                           ref best, ref bestResidual);
            }
        }

        public static MatchResult Match(Vector3[] detections, RigidBodyModel model, float tolerance, int minSpheres)
        {
            MatchResult best = new MatchResult(false, new int[0], float.NaN, 0, new int[0]);
            float bestRms = float.MaxValue;

            for (int k = model.SphereCount; k >= minSpheres; k--)
            {
                if (detections.Length < k) continue;   // con k=4 y 3 detecciones, salta a k=3

                // Enumera qué k esferas del modelo (C(SphereCount,k) subconjuntos)
                foreach (int[] modelSubset in Subconjuntos(model.SphereCount, k))
                {
                    // distancias del modelo cuyos DOS extremos están en el subconjunto, reindexadas a 0..k-1
                    var lista = new List<SphereDistance>();
                    foreach (var sd in model.Distances)
                    {
                        int pa = Array.IndexOf(modelSubset, sd.IndexA);
                        int pb = Array.IndexOf(modelSubset, sd.IndexB);
                        if (pa >= 0 && pb >= 0)
                            lista.Add(new SphereDistance(pa, pb, sd.DistanceMm));
                    }
                    SphereDistance[] distLocal = lista.ToArray();

                    int[] grupo = new int[modelSubset.Length];
                    if (Combine(grupo, 0, 0, detections, modelSubset, distLocal, tolerance, out MatchResult r))
                    {
                        if (r.Residual < bestRms) { bestRms = r.Residual; best = r; }
                    }
                }

                // Si algún subconjunto de este k casó, no bajamos a menos esferas
                if (best.Success) return best;
            }
            return best;
        }


        private static void Swap(int[] arr, int i, int j) // intercambia arr[i] con arr[j]
        {
            (arr[i], arr[j]) = (arr[j], arr[i]);
        }

        // Explora TODAS las permutaciones y se queda con la de menor residual.
        // Antes cortaba en la primera que cuadraba dentro de tolerancia, asi que un
        // emparejamiento mediocre podia ganar solo por aparecer antes en el orden de exploracion.
        private static bool Permute(int[] perm, int start, Vector3[] detections, SphereDistance[] distLocal, float tolerance, out MatchResult result) // permuta el array de índices 0..k-1 y prueba cada orden
        {
            result = new MatchResult(false, new int[0], float.NaN, 0, new int[0]);
            float bestResidual = float.MaxValue;

            PermuteRec(perm, start, detections, distLocal, tolerance, ref result, ref bestResidual);

            return result.Success;
        }

        private static void PermuteRec(int[] perm, int start, Vector3[] detections, SphereDistance[] distLocal,
            float tolerance, ref MatchResult best, ref float bestResidual)
        {
            if (start == perm.Length)
            {
                if (CheckPermutation(perm, detections, distLocal, tolerance, out float residual))
                {
                    if (residual < bestResidual)
                    {
                        bestResidual = residual;
                        // perm[i] = índice de la detección asignada a la esfera i del modelo (convención B)
                        best = new MatchResult(true, (int[])perm.Clone(), residual, perm.Length, new int[0]);
                    }
                }
                return;   // ya no se corta: se siguen probando el resto de permutaciones
            }

            for (int i = start; i < perm.Length; i++)
            {
                Swap(perm, start, i);
                PermuteRec(perm, start + 1, detections, distLocal, tolerance, ref best, ref bestResidual);
                Swap(perm, start, i); // backtrack
            }
        }

        private static List<int[]> Subconjuntos(int n, int k)
        {
            var res = new List<int[]>();
            int[] actual = new int[k];
            void Rec(int desde, int lleno)
            {
                if (lleno == k) { res.Add((int[])actual.Clone()); return; }
                for (int i = desde; i < n; i++) { actual[lleno] = i; Rec(i + 1, lleno + 1); }
            }
            Rec(0, 0);
            return res;
        }

        private static bool CheckPermutation(int[] perm, Vector3[] detections, SphereDistance[] distLocal, float tolerance, out float residual)
        {
            float residualAccum = 0f;

            foreach (var sd in distLocal)
            {   // distancias internas del subconjunto, en índices locales 0..k-1
                Vector3 detA = detections[perm[sd.IndexA]];
                Vector3 detB = detections[perm[sd.IndexB]];

                float dist = Vector3.Distance(detA, detB);
                float diff = Math.Abs(dist - sd.DistanceMm);

                if (diff > tolerance)
                {
                    {
                        //Console.WriteLine($"  reject: modelo={sd.DistanceMm:F1} medido={dist:F1} diff={diff:F1}");
                        if (diff < 30) RejectLeve++;
                        else if (diff < 100) RejectMedio++;
                        else RejectGrave++;

                        residual = 0f;
                        return false;
                    }
                }
                residualAccum += diff * diff;

            }

            residual = (float)Math.Sqrt(residualAccum / distLocal.Length); //RMS
            return true;
        }
    }
}