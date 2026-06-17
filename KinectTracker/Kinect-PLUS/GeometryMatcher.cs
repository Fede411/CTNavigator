using System;
using System.Numerics;

namespace KinectTracker {

    public readonly struct MatchResult
    {
        public readonly bool Success;
        public readonly int[] Correspondences; //model.LocalSpheres[i] corresponde a detections[Correspondences[i]]
        public readonly float Residual;
        public readonly int Matches;

        public MatchResult(bool success, int[] correspondences, float residual, int matches)
        {
            this.Success = success;
            this.Correspondences = correspondences;
            this.Residual = residual;
            this.Matches = matches;
        }
    }

    public class GeometryMatcher
		{

        private static bool Combine(int[] grupo, int grupoLleno, int desde,
            Vector3[] detections, RigidBodyModel model, float tolerance, out MatchResult result)
                {
                    result = new MatchResult(false, new int[0], float.NaN, 0);

                    if (grupoLleno == model.SphereCount)
                    {
                        // Construir sub-array con las detecciones del grupo (tamaño variable según modelo)
                        Vector3[] subDetections = new Vector3[model.SphereCount];
                        for (int k = 0; k < model.SphereCount; k++)
                            subDetections[k] = detections[grupo[k]];

                        // Permutar el orden dentro del sub-array (Permute prueba todos los órdenes)
                        int[] perm = new int[model.SphereCount];
                        for (int k = 0; k < perm.Length; k++)
                            perm[k] = k;

                        if (Permute(perm, 0, subDetections, model, tolerance, out MatchResult subResult))
                        {
                            // subResult.Correspondences tiene índices 0..N-1 del sub-array.
                            // Traducir a índices originales de detections[] usando grupo[].
                            int[] traducido = new int[model.SphereCount];
                            for (int k = 0; k < model.SphereCount; k++)
                                traducido[k] = grupo[subResult.Correspondences[k]];

                            result = new MatchResult(true, traducido, subResult.Residual, model.SphereCount);
                            return true;
                        }

                        return false;
                    }

                    for (int i = desde; i < detections.Length; i++)
                    {
                        grupo[grupoLleno] = i;
                        if (Combine(grupo, grupoLleno + 1, i + 1, detections, model, tolerance, out result))
                            return true;
                    }

                    return false;
                }

        public static MatchResult Match(Vector3[] detections, RigidBodyModel model, float tolerance)
        {
            if (detections.Length < model.SphereCount)
            {
                return new MatchResult(false, new int[0], float.NaN, 0);
            }

            int[] grupo = new int[model.SphereCount];
            if (Combine(grupo, 0, 0, detections, model, tolerance, out MatchResult result))
            {
                return result;
            }

            return new MatchResult(false, new int[0], float.NaN, 0);
        }
        

        private static void Swap(int[] arr, int i, int j)
        {
            (arr[i], arr[j]) = (arr[j], arr[i]); // intercambia arr[i] con arr[j]
        }

        private static bool Permute(int[] perm, int start, Vector3[] detections, RigidBodyModel model, float tolerance, out MatchResult result)
        {
            result = new MatchResult(false, new int[0], float.NaN, 0);

            if (start == perm.Length)
            {
                if (CheckPermutation(perm, detections, model, tolerance, out float residual))
                {
                    // perm[i] = índice de la detección asignada a la esfera i del modelo (convención B)
                    result = new MatchResult(true, (int[])perm.Clone(), residual, model.SphereCount);
                    return true;
                }
                return false;
            }

            for (int i = start; i < perm.Length; i++)
            {
                Swap(perm, start, i);
                if (Permute(perm, start + 1, detections, model, tolerance, out result))
                    return true;
                Swap(perm, start, i); // backtrack
            }

            return false;
        }

        private static bool CheckPermutation(int[] perm, Vector3[] detections, RigidBodyModel model, float tolerance, out float residual) {
            float residualAccum = 0f;

            foreach (var sd in model.Distances) {
                Vector3 detA = detections[perm[sd.IndexA]];
                Vector3 detB = detections[perm[sd.IndexB]];

                float dist = Vector3.Distance(detA, detB);
                float diff = Math.Abs(dist - sd.DistanceMm);

                if (diff > tolerance)
                {
                    residual = 0f;
                    return false; // No match
                }
                residualAccum += diff * diff;

            }

            residual = (float) Math.Sqrt(residualAccum/model.Distances.Length); //RMS
            return true;
        }
    }
}
