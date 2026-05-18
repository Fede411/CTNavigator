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
        public static MatchResult Match(Vector3[] detections, RigidBodyModel model, float tolerance)
        {
            if (detections.Length != model.SphereCount)
            {
                //Console.WriteLine("Number of detections must match the number of spheres in the model.");
                return new MatchResult(false, new int[0], float.NaN, 0);
            }

            int[] perm = new int[model.SphereCount];
            for (int i = 0; i < perm.Length; i++)
            {
                perm[i] = i;
            }

            if (Permute(perm, 0, detections, model, tolerance, out MatchResult result))
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
