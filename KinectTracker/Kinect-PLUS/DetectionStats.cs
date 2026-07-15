using System;
using System.Numerics;

namespace KinectTracker
{

    public struct RejectionCounts
    {
        public int ByArea;
        public int ByCircularity;
        public int ByAspect;
        public int ByNoDepth;
        public int ByZSize;
    }

    public struct PoseQuality
    {
        public double ErrorSum;
        public int ErrorCount;
        public double ErrorMax;

        public Vector3 LastTip;
        public bool HasLastTip;
        public double JumpSum;
        public int JumpCount;
        public double JumpMax;
        public int JumpsOver20;

        public double ResidualSum;
        public int ResidualCount;
        public double ResidualMax;
    }

    public class DetectionStats
    {
        private int FramesProcessed;
        private int[] DetectionHistogram = new int[10];
        private RejectionCounts Rejections;
        private PoseQuality Pose;

        public void RegisterFrame() { FramesProcessed++; }

        public void RegisterMatchResidual(float residual)
        {
            Pose.ResidualSum += residual;
            Pose.ResidualCount++;
            if (residual > Pose.ResidualMax) Pose.ResidualMax = residual;
        }

        public void RegisterDetection(int n)
        {
            if (n < DetectionHistogram.Length)
                DetectionHistogram[n]++;
            else
                DetectionHistogram[DetectionHistogram.Length - 1]++;
        }

        public void AddRejections(RejectionCounts incoming)
        {
            Rejections.ByArea += incoming.ByArea;
            Rejections.ByCircularity += incoming.ByCircularity;
            Rejections.ByAspect += incoming.ByAspect;
            Rejections.ByNoDepth += incoming.ByNoDepth;
            Rejections.ByZSize += incoming.ByZSize;
        }

        // Llamar en AcceptPose: error de pose + posición de la punta este frame
        public void RegisterPose(float error, Vector3 tip)
        {
            Pose.ErrorSum += error;
            Pose.ErrorCount++;
            if (error > Pose.ErrorMax) Pose.ErrorMax = error;

            if (Pose.HasLastTip)
            {
                double jump = Vector3.Distance(tip, Pose.LastTip);
                Pose.JumpSum += jump;
                Pose.JumpCount++;
                if (jump > Pose.JumpMax) Pose.JumpMax = jump;
                if (jump > 20) Pose.JumpsOver20++;
            }
            Pose.LastTip = tip;
            Pose.HasLastTip = true;
        }

        public void StatsSummary(ToolTipProcessor ttp)
        {
            Console.WriteLine($"\nFrames procesados: {FramesProcessed}");
            for (int i = 0; i < DetectionHistogram.Length; i++)
            {
                double pct = FramesProcessed > 0 ? 100.0 * DetectionHistogram[i] / FramesProcessed : 0;
                Console.WriteLine($"  {i} detecciones: {DetectionHistogram[i]} ({pct:F1}%)");
            }

            // Un match completo puede salir de cualquier frame con AL MENOS 4 detecciones
            // (no solo de los que tienen exactamente 4), de ahi que antes salieran >100%
            int framesN4 = 0;
            for (int i = 4; i < DetectionHistogram.Length; i++)
                framesN4 += DetectionHistogram[i];

            double matchPctGlobal = FramesProcessed > 0 ? 100.0 * ttp.MatchesSuccessful / FramesProcessed : 0;
            double matchPctN4 = framesN4 > 0 ? 100.0 * ttp.MatchesSuccessful / framesN4 : 0;
            Console.WriteLine($"Matches exitosos: {ttp.MatchesSuccessful} ({matchPctGlobal:F1}% global, {matchPctN4:F1}% de n>=4)");
            Console.WriteLine($"Partial matches (3/4): {ttp.PartialMatchesSuccessful}");
            int totalPoses = ttp.MatchesSuccessful + ttp.PartialMatchesSuccessful;
            double posesPct = FramesProcessed > 0 ? 100.0 * totalPoses / FramesProcessed : 0;
            Console.WriteLine($"Total poses: {totalPoses} ({posesPct:F1}% global)");

            // Coasting: frames sin pose que se han extrapolado, y los que ya se dan por perdidos
            double coastPct = FramesProcessed > 0 ? 100.0 * ttp.FramesCoastedTotal / FramesProcessed : 0;
            double lostPct = FramesProcessed > 0 ? 100.0 * ttp.FramesLostTotal / FramesProcessed : 0;
            Console.WriteLine($"Frames coasteados: {ttp.FramesCoastedTotal} ({coastPct:F1}% global)");
            Console.WriteLine($"Frames perdidos (sin coasting): {ttp.FramesLostTotal} ({lostPct:F1}% global)");
            double coveragePct = FramesProcessed > 0 ? 100.0 * (totalPoses + ttp.FramesCoastedTotal) / FramesProcessed : 0;
            Console.WriteLine($"Cobertura (poses + coasting): {coveragePct:F1}% global");

            double markerPct = FramesProcessed > 0 ? 100.0 * ttp.MarkerMatchesSuccessful / FramesProcessed : 0;
            Console.WriteLine($"Marcador detectado: {ttp.MarkerMatchesSuccessful} ({markerPct:F1}% global)");

            Console.WriteLine($"Rechazos por área: {Rejections.ByArea}");
            Console.WriteLine($"Rechazos por circularidad: {Rejections.ByCircularity}");
            Console.WriteLine($"Rechazos por aspecto: {Rejections.ByAspect}");
            Console.WriteLine($"Rechazos por falta de profundidad: {Rejections.ByNoDepth}");
            Console.WriteLine($"Rechazos por tamaño Z incompatible: {Rejections.ByZSize}");
            Console.WriteLine($"Detecciones aisladas descartadas (clustering): {ttp.ClusterRejected}");
            Console.WriteLine($"Detecciones descartadas por prediccion: {ttp.PredictionRejected}");

            int totalRej = GeometryMatcher.RejectLeve + GeometryMatcher.RejectMedio + GeometryMatcher.RejectGrave;
            if (totalRej > 0)
            {
                Console.WriteLine($"\nRechazos matcher: leve(<30) {GeometryMatcher.RejectLeve} " +
                                  $"({100.0 * GeometryMatcher.RejectLeve / totalRej:F1}%) | " +
                                  $"medio {GeometryMatcher.RejectMedio} | " +
                                  $"grave(>=100) {GeometryMatcher.RejectGrave} " +
                                  $"({100.0 * GeometryMatcher.RejectGrave / totalRej:F1}%)");
            }

            // Calidad de pose
            if (Pose.ErrorCount > 0)
            {
                Console.WriteLine($"\nError de pose medio: {Pose.ErrorSum / Pose.ErrorCount:F2}mm (máx {Pose.ErrorMax:F2})");
                double jumpAvg = Pose.JumpCount > 0 ? Pose.JumpSum / Pose.JumpCount : 0;
                Console.WriteLine($"Salto de punta entre frames: medio {jumpAvg:F2}mm (máx {Pose.JumpMax:F2})");
                Console.WriteLine($"Saltos de punta >20mm: {Pose.JumpsOver20} de {Pose.JumpCount}");
                Console.WriteLine($"Esfera ausente en partials -> 0:{ttp.MissingSphereCount[0]} 1:{ttp.MissingSphereCount[1]} 2:{ttp.MissingSphereCount[2]} 3:{ttp.MissingSphereCount[3]}");
            }
            else
            {
                Console.WriteLine("\nSin poses aceptadas (no hay datos de calidad).");
            }

            if (Pose.ResidualCount > 0)
                Console.WriteLine($"Residual de match: medio {Pose.ResidualSum / Pose.ResidualCount:F2}mm (máx {Pose.ResidualMax:F2})");
        }
    }
}