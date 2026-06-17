using System;

namespace KinectTracker {

    public struct RejectionCounts
    {
        public int ByArea;
        public int ByCircularity;
        public int ByAspect;
        public int ByNoDepth;
        public int ByZSize;
    }

    public class DetectionStats
    {
        private int FramesProcessed;
        private int[] DetectionHistogram = new int[10];
        private RejectionCounts Rejections;

        public void RegisterFrame() { FramesProcessed++; }

        public void RegisterDetection(int n) {
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

        public void StatsSummary(ToolTipProcessor ttp)
        {
            Console.WriteLine($"\nFrames procesados: {FramesProcessed}");
            for (int i = 0; i < DetectionHistogram.Length; i++)
            {
                double pct = FramesProcessed > 0 ? 100.0 * DetectionHistogram[i] / FramesProcessed : 0;
                Console.WriteLine($"  {i} detecciones: {DetectionHistogram[i]} ({pct:F1}%)");
            }

            // Bloque de match (leído del matcher)
            int framesN4 = DetectionHistogram[4];
            double matchPctGlobal = FramesProcessed > 0 ? 100.0 * ttp.MatchesSuccessful / FramesProcessed : 0;
            double matchPctN4 = framesN4 > 0 ? 100.0 * ttp.MatchesSuccessful / framesN4 : 0;
            Console.WriteLine($"Matches exitosos: {ttp.MatchesSuccessful} ({matchPctGlobal:F1}% global, {matchPctN4:F1}% de n=4)");
            Console.WriteLine($"Partial matches (3/4): {ttp.PartialMatchesSuccessful}");
            int totalPoses = ttp.MatchesSuccessful + ttp.PartialMatchesSuccessful;
            double posesPct = FramesProcessed > 0 ? 100.0 * totalPoses / FramesProcessed : 0;
            Console.WriteLine($"Total poses: {totalPoses} ({posesPct:F1}% global)");
            double markerPct = FramesProcessed > 0 ? 100.0 * ttp.MarkerMatchesSuccessful / FramesProcessed : 0;
            Console.WriteLine($"Marcador detectado: {ttp.MarkerMatchesSuccessful} ({markerPct:F1}% global)");

            // Desglose de rechazos
            Console.WriteLine($"Rechazos por área: {Rejections.ByArea}");
            Console.WriteLine($"Rechazos por circularidad: {Rejections.ByCircularity}");
            Console.WriteLine($"Rechazos por aspecto: {Rejections.ByAspect}");
            Console.WriteLine($"Rechazos por falta de profundidad: {Rejections.ByNoDepth}");
            Console.WriteLine($"Rechazos por tamaño Z incompatible: {Rejections.ByZSize}");
        }
    }   

}


