using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using MathNet.Numerics.LinearAlgebra;

namespace KinectTracker
{
    public class ToolTipProcessor
    {
        private Vector3 lastToolTip = Vector3.Zero;
        private bool hasLastPose = false;
        private DateTime lastMatchTime = DateTime.MinValue;
        private Vector3[] lastProjectedSpheres = null;
        private int consecutivePartials = 0;
        private KalmanPoseFilter kalmanFilter;

        public int MatchesSuccessful { get; private set; } = 0;
        public int PartialMatchesSuccessful { get; private set; } = 0;
        public int MarkerMatchesSuccessful { get; private set; } = 0;

        // Pose cruda del marcador (para enviar a Slicer)
        public bool MarkerFound { get; private set; } = false;
        public Matrix<double> MarkerR { get; private set; }
        public Vector3 MarkerT { get; private set; }
        public Vector3 MarkerPosition { get; private set; }
        public bool ToolFound { get; private set; } = false;
        public Matrix<double> ToolR { get; private set; }
        public Vector3 ToolT { get; private set; }

        public ToolTipProcessor(KalmanPoseFilter kalmanFilter)
        {
            this.kalmanFilter = kalmanFilter;
        }

        public void Process(List<PointF> currentCentroids, List<Vector3> current3DPoints,
            byte[] irPixels, RigidBodyModel instrumentModel, RigidBodyModel markerModel,
            DepthMapper depthMapper)
        {
            Vector3[] detectionsArr = current3DPoints.ToArray();

            //1. Buscar el marcador (3 esferas) entre todas las detecciones
            MarkerFound = false;
            MarkerFound = false;
            ToolFound = false;
            Vector3[] instrumentDetections = detectionsArr;

            MatchResult markerMatch = GeometryMatcher.Match(detectionsArr, markerModel, 10.0f);

            if (markerMatch.Success)
            {
                var markerPose = PoseEstimator.ComputePose(
                    markerModel.LocalSpheres, detectionsArr, markerMatch.Correspondences);

                if (markerPose.error < 10.0f)
                {
                    // El origen local del marcador es su centroide
                    var origin = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(
                        new double[] { 0, 0, 0 });
                    var mo = markerPose.R * origin;
                    Vector3 markerPos = new Vector3(
                        (float)mo[0] + markerPose.t.X,
                        (float)mo[1] + markerPose.t.Y,
                        (float)mo[2] + markerPose.t.Z);

                    MarkerFound = true;
                    MarkerMatchesSuccessful++;
                    MarkerR = markerPose.R;
                    MarkerT = markerPose.t;
                    MarkerPosition = markerPos;
                    DrawMarker(markerPos, markerMatch.Correspondences, currentCentroids, irPixels, depthMapper);

                    Console.WriteLine($"  MARKER! error={markerPose.error:F2}mm  pos=({markerPos.X:F1}, {markerPos.Y:F1}, {markerPos.Z:F1}) mm");

                    // Quitar las 3 detecciones del marcador para el match del instrumento
                    instrumentDetections = ExcludeIndices(detectionsArr, markerMatch.Correspondences);
                }
            }

            //2. Buscar el instrumento (4 esferas) entre las detecciones restantes
            MatchResult matchResult = GeometryMatcher.Match(instrumentDetections, instrumentModel, 10.0f);

            if (matchResult.Success)
            {
                var pose = PoseEstimator.ComputePose(
                    instrumentModel.LocalSpheres, instrumentDetections, matchResult.Correspondences);

                var toolTipLocal = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(
                    new double[] { 0, 0, 0 });
                var toolTipKinect = pose.R * toolTipLocal;

                Vector3 toolTip = new Vector3(
                    (float)toolTipKinect[0] + pose.t.X,
                    (float)toolTipKinect[1] + pose.t.Y,
                    (float)toolTipKinect[2] + pose.t.Z);

                if (pose.error < 10.0f)
                {
                    bool poseValid = true;
                    if (hasLastPose)
                    {
                        float jump = Vector3.Distance(toolTip, lastToolTip);
                        if (jump > 50.0f) poseValid = false;
                    }

                    if (poseValid)
                    {
                        MatchesSuccessful++;
                        consecutivePartials = 0;
                        AcceptPose(pose, toolTip, currentCentroids, irPixels, instrumentModel, depthMapper);
                    }
                }
            }
            else if (instrumentDetections.Length >= 3 && lastProjectedSpheres != null &&
                     (DateTime.Now - lastMatchTime).TotalSeconds < 0.5)
            {
                TryPartialMatch(instrumentDetections, currentCentroids, irPixels, instrumentModel, depthMapper);
            }
        }

        // Devuelve un nuevo array sin las detecciones cuyos índices están en toExclude
        private Vector3[] ExcludeIndices(Vector3[] detections, int[] toExclude)
        {
            var excluded = new HashSet<int>(toExclude);
            List<Vector3> result = new List<Vector3>();
            for (int i = 0; i < detections.Length; i++)
            {
                if (!excluded.Contains(i))
                    result.Add(detections[i]);
            }
            return result.ToArray();
        }

        private void AcceptPose((Matrix<double> R, Vector3 t, float error) pose,
            Vector3 toolTip, List<PointF> currentCentroids, byte[] irPixels,
            RigidBodyModel instrumentModel, DepthMapper depthMapper)
        {
            ToolFound = true;
            ToolR = pose.R;
            ToolT = pose.t;
            lastToolTip = toolTip;
            hasLastPose = true;
            lastMatchTime = DateTime.Now;

            Matrix4x4 rotMatrix = new Matrix4x4(

                (float)pose.R[0, 0], (float)pose.R[0, 1], (float)pose.R[0, 2], 0,
                (float)pose.R[1, 0], (float)pose.R[1, 1], (float)pose.R[1, 2], 0,
                (float)pose.R[2, 0], (float)pose.R[2, 1], (float)pose.R[2, 2], 0,
                0, 0, 0, 1);
            Quaternion rotation = Quaternion.CreateFromRotationMatrix(rotMatrix);
            kalmanFilter.Update(toolTip, rotation, DateTime.Now);

            UpdateProjections(pose, instrumentModel);
            DrawToolTip(toolTip, currentCentroids, irPixels, depthMapper);
        }

        private void TryPartialMatch(Vector3[] detections, List<PointF> currentCentroids,
            byte[] irPixels, RigidBodyModel instrumentModel, DepthMapper depthMapper)
        {
            int[] partialCorrespondences = new int[4];
            bool[] detectionUsed = new bool[detections.Length];
            int associationCount = 0;

            for (int i = 0; i < 4; i++)
            {
                partialCorrespondences[i] = -1;
                float bestDist = 30.0f;

                for (int j = 0; j < detections.Length; j++)
                {
                    if (detectionUsed[j]) continue;
                    float dist = Vector3.Distance(lastProjectedSpheres[i], detections[j]);
                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        partialCorrespondences[i] = j;
                    }
                }

                if (partialCorrespondences[i] >= 0)
                {
                    detectionUsed[partialCorrespondences[i]] = true;
                    associationCount++;
                }
            }

            if (associationCount >= 3)
            {
                List<Vector3> modelPts = new List<Vector3>();
                List<Vector3> detectedPts = new List<Vector3>();

                for (int i = 0; i < 4; i++)
                {
                    if (partialCorrespondences[i] >= 0)
                    {
                        modelPts.Add(instrumentModel.LocalSpheres[i]);
                        detectedPts.Add(detections[partialCorrespondences[i]]);
                    }
                }

                int[] directCorr = new int[modelPts.Count];
                for (int i = 0; i < modelPts.Count; i++)
                    directCorr[i] = i;

                var pose = PoseEstimator.ComputePose(
                    modelPts.ToArray(), detectedPts.ToArray(), directCorr);

                if (pose.error < 10.0f)
                {
                    var toolTipLocal = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(
                        new double[] { 0, 0, 0 });
                    var toolTipKinect = pose.R * toolTipLocal;

                    Vector3 toolTip = new Vector3(
                        (float)toolTipKinect[0] + pose.t.X,
                        (float)toolTipKinect[1] + pose.t.Y,
                        (float)toolTipKinect[2] + pose.t.Z);

                    float jump = Vector3.Distance(toolTip, lastToolTip);
                    if (jump < 50.0f && consecutivePartials < 5)
                    {
                        consecutivePartials++;
                        PartialMatchesSuccessful++;
                        AcceptPose(pose, toolTip, currentCentroids, irPixels, instrumentModel, depthMapper);

                        Console.WriteLine($"  PARTIAL MATCH ({associationCount}/4)! pose_error={pose.error:F2}mm");
                    }
                }
            }
        }

        private void UpdateProjections((Matrix<double> R, Vector3 t, float error) pose,
            RigidBodyModel instrumentModel)
        {
            lastProjectedSpheres = new Vector3[4];
            for (int i = 0; i < 4; i++)
            {
                var mp = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(
                    new double[] { instrumentModel.LocalSpheres[i].X,
                                  instrumentModel.LocalSpheres[i].Y,
                                  instrumentModel.LocalSpheres[i].Z });
                var rp = pose.R * mp;
                lastProjectedSpheres[i] = new Vector3(
                    (float)rp[0] + pose.t.X,
                    (float)rp[1] + pose.t.Y,
                    (float)rp[2] + pose.t.Z);
            }
        }

        private void DrawToolTip(Vector3 toolTip, List<PointF> currentCentroids,
            byte[] irPixels, DepthMapper depthMapper)
        {
            var tt2D = depthMapper.ConvertTo2D(toolTip.X, toolTip.Y, toolTip.Z);

            for (int i = 0; i < currentCentroids.Count; i++)
            {
                var c = currentCentroids[i];
                ImageUtils.DrawLine(irPixels, (int)c.X, (int)c.Y, tt2D.X, tt2D.Y, 255, 255, 0);
            }
            ImageUtils.DrawCircle(irPixels, tt2D.X, tt2D.Y, 5, 255, 0, 0);
        }

        private void DrawMarker(Vector3 markerPos, int[] correspondences,
            List<PointF> currentCentroids, byte[] irPixels, DepthMapper depthMapper)
                {
                    // Proyectamos el centro 3D del marcador a píxeles de la imagen IR,
                    // igual que hacemos con la punta del instrumento
                    var m2D = depthMapper.ConvertTo2D(markerPos.X, markerPos.Y, markerPos.Z);

                    // Una línea azul desde cada esfera del marcador hasta su centro.
                    // correspondences[i_modelo] = i_detección, así que cada índice
                    // apunta a un centroide real de currentCentroids
                    foreach (int idx in correspondences)
                    {
                        if (idx < 0 || idx >= currentCentroids.Count) continue;
                        var c = currentCentroids[idx];
                        ImageUtils.DrawLine(irPixels, (int)c.X, (int)c.Y, m2D.X, m2D.Y, 0, 0, 255);
                    }

                    // Círculo cian en el centro, para diferenciarlo del tooltip (rojo)
                    ImageUtils.DrawCircle(irPixels, m2D.X, m2D.Y, 5, 0, 255, 255);
                }
    }
}