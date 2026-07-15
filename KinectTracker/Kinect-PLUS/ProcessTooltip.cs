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
        private int framesCoasted = 0;
        private const int MAX_COAST = 6;

        public int FramesCoastedTotal = 0;   // diagnostico
        public int FramesLostTotal = 0;      // diagnostico
        public int ClusterRejected = 0;      // detecciones aisladas descartadas antes del matcher
        public int PredictionRejected = 0;   // detecciones lejos de la prediccion
                                             
        private bool toolAudioState = false;      // estado sonoro actual: true = "siguiendo"
        private int framesSinceStateChange = 0;
        private const int AUDIO_HYSTERESIS = 5;   // frames que hay que aguantar antes de cambiar de estado


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
        public float LastPoseError { get; private set; } = 0f;
        public int[] MissingSphereCount { get; private set; } = new int[4];
        public float LastMatchResidual { get; private set; } = 0f;

        public ToolTipProcessor(KalmanPoseFilter kalmanFilter)
        {
            this.kalmanFilter = kalmanFilter;
        }

        public void Process(List<PointF> currentCentroids, List<Vector3> current3DPoints, //evalua esferas por frame, busca marcador, lo excluye y luego busca instrumento
            byte[] irPixels, RigidBodyModel instrumentModel, RigidBodyModel markerModel)
        {
            Vector3[] detectionsArr = current3DPoints.ToArray();

            //1. Buscar el marcador (3 esferas) entre todas las detecciones
            MarkerFound = false;
            MarkerFound = false;
            ToolFound = false;
            Vector3[] instrumentDetections = detectionsArr;

            MatchResult markerMatch = GeometryMatcher.Match(detectionsArr, markerModel, 10.0f, markerModel.SphereCount);

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
                    //DrawMarker(markerPos, markerMatch.Correspondences, currentCentroids, irPixels);

                    Console.WriteLine($"  MARKER! error={markerPose.error:F2}mm  pos=({markerPos.X:F1}, {markerPos.Y:F1}, {markerPos.Z:F1}) mm");

                    // Quitar las 3 detecciones del marcador para el match del instrumento
                    instrumentDetections = ExcludeIndices(detectionsArr, markerMatch.Correspondences);
                }
            }

            //Filtro clustering (distancia maxima del modelo = 107.28mm)
            instrumentDetections = ClusterFilter(instrumentDetections);

            //Filtro por prediccion: si venimos de una pose reciente, sabemos donde deberían caer las 4 esferas.
            instrumentDetections = PredictionFilter(instrumentDetections);

            //Buscar el instrumento (4 esferas) entre las detecciones restantes
            MatchResult matchResult = GeometryMatcher.Match(instrumentDetections, instrumentModel, 15.0f, instrumentModel.SphereCount);

            // DIAG: si hay 4+ detecciones pero el matcher no cierra, volcar las distancias entre las detecciones crudas para ver que geometria esta rechazando.
            // Las teoricas del modelo: 38.29 / 58.38 / 82.63 / 92.73 / 99.78 / 107.28
            if (!matchResult.Success && instrumentDetections.Length >= 4)
            {
                Console.Write("    [FALLO n>=4] dists crudas: ");
                for (int a = 0; a < instrumentDetections.Length; a++)
                    for (int b = a + 1; b < instrumentDetections.Length; b++)
                        Console.Write($"{Vector3.Distance(instrumentDetections[a], instrumentDetections[b]):F1} ");
                Console.WriteLine($" (n={instrumentDetections.Length})");
            }

            if (matchResult.Success)
            {
                // Construir el sub-modelo con SOLO las esferas que casaron (4 en match completo, 3 en parcial)
                // ModelSpheres = índices de las esferas del modelo que el matcher emparejó
                Vector3[] modelPts = new Vector3[matchResult.ModelSpheres.Length];
                for (int i = 0; i < modelPts.Length; i++)
                    modelPts[i] = instrumentModel.LocalSpheres[matchResult.ModelSpheres[i]];

                var pose = PoseEstimator.ComputePose(
                    modelPts, instrumentDetections, matchResult.Correspondences);
                //Console.WriteLine($"  [POSE] error={pose.error:F2}mm residual={matchResult.Residual:F2}");

                var toolTipLocal = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(
                    new double[] { 0, 0, 0 });
                var toolTipKinect = pose.R * toolTipLocal;

                Vector3 toolTip = new Vector3(
                    (float)toolTipKinect[0] + pose.t.X,
                    (float)toolTipKinect[1] + pose.t.Y,
                    (float)toolTipKinect[2] + pose.t.Z);

                if (pose.error < 12.0f)
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
                        Console.WriteLine($"  MATCH COMPLETO (4/4)! pose_error={pose.error:F2}mm  residual={matchResult.Residual:F2}mm");
                        LastMatchResidual = matchResult.Residual;
                        consecutivePartials = 0;
                        AcceptPose(pose, toolTip, currentCentroids, irPixels, instrumentModel);
                    }
                }
            }
            else if (instrumentDetections.Length >= 3 && lastProjectedSpheres != null &&
                     (DateTime.Now - lastMatchTime).TotalSeconds < 5)
            {
                TryPartialMatch(instrumentDetections, currentCentroids, irPixels, instrumentModel);
            }

            // Coasting: si este frame no dio pose (ni completa ni parcial), extrapolar
            // desde la ultima pose REAL usando la velocidad FILTRADA del Kalman (mm/s)
            if (!ToolFound && lastProjectedSpheres != null &&
                (DateTime.Now - lastMatchTime).TotalSeconds < 5)
            {
                framesCoasted++;

                if (framesCoasted <= MAX_COAST && kalmanFilter.IsInitialized)
                {
                    FramesCoastedTotal++;

                    double coastDt = (DateTime.Now - lastMatchTime).TotalSeconds;
                    Vector3 tipCoasted = lastToolTip + kalmanFilter.FilteredVelocity * (float)coastDt;

                    DrawCoastedTip(tipCoasted, irPixels);
                }
                else
                {
                    // Pasado el limite de coasting, solo el fantasma gris (pose vieja, no fiable)
                    FramesLostTotal++;
                    DrawGhostTip(lastToolTip, irPixels);
                }
                // Sonido de adquisicion/perdida. Estado efectivo = hay pose real O se esta coasteando
                // (el instrumento sigue "vivo" en pantalla). Perdido de verdad = ni pose ni coasting.
                bool trackingNow = ToolFound || (framesCoasted > 0 && framesCoasted <= MAX_COAST);
                UpdateToolAudio(trackingNow);
            }
        }

        // Emite un pitido solo cuando el estado se mantiene AUDIO_HYSTERESIS frames,
        // para no disparar sonidos en cada parpadeo del tracking.
        private void UpdateToolAudio(bool trackingNow)
        {
            if (trackingNow == toolAudioState)
            {
                framesSinceStateChange = 0;   // sigue igual, nada que hacer
                return;
            }

            framesSinceStateChange++;
            if (framesSinceStateChange < AUDIO_HYSTERESIS)
                return;   // el cambio aun no se ha sostenido lo suficiente

            // El cambio se ha mantenido: confirmamos transicion y sonamos
            toolAudioState = trackingNow;
            framesSinceStateChange = 0;

            try
            {
                if (trackingNow)
                    Console.Beep(1200, 80);   // agudo corto = instrumento adquirido
                else
                    Console.Beep(500, 150);   // grave mas largo = instrumento perdido
            }
            catch { /* Console.Beep puede fallar segun plataforma/salida; no es critico */ }
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

        // Descarta detecciones aisladas. MIN_NEIGHBORS = 2 y no 3, para que un grupo de 3 esferas (partial) sobreviva.
        private Vector3[] ClusterFilter(Vector3[] detections)
        {
            const float CLUSTER_RADIUS = 130.0f;   // algo mas que los 107.28mm del modelo, margen de ruido
            const int MIN_NEIGHBORS = 2;

            if (detections.Length <= MIN_NEIGHBORS)
                return detections;   // muy pocas para filtrar, se dejan pasar

            List<Vector3> kept = new List<Vector3>();

            for (int i = 0; i < detections.Length; i++)
            {
                int neighbors = 0;
                for (int j = 0; j < detections.Length; j++)
                {
                    if (i == j) continue;
                    if (Vector3.Distance(detections[i], detections[j]) <= CLUSTER_RADIUS)
                        neighbors++;
                }

                if (neighbors >= MIN_NEIGHBORS)
                    kept.Add(detections[i]);
                else
                    ClusterRejected++;
            }

            return kept.ToArray();
        }

        // Coherencia temporal: si hay una pose reciente, las 4 esferas deberian caer cerca de donde estaban. Nos quedamos solo con las detecciones proximas a esa prediccion.
        
        private Vector3[] PredictionFilter(Vector3[] detections)
        {
            const float PREDICTION_RADIUS = 60.0f;   // el salto de punta llega a ~50mm en movimientos rapidos
            const int MIN_KEPT = 3;                  // por debajo de 3 no hay ni partial: no filtramos

            // Sin pose reciente no hay prediccion en la que confiar: re-adquisicion desde cero
            if (lastProjectedSpheres == null ||
                (DateTime.Now - lastMatchTime).TotalSeconds >= 0.3)
                return detections;

            List<Vector3> kept = new List<Vector3>();

            foreach (var det in detections)
            {
                float nearest = float.MaxValue;
                foreach (var predicted in lastProjectedSpheres)
                {
                    float d = Vector3.Distance(det, predicted);
                    if (d < nearest) nearest = d;
                }

                if (nearest <= PREDICTION_RADIUS)
                    kept.Add(det);
            }

            // Si el filtro se ha pasado de agresivo, mejor no filtrar que perder el instrumento
            if (kept.Count < MIN_KEPT)
                return detections;

            PredictionRejected += (detections.Length - kept.Count);
            return kept.ToArray();
        }

        private void AcceptPose((Matrix<double> R, Vector3 t, float error) pose, //pose válida, expone error y dibuja tooltip
            Vector3 toolTip, List<PointF> currentCentroids, byte[] irPixels,
            RigidBodyModel instrumentModel)
        {
            ToolFound = true;
            ToolR = pose.R;
            ToolT = pose.t;
            LastPoseError = pose.error;

            framesCoasted = 0;   // pose real: se reinicia el coasting

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
            DrawToolTip(kalmanFilter.FilteredPosition, currentCentroids, irPixels);
        }

        private void TryPartialMatch(Vector3[] detections, List<PointF> currentCentroids, //pipeline con 2 o 3 esferas
            byte[] irPixels, RigidBodyModel instrumentModel)
        {
            int[] partialCorrespondences = new int[4];
            bool[] detectionUsed = new bool[detections.Length];
            int associationCount = 0;

            for (int i = 0; i < 4; i++)
            {
                partialCorrespondences[i] = -1;
                float bestDist = 45.0f;

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

            if (associationCount >= 2)
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

                for (int i = 0; i < 4; i++)
                    if (partialCorrespondences[i] < 0)
                    {
                        MissingSphereCount[i]++;

                        float nearest = float.MaxValue;
                        bool nearestTaken = false;
                        for (int j = 0; j < detections.Length; j++)
                        {
                            float d = Vector3.Distance(lastProjectedSpheres[i], detections[j]);
                            if (d < nearest)
                            {
                                nearest = d;
                                nearestTaken = detectionUsed[j];
                            }
                        }

                        string causa;
                        if (nearest < 10.0f && nearestTaken) causa = "FUSION (pegada y ya asignada)";
                        else if (nearestTaken) causa = "la mas cercana ya esta asignada, pero lejos";
                        else if (nearest < 60.0f) causa = "libre, fuera del radio de busqueda";
                        else causa = "no se ve / prediccion desviada";

                        Console.WriteLine($"    [DIAG] esfera {i} ausente. Deteccion mas cercana a {nearest:F1} mm -> {causa}");
                    }

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
                    if (jump < 35.0f && consecutivePartials < 50)
                    {
                        consecutivePartials++;
                        PartialMatchesSuccessful++;
                        AcceptPose(pose, toolTip, currentCentroids, irPixels, instrumentModel);

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

        private void DrawToolTip(Vector3 toolTip, List<PointF> currentCentroids, byte[] irPixels)
        {
            var (tx, ty) = StereoDepthMapper.Project2D(toolTip);

            foreach (var c in currentCentroids)
                ImageUtils.DrawLine(irPixels, (int)c.X, (int)c.Y, tx, ty, 255, 255, 0);

            ImageUtils.DrawCircle(irPixels, tx, ty, 5, 255, 0, 0);
        }

        public void DrawCoastedTip(Vector3 tip, byte[] irPixels)
        {
            var (tx, ty) = StereoDepthMapper.Project2D(tip);
            ImageUtils.DrawCircle(irPixels, tx, ty, 5, 255, 128, 0);  // naranja: distinguir de la punta roja real
        }

        private void DrawGhostTip(Vector3 tip, byte[] irPixels)
        {
            var (tx, ty) = StereoDepthMapper.Project2D(tip);
            ImageUtils.DrawCircle(irPixels, tx, ty, 5, 128, 128, 128);  // gris: pose vieja, no fiable
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
                //ImageUtils.DrawLine(irPixels, (int)c.X, (int)c.Y, m2D.X, m2D.Y, 0, 0, 255);
            }

            // Círculo cian en el centro, para diferenciarlo del tooltip (rojo)
            //ImageUtils.DrawCircle(irPixels, m2D.X, m2D.Y, 5, 0, 255, 255);
        }
    }
}