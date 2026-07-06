using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;

namespace KinectTracker{
    public class KinectConfig
    {
        //Objetos de otras clases
        private KinectSensor sensorA;
        private KinectSensor sensorB;

        private ViewerWindow viewer;
        private BlobDetector blobDetector;
        private KalmanPoseFilter kalmanFilter;
        private ToolTipProcessor toolTipProcessor;
        private OpenIGTLinkServer igtlServer;
        private DetectionStats stats;

        private RigidBodyModel instrumentModel;
        private RigidBodyModel markerModel;

        private DateTime lastGoodPoseTime = DateTime.MinValue;
        

        //Streams arrays vacios para almacenar los datos del Kinect y luego convertirlos a bitmaps
        private byte[] colorPixelsA;
        private byte[] irPixelsA;
        private byte[] colorPixelsB;
        private byte[] irPixelsB;

        // Resultados
        public List<PointF> DetectedCentroids = new List<PointF>();
        List<Vector3> Detected3DPoints = new List<Vector3>();
        private readonly object centroidsLock = new object();

        private OperationMode mode;
        private float knownDistance;
        private BallProfiler profiler;

        public KinectConfig(ViewerWindow viewer, OperationMode mode, float knownDistance)
        {
            this.viewer = viewer;
            this.mode = mode;
            this.knownDistance = knownDistance;
        }

        public bool Start()
        {
            
            if (KinectSensor.KinectSensors.Count == 0)
            {
                Console.WriteLine("ERROR: No se detectó Kinect");
                Console.ReadLine();
                return false;
            }

            if (KinectSensor.KinectSensors.Count < 2 && KinectSensor.KinectSensors.Count != 0)
            {
                Console.WriteLine($"ERROR: se necesitan 2 Kinect, detectadas {KinectSensor.KinectSensors.Count}");
                Console.ReadLine();
                return false;
            }

            foreach (var s in KinectSensor.KinectSensors)
            {
                if (s.UniqueKinectId == Constants.KINECT_A_ID) sensorA = s;
                else if (s.UniqueKinectId == Constants.KINECT_B_ID) sensorB = s;
            }

            if (sensorA == null || sensorB == null)
            {
                Console.WriteLine("ERROR: no se reconocen las dos Kinect por su UniqueKinectId.");
                Console.WriteLine("IDs detectados:");
                foreach (var s in KinectSensor.KinectSensors)
                    Console.WriteLine($"   {s.UniqueKinectId}");
                Console.ReadLine();
                return false;
            }
            Console.WriteLine($"A: {sensorA.Status} / B: {sensorB.Status}");
            Console.WriteLine($"A: {sensorA.Status} / B: {sensorB.Status}");  //NotPowered indica problema de drivers si todo el hardware está en orden
            
            if (sensorA.Status != KinectStatus.Connected || sensorB.Status != KinectStatus.Connected)
            {
                Console.WriteLine("ERROR: alguna Kinect no está Connected. Reconecta y reinicia.");
                Console.ReadLine();
                return false;
            }

            //Objetos auxiliares
            blobDetector = new BlobDetector();
            kalmanFilter = new KalmanPoseFilter();
            toolTipProcessor = new ToolTipProcessor(kalmanFilter);
            igtlServer = new OpenIGTLinkServer(18944);
            igtlServer.Start();
            stats = new DetectionStats();

            // en Start(), donde creas el profiler:
            if (mode == OperationMode.Profiling || mode == OperationMode.Calibration)
                profiler = new BallProfiler(knownDistance);

            //Habilitamos los streams de IR con la resolución y framerate deseados
            sensorA.ColorStream.Enable(ColorImageFormat.InfraredResolution640x480Fps30);
            sensorB.ColorStream.Enable(ColorImageFormat.InfraredResolution640x480Fps30);
            

            //Buffers para almacenar los datos de los streams
            colorPixelsA = new byte[sensorA.ColorStream.FramePixelDataLength];
            irPixelsA = new byte[Constants.IMG_WIDTH * Constants.IMG_HEIGHT * 4]; //4B/px
            colorPixelsB = new byte[sensorB.ColorStream.FramePixelDataLength];
            irPixelsB = new byte[Constants.IMG_WIDTH * Constants.IMG_HEIGHT * 4]; //4B/px    


            sensorA.AllFramesReady += SensorA_FrameReady;
            sensorB.AllFramesReady += SensorB_FrameReady;

            try
            {
                sensorA.Start();
                sensorB.Start();

                Console.WriteLine("Dos Kinects iniciadas - Streams IR activos");
                instrumentModel = KnownModels.CreateInstrument();
                markerModel = KnownModels.CreateMarker();
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"ERROR: {ex.Message}");
                Console.ReadLine();
                return false;
            }
        }

        public void Stop()
        {
            if (sensorA != null && sensorA.IsRunning)
            {
                stats.StatsSummary(toolTipProcessor);

                if (mode == OperationMode.Profiling && profiler != null)
                {
                    string csvPath = $@"C:\ruta\que\elijas\profile_{knownDistance}.csv";
                    profiler.Summary(csvPath);
                }

                sensorA.Stop();
                igtlServer.Stop();
            }

            if (sensorB != null && sensorB.IsRunning)
            {
                stats.StatsSummary(toolTipProcessor);

                if (mode == OperationMode.Profiling && profiler != null)
                {
                    string csvPath = $@"C:\ruta\que\elijas\profile_{knownDistance}.csv";
                    profiler.Summary(csvPath);
                }

                sensorB.Stop();
                igtlServer.Stop();
            }
        }

        //Handlers principales: se llaman 30 veces por segundo con frames sincronizados
        private readonly object frameLock = new object();
        private long tsA = -1, tsB = -1;
        private const long SYNC_TOL_MS = 15;   // medio frame a 30fps

        private void SensorA_FrameReady(object sender, AllFramesReadyEventArgs e)
        {
            using (ColorImageFrame f = e.OpenColorImageFrame())
            {
                if (f == null) return;
                lock (frameLock)
                {
                    f.CopyPixelDataTo(colorPixelsA);
                    tsA = f.Timestamp;
                }
            }
            TryProcessPair();
        }

        private void SensorB_FrameReady(object sender, AllFramesReadyEventArgs e)
        {
            using (ColorImageFrame f = e.OpenColorImageFrame())
            {
                if (f == null) return;
                lock (frameLock)
                {
                    f.CopyPixelDataTo(colorPixelsB);
                    tsB = f.Timestamp;
                }
            }
            TryProcessPair();
        }

        private void TryProcessPair() //comprueba que hay frame de las dos cámaras y que sus timestamps difieren menos de 15ms
        {
            byte[] snapA, snapB;
            lock (frameLock)
            {
                if (tsA < 0 || tsB < 0) return;                 // falta algún frame
                //Console.WriteLine($"[PAIR] tsA={tsA} tsB={tsB} diff={Math.Abs(tsA - tsB)}");
                if (Math.Abs(tsA - tsB) > SYNC_TOL_MS) return;  // demasiado desfasados: espera al siguiente
                snapA = (byte[])colorPixelsA.Clone();
                snapB = (byte[])colorPixelsB.Clone();
            }
            ProcessStereoPair(snapA, snapB);
        }

        private void ProcessStereoPair(byte[] colorA, byte[] colorB) //pipeline por cada par de frames sincronizados
        {
            //MODO CALIBRACIÓN
            if (mode == OperationMode.Calibration)
            {
                // Stream en gris crudo para guiar la colocación del tablero (sin threshold ni blobs)
                byte[] grisA = ImageUtils.IRaGris(colorA);
                byte[] grisB = ImageUtils.IRaGris(colorB);
                viewer.UpdateIRImages(grisA, grisB);

                if (viewer.ConsumirCaptura())
                    profiler.GuardarParCalib(grisA, grisB, @"C:\calib");

                return;   // en calibración no se detecta ni triangula
            }

            stats.RegisterFrame();

            // Detección 2D en cada cámara (Process recortado: solo centroides)
            List<PointF> centroidsA = IRProcessor.Process(colorA, irPixelsA, blobDetector, out RejectionCounts rejA);
            List<PointF> centroidsB = IRProcessor.Process(colorB, irPixelsB, blobDetector, out RejectionCounts rejB);

            stats.AddRejections(rejA);   // por ahora solo las de A; ya decidiremos cómo contar las dos

            // Provisional: confirmar que ambas detectan antes de triangular
            //if (centroidsA.Count > 0 || centroidsB.Count > 0)
                //Console.WriteLine($"[{DateTime.Now:HH:mm:ss}] A:{centroidsA.Count} | B:{centroidsB.Count}");

            foreach (PointF c in centroidsA)
                ImageUtils.DrawCircle(irPixelsA, (int)c.X, (int)c.Y, 8, 0, 255, 0);
            foreach (PointF c in centroidsB)
                ImageUtils.DrawCircle(irPixelsB, (int)c.X, (int)c.Y, 8, 0, 255, 0);

            

            var stereo = new StereoDepthMapper();
            // stereo mejor como campo de la clase, no new por frame
            // --- Correspondencia A<->B uno-a-uno (asignación codiciosa por gap mínimo) ---
            var pares = new List<(int ia, int jb, float gap, Vector3 p)>();
            for (int i = 0; i < centroidsA.Count; i++)
                for (int j = 0; j < centroidsB.Count; j++)
                {
                    Vector3 p = stereo.Reconstruct(centroidsA[i], centroidsB[j], out float gap);
                    if (gap < 8f && p.Z > 500 && p.Z < 1500)
                        pares.Add((i, j, gap, p));
                }

            pares.Sort((x, y) => x.gap.CompareTo(y.gap));   // mejores parejas primero

            var usadosA = new HashSet<int>();
            var usadosB = new HashSet<int>();
            var current3DPoints = new List<Vector3>();
            var currentCentroids = new List<PointF>();   // 2D de A, alineado con los 3D (para dibujo)

            int duplicadosDescartados = 0;   // DEPURACIÓN
            foreach (var par in pares)
            {
                if (usadosA.Contains(par.ia) || usadosB.Contains(par.jb))
                {
                    duplicadosDescartados++;
                    continue;
                }
                usadosA.Add(par.ia);
                usadosB.Add(par.jb);
                current3DPoints.Add(par.p);
                currentCentroids.Add(centroidsA[par.ia]);
            }
            //if (current3DPoints.Count == 4)
            //{
            //    Console.Write("[NUBE n=4] dist: ");
            //    for (int i = 0; i < 4; i++)
            //        for (int j = i + 1; j < 4; j++)
            //            Console.Write($"{Vector3.Distance(current3DPoints[i], current3DPoints[j]):F1} ");
            //    Console.WriteLine();
            //}

            //if (current3DPoints.Count > 0)
            //{
            //Console.Write($"[3D] n={current3DPoints.Count}: ");
            //foreach (var p in current3DPoints)
            //Console.Write($"({p.X:F0},{p.Y:F0},{p.Z:F0}) ");
            //Console.WriteLine();
            //}

            stats.RegisterDetection(current3DPoints.Count);
            toolTipProcessor.Process(currentCentroids, current3DPoints, irPixelsA,
                                     instrumentModel, markerModel);

            DateTime now = DateTime.Now;

        if (toolTipProcessor.ToolFound)
            {
                lastGoodPoseTime = now;
                // FilteredPosition ya está actualizada por el Update de AcceptPose.
                // Aquí pintas/envías la filtrada.
                stats.RegisterPose(toolTipProcessor.LastPoseError, toolTipProcessor.ToolT);
                stats.RegisterMatchResidual(toolTipProcessor.LastMatchResidual);
                igtlServer.SendTransform("ToolToTracker", toolTipProcessor.ToolR, kalmanFilter.FilteredPosition);
            }

            if (toolTipProcessor.MarkerFound)
            {
                kalmanFilter.Predict(now);
                // Pintar la predicción sobre irPixelsA y (paso 5) enviarla a Slicer.
                //toolTipProcessor.DrawCoastedTip(kalmanFilter.FilteredPosition, irPixelsA);
                igtlServer.SendTransform("MarkerToTracker", toolTipProcessor.MarkerR, toolTipProcessor.MarkerT);
            }

            if (toolTipProcessor.ToolFound && toolTipProcessor.MarkerFound)
            {
                var Rm = toolTipProcessor.MarkerR;          // MarkerToTracker (rotación)
                var RmT = Rm.Transpose();                     // inversa de una rotación = traspuesta

                // diferencia de traslaciones (punta filtrada − centroide del marcador)
                Vector3 tip = kalmanFilter.FilteredPosition;
                Vector3 mkr = toolTipProcessor.MarkerT;
                var diff = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(
                    new double[] { tip.X - mkr.X, tip.Y - mkr.Y, tip.Z - mkr.Z });

                var tNewVec = RmT * diff;                     // t_new
                Vector3 tNew = new Vector3(
                    (float)tNewVec[0], (float)tNewVec[1], (float)tNewVec[2]);

                var Rnew = RmT * toolTipProcessor.ToolR;      // R_new

                igtlServer.SendTransform("ToolToMarker", Rnew, tNew);
            }

            viewer.UpdateIRImages(irPixelsA, irPixelsB);
        }

        private int parIdx = 0;
        private void GuardarPar(byte[] irA, byte[] irB)
        {
            if (parIdx >= 20) return;
            string dir = @"C:\calib";
            System.IO.Directory.CreateDirectory(dir);
            ImageUtils.GuardarPNG(irA, $@"{dir}\A_{parIdx:D2}.png");
            ImageUtils.GuardarPNG(irB, $@"{dir}\B_{parIdx:D2}.png");
            Console.WriteLine($"[GUARDADO] par {parIdx}");
            parIdx++;
        }
    }

}
