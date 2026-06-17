using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;

namespace KinectTracker{
    public class KinectConfig
    {
        //Objetos de otras clases
        private KinectSensor sensor;
        private ViewerWindow viewer;
        private DepthMapper depthMapper;
        private BlobDetector blobDetector;
        private KalmanPoseFilter kalmanFilter;
        private ToolTipProcessor toolTipProcessor;
        private OpenIGTLinkServer igtlServer;
        private DetectionStats stats;

        private RigidBodyModel instrumentModel;
        private RigidBodyModel markerModel;


        //Streams arrays vacios para almacenar los datos del Kinect y luego convertirlos a bitmaps
        private byte[] colorPixels;
        private byte[] irPixels;
        private short[] depthData;
        private byte[] depthPixels;

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

            sensor = KinectSensor.KinectSensors[0]; //Tomamos el primer Kinect aunque hubiesen varios
            Console.WriteLine($"Estado: {sensor.Status}"); //NotPowered indica problema de drivers si todo el hardware está en orden

            //Objetos auxiliares
            depthMapper = new DepthMapper(sensor);
            blobDetector = new BlobDetector();
            kalmanFilter = new KalmanPoseFilter();
            toolTipProcessor = new ToolTipProcessor(kalmanFilter);
            igtlServer = new OpenIGTLinkServer(18944);
            igtlServer.Start();
            stats = new DetectionStats();

            if (mode == OperationMode.Profiling)
                profiler = new BallProfiler(knownDistance);

            //Habilitamos los streams de IR y Depth con la resolución y framerate deseados
            sensor.ColorStream.Enable(ColorImageFormat.InfraredResolution640x480Fps30);
            sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);

            //Buffers para almacenar los datos de los streams
            colorPixels = new byte[sensor.ColorStream.FramePixelDataLength];
            irPixels = new byte[Constants.IMG_WIDTH * Constants.IMG_HEIGHT * 4]; //4B/px          
            depthData = new short[sensor.DepthStream.FramePixelDataLength];
            depthPixels = new byte[Constants.IMG_WIDTH * Constants.IMG_HEIGHT * 4];

            sensor.AllFramesReady += Sensor_AllFramesReady;

            try
            {
                sensor.Start();
                Console.WriteLine("Kinect iniciada - Streams IR + Depth activos");
                Console.WriteLine($"ID: {sensor.UniqueKinectId}");
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
            if (sensor != null && sensor.IsRunning)
            {
                stats.StatsSummary(toolTipProcessor);

                if (mode == OperationMode.Profiling && profiler != null)
                {
                    string csvPath = $@"C:\ruta\que\elijas\profile_{knownDistance}.csv";
                    profiler.Summary(csvPath);
                }

                sensor.Stop();
                igtlServer.Stop();
            }
        }

        //Handler principal: se llama 30 veces por segundo con frames sincronizados
        private void Sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            using (ColorImageFrame irFrame = e.OpenColorImageFrame())
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (irFrame == null || depthFrame == null) return;

                stats.RegisterFrame();

                //var sw = System.Diagnostics.Stopwatch.StartNew();

                //Copia los datos a los buffers en memoria
                irFrame.CopyPixelDataTo(colorPixels);
                depthFrame.CopyPixelDataTo(depthData);

                ProcessIR();
                DepthProcessor.Process(depthData, depthPixels, viewer);

                //sw.Stop();
                //if (sw.ElapsedMilliseconds > 30)
                //Console.WriteLine($"[SLOW] Frame: {sw.ElapsedMilliseconds}ms");
            }
        }

        private void ProcessIR()
        {
            var (currentCentroids, current3DPoints) = IRProcessor.Process(
                colorPixels, irPixels, depthData, blobDetector, depthMapper,
                    out RejectionCounts rejections, out float survivorRadius);

            stats.AddRejections(rejections);
            int n = current3DPoints.Count;
            stats.RegisterDetection(n);

            if (mode == OperationMode.Profiling)
            {
                if (current3DPoints.Count == 1)
                {
                    Vector3 p = current3DPoints[0];
                    profiler.Observe(survivorRadius, p.X, p.Y, p.Z);
                }
                // en perfilado NO se llama al matcher
            }
            else
            {
                // Matching y poses
                toolTipProcessor.Process(currentCentroids, current3DPoints,
                    irPixels, instrumentModel, markerModel, depthMapper);

                if (toolTipProcessor.MarkerFound)
                    igtlServer.SendTransform("MarkerToTracker", toolTipProcessor.MarkerR, toolTipProcessor.MarkerT);

                if (toolTipProcessor.ToolFound)
                    igtlServer.SendTransform("ToolToTracker", toolTipProcessor.ToolR, toolTipProcessor.ToolT);

                // Guardar resultados
                lock (centroidsLock)
                {
                    DetectedCentroids = currentCentroids;
                    Detected3DPoints = current3DPoints;
                }

                // Debug
                if (currentCentroids.Count > 0)
                {
                    Console.WriteLine($"\n[{DateTime.Now:HH:mm:ss}] Detectados {currentCentroids.Count} blobs:");
                    for (int i = 0; i < currentCentroids.Count; i++)
                    {
                        var c = currentCentroids[i];
                        var p = current3DPoints[i];
                        Console.WriteLine($"  2D: ({c.X:F1}, {c.Y:F1})  3D: ({p.X:F0}, {p.Y:F0}, {p.Z:F0}) mm");
                    }
                }

                foreach (PointF centroid in currentCentroids)
                {
                    ImageUtils.DrawCircle(irPixels, (int)centroid.X, (int)centroid.Y, 8, 0, 255, 0);
                }
                viewer.UpdateIRImage(irPixels);
            }

        }
    }

}
