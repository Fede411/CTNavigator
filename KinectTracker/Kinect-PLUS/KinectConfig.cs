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

        //Variables de detección y estadísticas
        private int framesProcessed = 0;
        private int[] detectionHistogram = new int[10];
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

        public KinectConfig(ViewerWindow viewer) {
            this.viewer = viewer;

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
                Console.WriteLine($"\nFrames procesados: {framesProcessed}");
                for (int i = 0; i < detectionHistogram.Length; i++)
                {
                    double pct = framesProcessed > 0 ? 100.0 * detectionHistogram[i] / framesProcessed : 0;
                    Console.WriteLine($"  {i} detecciones: {detectionHistogram[i]} ({pct:F1}%)");
                }

                int framesN4 = detectionHistogram[4];
                double matchPctGlobal = framesProcessed > 0 ? 100.0 * toolTipProcessor.MatchesSuccessful / framesProcessed : 0;
                double matchPctN4 = framesN4 > 0 ? 100.0 * toolTipProcessor.MatchesSuccessful / framesN4 : 0;
                Console.WriteLine($"Matches exitosos: {toolTipProcessor.MatchesSuccessful} ({matchPctGlobal:F1}% global, {matchPctN4:F1}% de n=4)");
                Console.WriteLine($"Partial matches (3/4): {toolTipProcessor.PartialMatchesSuccessful}");
                Console.WriteLine($"Total poses: {toolTipProcessor.MatchesSuccessful + toolTipProcessor.PartialMatchesSuccessful} ({100.0 * (toolTipProcessor.MatchesSuccessful + toolTipProcessor.PartialMatchesSuccessful) / framesProcessed:F1}% global)");
                Console.WriteLine($"Marcador detectado: {toolTipProcessor.MarkerMatchesSuccessful} ({100.0 * toolTipProcessor.MarkerMatchesSuccessful / framesProcessed:F1}% global)");

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

                framesProcessed++;

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
                colorPixels, irPixels, depthData, blobDetector, depthMapper);

            int n = current3DPoints.Count;
            if (n < detectionHistogram.Length)
                detectionHistogram[n]++;
            else
                detectionHistogram[detectionHistogram.Length - 1]++;

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
