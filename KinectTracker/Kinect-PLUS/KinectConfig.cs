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

        //Variables de detección y estadísticas
        private int framesProcessed = 0;
        private int[] detectionHistogram = new int[10];
        private RigidBodyModel instrumentModel;
        private int matchesSuccessful = 0;
        private Vector3 lastToolTip = Vector3.Zero;
        private bool hasLastPose = false;

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
        public bool Initialize()
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
                double matchPctGlobal = framesProcessed > 0 ? 100.0 * matchesSuccessful / framesProcessed : 0;
                double matchPctN4 = framesN4 > 0 ? 100.0 * matchesSuccessful / framesN4 : 0;
                Console.WriteLine($"Matches exitosos: {matchesSuccessful} ({matchPctGlobal:F1}% global, {matchPctN4:F1}% de n=4)");

                sensor.Stop();
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
                ProcessDepth();

                //sw.Stop();
                //if (sw.ElapsedMilliseconds > 30)
                    //Console.WriteLine($"[SLOW] Frame: {sw.ElapsedMilliseconds}ms");
            }
        }

        private void ProcessIR() {
            for (int i = 0; i < Constants.IMG_WIDTH * Constants.IMG_HEIGHT; i++)
            {
                int irValue = colorPixels[i * 2] | (colorPixels[i * 2 + 1] << 8); //Combina 2 bytes
                byte intensity = (byte)(irValue >> 8); //Dividir entre 256 para ir de 16 a 8 bits 

                if (intensity < Constants.THRESHOLD)
                {
                    irPixels[i * 4] = 0; //negro
                    irPixels[i * 4 + 1] = 0;
                    irPixels[i * 4 + 2] = 0;
                }
                else
                {
                    irPixels[i * 4] = intensity; //Blue
                    irPixels[i * 4 + 1] = intensity; //Green
                    irPixels[i * 4 + 2] = intensity; //Red
                }

                irPixels[i * 4 + 3] = 255; //Alpha
            }

            //Detección de blob
            List<PointF> blobCentroids = blobDetector.DetectBlobs(irPixels);

            //Para cada centroide, calcular depth y 3D
            List<PointF> currentCentroids = new List<PointF>();
            List<Vector3> current3DPoints = new List<Vector3>();

            foreach (PointF centroid in blobCentroids)
            {
                int xInt = (int)centroid.X;
                int yInt = (int)centroid.Y;

                //Validar dentro de imagen
                if (xInt < 0 || xInt >= Constants.IMG_WIDTH || yInt < 0 || yInt >= Constants.IMG_HEIGHT)
                    continue;

                //Buscar depth válido y convertir a 3D
                int depthMm = depthMapper.FindValidDepth(xInt, yInt, depthData);
                if (depthMm < 0) continue;
                SkeletonPoint world = depthMapper.ConvertTo3D(xInt, yInt, depthMm);
                Vector3 worldMm = new Vector3(world.X * 1000f, world.Y * 1000f, world.Z * 1000f);

                //Guardar 2D y 3D
                currentCentroids.Add(centroid);
                current3DPoints.Add(worldMm);
            }

            int n = current3DPoints.Count;
            if (n < detectionHistogram.Length)
                detectionHistogram[n]++;
            else
                detectionHistogram[detectionHistogram.Length - 1]++;

            // Pasar la lista de Vector3 a array para el matcher
            Vector3[] detectionsArr = current3DPoints.ToArray();

            // Llamar al matcher con una tolerancia dada
            MatchResult matchResult = GeometryMatcher.Match(detectionsArr, instrumentModel, 10.0f); //7.5 cuando detecte bien n=4

            if (matchResult.Success)
            {

                // Calcular pose 6DOF
                var pose = PoseEstimator.ComputePose(
                    instrumentModel.LocalSpheres,
                    detectionsArr,
                    matchResult.Correspondences);

                // Posición del tooltip (origen del modelo) transformada al espacio del Kinect
                var toolTipLocal = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(
                    new double[] { 0, 0, 0 }); // tooltip es el origen del sistema local
                var toolTipKinect = pose.R * toolTipLocal;

                Vector3 toolTip = new Vector3(
                    (float)toolTipKinect[0] + pose.t.X,
                    (float)toolTipKinect[1] + pose.t.Y,
                    (float)toolTipKinect[2] + pose.t.Z);

                Matrix4x4 rotMatrix = new Matrix4x4(
                    (float)pose.R[0, 0], (float)pose.R[0, 1], (float)pose.R[0, 2], 0,
                    (float)pose.R[1, 0], (float)pose.R[1, 1], (float)pose.R[1, 2], 0,
                    (float)pose.R[2, 0], (float)pose.R[2, 1], (float)pose.R[2, 2], 0,
                    0, 0, 0, 1);

                Quaternion rotation = Quaternion.CreateFromRotationMatrix(rotMatrix);
                kalmanFilter.Update(toolTip, rotation, DateTime.Now);

                if (pose.error < 10.0f)
                {
                    // Filtro de consistencia temporal
                    bool poseValid = true;
                    if (hasLastPose)
                    {
                        float jump = Vector3.Distance(toolTip, lastToolTip);
                        if (jump > 50.0f)  // más de 50mm entre frames = espejado o error
                            poseValid = false;
                    }

                    if (poseValid)
                    {
                        matchesSuccessful++;
                        lastToolTip = toolTip;
                        hasLastPose = true;

                        Console.WriteLine($"  MATCH! residual={matchResult.Residual:F2}mm, pose_error={pose.error:F2}mm");
                        Console.WriteLine($"    ToolTip: ({toolTip.X:F1}, {toolTip.Y:F1}, {toolTip.Z:F1}) mm");

                        // Convertir tooltip 3D a 2D
                        var tt2D = depthMapper.ConvertTo2D(toolTip.X, toolTip.Y, toolTip.Z);

                        // Dibujar líneas desde cada esfera detectada al tooltip
                        for (int i = 0; i < currentCentroids.Count; i++)
                        {
                            var c = currentCentroids[i];
                            ImageUtils.DrawLine(irPixels, (int)c.X, (int)c.Y, tt2D.X, tt2D.Y, 255, 255, 0);
                        }

                        // Dibujar el tooltip como círculo rojo
                        ImageUtils.DrawCircle(irPixels, tt2D.X, tt2D.Y, 5, 255, 0, 0);
                    }
                }
            }

            //Guardar resultados (thread-safe)
            lock (centroidsLock)
            {
                DetectedCentroids = currentCentroids;
                Detected3DPoints = current3DPoints;
            }

            //Debug: mostrar coordenadas
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
            //Dibujar círculos sobre los centroides detectados
            foreach (PointF centroid in currentCentroids)
            {
                ImageUtils.DrawCircle(irPixels, (int)centroid.X, (int)centroid.Y, 8, 0, 255, 0);
            }
            viewer.UpdateIRImage(irPixels);

        }

        private void ProcessDepth() {
            for (int i = 0; i < Constants.IMG_WIDTH * Constants.IMG_HEIGHT; i++)
            {
                short pixel = depthData[i];
                int depthInMm = (pixel & 0xFFFF) >> 3;

                byte intensity;
                if (depthInMm < Constants.MIN_DEPTH || depthInMm > Constants.MAX_DEPTH)
                {
                    intensity = 0;
                }
                else
                {
                    double normalized = 1.0 - ((double)(depthInMm - Constants.MIN_DEPTH) / (Constants.MAX_DEPTH - Constants.MIN_DEPTH));
                    intensity = (byte)(normalized * 255);
                }

                depthPixels[i * 4] = intensity;
                depthPixels[i * 4 + 1] = intensity;
                depthPixels[i * 4 + 2] = intensity;
                depthPixels[i * 4 + 3] = 255;
            }

            viewer.UpdateDepthImage(depthPixels);
        }

    }
}
