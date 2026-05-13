using System;
using System.Collections.Generic;
using System.Drawing;
using Microsoft.Kinect;

namespace KinectTracker{
	public class KinectConfig
	{
        //Objetos
        private KinectSensor sensor;
        private ViewerWindow viewer;
        private DepthMapper depthMapper;
        private BlobDetector blobDetector;

        //Streams arrays vacios para almacenar los datos del Kinect y luego convertirlos a bitmaps
        private byte[] colorPixels;
        private byte[] irPixels;
        private short[] depthData;
        private byte[] depthPixels;

        // Resultados
        public List<PointF> DetectedCentroids = new List<PointF>();
        public List<SkeletonPoint> Detected3DPoints = new List<SkeletonPoint>();
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
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"ERROR: {ex.Message}");
                Console.ReadLine();
                return false;
            }
        }

        public void Stop() { //Para detener el Kinect al cerrar la aplicación
            if (sensor != null && sensor.IsRunning)
            {
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
            List<SkeletonPoint> current3DPoints = new List<SkeletonPoint>();

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

                //Guardar 2D y 3D
                currentCentroids.Add(centroid);
                current3DPoints.Add(world);
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
                    Console.WriteLine($"  2D: ({c.X:F1}, {c.Y:F1})  3D: ({p.X * 1000:F0}, {p.Y * 1000:F0}, {p.Z * 1000:F0}) mm");
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
