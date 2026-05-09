using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Windows.Forms;
using Microsoft.Kinect;
using Emgu.CV;
using Emgu.CV.Structure;
using Emgu.CV.CvEnum;
using Emgu.CV.Util;
using System.Collections.Generic;

namespace KinectTracker
{
    class Program
    {
        static KinectSensor sensor;
        static PictureBox irPictureBox;
        static PictureBox depthPictureBox;
        static Form viewerForm;

        // IR Stream
        static byte[] colorPixels;
        static byte[] irPixels;
        const int IMG_WIDTH = 640;
        const int IMG_HEIGHT = 480;

        // Doble buffer IR
        static Bitmap irBufferA = null;
        static Bitmap irBufferB = null;
        static bool useIrA = true;

        const int THRESHOLD = 230; //Valor RGB para solo ver las esferas

        // Depth Stream
        static short[] depthData;
        static byte[] depthPixels;
        static Bitmap depthBufferA = null;
        static Bitmap depthBufferB = null;
        static bool useDepthA = true;
        const int MIN_DEPTH = 800;  //mm
        const int MAX_DEPTH = 4000; //mm
        const int SEARCH_RADIUS = 30; //pixeles para buscar depth válido alrededor del centroide

        // Detección de blobs
        static System.Collections.Generic.List<PointF> detectedCentroids = new System.Collections.Generic.List<PointF>();
        static readonly object centroidsLock = new object();
        static List<SkeletonPoint> detected3DPoints = new List<SkeletonPoint>(); //Posiciones 3D estimadas a partir de los centroides 2D y la profundidad

        // Parámetros de detección
        const int MIN_BLOB_AREA = 0;             //pixeles mínimos
        const int MAX_BLOB_AREA = 1000;          //pixeles máximos
        const double MIN_CIRCULARITY = 0.5;      //Esferas son redondas (~1.0), rechaza líneas/ruido
        const double MIN_ASPECT = 0.6;           //Aspect ratio mínimo (rechaza líneas alargadas)
        const double MAX_ASPECT = 1.7;           //Aspect ratio máximo

        [STAThread]
        static void Main(string[] args)
        {
            Console.WriteLine("=== Kinect IR + Depth Viewer ===\n");

            if (KinectSensor.KinectSensors.Count == 0)
            {
                Console.WriteLine("ERROR: No se detectó Kinect");
                Console.ReadLine();
                return;
            }

            sensor = KinectSensor.KinectSensors[0]; //Tomamos el primer Kinect aunque hubiesen varios
            Console.WriteLine($"Estado: {sensor.Status}"); //NotPowered indica problema de drivers si todo el hardware está en orden

            sensor.ColorStream.Enable(ColorImageFormat.InfraredResolution640x480Fps30);
            sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);

            //Buffers IR
            colorPixels = new byte[sensor.ColorStream.FramePixelDataLength];
            irPixels = new byte[IMG_WIDTH * IMG_HEIGHT * 4]; //4B/px

            irBufferA = new Bitmap(IMG_WIDTH, IMG_HEIGHT, PixelFormat.Format32bppRgb);
            irBufferB = new Bitmap(IMG_WIDTH, IMG_HEIGHT, PixelFormat.Format32bppRgb);

            //Buffers Depth
            depthData = new short[sensor.DepthStream.FramePixelDataLength];
            depthPixels = new byte[IMG_WIDTH * IMG_HEIGHT * 4];

            depthBufferA = new Bitmap(IMG_WIDTH, IMG_HEIGHT, PixelFormat.Format32bppRgb);
            depthBufferB = new Bitmap(IMG_WIDTH, IMG_HEIGHT, PixelFormat.Format32bppRgb);

            sensor.AllFramesReady += Sensor_AllFramesReady;

            try
            {
                sensor.Start();
                Console.WriteLine("Kinect iniciada - Streams IR + Depth activos");
                Console.WriteLine($"ID: {sensor.UniqueKinectId}");
                Console.WriteLine("\nAbriendo ventana de visualizacion...");
                Console.WriteLine("Cierra la ventana para terminar\n");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"ERROR: {ex.Message}");
                Console.ReadLine();
                return;
            }

            ShowViewerWindow(); //Abre la ventana, bloquea el programa hasta cerrarse

            if (sensor != null && sensor.IsRunning)
            {
                sensor.Stop();
            }

            Console.WriteLine("\nKinect detenida. Presiona ENTER");
            Console.ReadLine();
        }

        static void ShowViewerWindow()
        {
            Application.EnableVisualStyles();

            //Crea ventana
            viewerForm = new Form();
            viewerForm.Text = "Kinect IR + Depth Stream";
            viewerForm.Size = new Size(1320, 560);
            viewerForm.StartPosition = FormStartPosition.CenterScreen;

            //Layout 2 columnas: IR a la izquierda, Depth a la derecha
            TableLayoutPanel layout = new TableLayoutPanel();
            layout.Dock = DockStyle.Fill;
            layout.ColumnCount = 2;
            layout.RowCount = 2;
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 30)); //Labels arriba
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100)); //Imagenes abajo
            viewerForm.Controls.Add(layout);

            //Labels descriptivos
            Label irLabel = new Label();
            irLabel.Text = "IR Stream (con threshold)";
            irLabel.Dock = DockStyle.Fill;
            irLabel.TextAlign = ContentAlignment.MiddleCenter;
            irLabel.Font = new Font("Consolas", 10, FontStyle.Bold);
            layout.Controls.Add(irLabel, 0, 0);

            Label depthLabel = new Label();
            depthLabel.Text = "Depth Stream (cerca=blanco, lejos=negro)";
            depthLabel.Dock = DockStyle.Fill;
            depthLabel.TextAlign = ContentAlignment.MiddleCenter;
            depthLabel.Font = new Font("Consolas", 10, FontStyle.Bold);
            layout.Controls.Add(depthLabel, 1, 0);

            //Crea PictureBox IR (izquierda)
            irPictureBox = new PictureBox();
            irPictureBox.Dock = DockStyle.Fill;
            irPictureBox.SizeMode = PictureBoxSizeMode.Zoom;
            irPictureBox.BackColor = Color.Black;
            layout.Controls.Add(irPictureBox, 0, 1);

            //Crea PictureBox Depth (derecha)
            depthPictureBox = new PictureBox();
            depthPictureBox.Dock = DockStyle.Fill;
            depthPictureBox.SizeMode = PictureBoxSizeMode.Zoom;
            depthPictureBox.BackColor = Color.Black;
            layout.Controls.Add(depthPictureBox, 1, 1);

            Application.Run(viewerForm); //El bloqueo
        }

        //Dibujar círculo en imagen BGRA en memoria
        static void DrawCircle(byte[] pixels, int cx, int cy, int radius, byte r, byte g, byte b)
        {
            //Algoritmo de círculo (puntos del borde)
            for (int angle = 0; angle < 360; angle++)
            {
                double rad = angle * Math.PI / 180.0;
                int x = cx + (int)(radius * Math.Cos(rad));
                int y = cy + (int)(radius * Math.Sin(rad));

                if (x >= 0 && x < IMG_WIDTH && y >= 0 && y < IMG_HEIGHT)
                {
                    int idx = (y * IMG_WIDTH + x) * 4;
                    pixels[idx] = b;     //Blue
                    pixels[idx + 1] = g; //Green
                    pixels[idx + 2] = r; //Red
                                         //pixels[idx + 3] ya es 255
                }
            }
        }

        static void Sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            using (ColorImageFrame irFrame = e.OpenColorImageFrame())
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (irFrame == null || depthFrame == null) return;

                // === 1. Copiar datos raw ===
                irFrame.CopyPixelDataTo(colorPixels);
                depthFrame.CopyPixelDataTo(depthData);

                // === 2. Procesar IR ===

                for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++)
                {
                    int irValue = colorPixels[i * 2] | (colorPixels[i * 2 + 1] << 8); //Combina 2 bits
                    byte intensity = (byte)(irValue >> 8); //Dividir entre 256 para ir de 16 a 8 bits 

                    if (intensity < THRESHOLD)
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

                //Detección de blobs
                Mat irMat = new Mat(IMG_HEIGHT, IMG_WIDTH, DepthType.Cv8U, 1); //Matrix, equivalente al bitmap pero en OpenCV
                byte[] grayPixels = new byte[IMG_WIDTH * IMG_HEIGHT];

                //Extraemos solo el canal R (que es igual a B y G en escala de grises)
                for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++)
                {
                    grayPixels[i] = irPixels[i * 4 + 2]; //R channel
                }

                System.Runtime.InteropServices.Marshal.Copy(grayPixels, 0, irMat.DataPointer, grayPixels.Length); //Copiamos al mat

                //Morphology open
                Mat kernel = CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(3, 3), new Point(-1, -1));
                CvInvoke.MorphologyEx(irMat, irMat, MorphOp.Open, kernel, new Point(-1, -1), 1, BorderType.Default, new MCvScalar());

                //Contornos
                VectorOfVectorOfPoint contours = new VectorOfVectorOfPoint();
                Mat hierarchy = new Mat();
                CvInvoke.FindContours(irMat, contours, hierarchy, RetrType.External, ChainApproxMethod.ChainApproxSimple);

                //Calcular centroides y filtrar por área, circularidad y aspect ratio
                List<PointF> currentCentroids = new List<PointF>();
                List<SkeletonPoint> current3DPoints = new List<SkeletonPoint>();

                for (int i = 0; i < contours.Size; i++)
                {
                    using (VectorOfPoint contour = contours[i])
                    {
                        double area = CvInvoke.ContourArea(contour);

                        //Filtro 1: por área
                        if (area < MIN_BLOB_AREA || area > MAX_BLOB_AREA)
                            continue;

                        //Filtro 2: circularidad (esferas son redondas, ~1.0)
                        //Fórmula: 4π × área / perímetro²
                        double perimeter = CvInvoke.ArcLength(contour, true);
                        double circularity = perimeter > 0 ? 4 * Math.PI * area / (perimeter * perimeter) : 0;
                        if (circularity < MIN_CIRCULARITY) continue;

                        //Filtro 3: aspect ratio (esferas son aprox cuadradas en bounding box)
                        Rectangle bbox = CvInvoke.BoundingRectangle(contour);
                        double aspect = (double)bbox.Width / Math.Max(1, bbox.Height);
                        if (aspect < MIN_ASPECT || aspect > MAX_ASPECT) continue;

                        //Calcular centroide via momentos
                        Moments moments = CvInvoke.Moments(contour);
                        if (moments.M00 == 0) continue;

                        float cx = (float)(moments.M10 / moments.M00);
                        float cy = (float)(moments.M01 / moments.M00);

                        //Validar dentro de imagen
                        int xInt = (int)cx;
                        int yInt = (int)cy;
                        if (xInt < 0 || xInt >= IMG_WIDTH || yInt < 0 || yInt >= IMG_HEIGHT)
                            continue;

                        //Leer depth alrededor del centroide buscando el MÍNIMO válido
                        //(las esferas reflectivas saturan el sensor, pero el borde sí da depth válido)
                        int depthMm = int.MaxValue;

                        for (int dy = -SEARCH_RADIUS; dy <= SEARCH_RADIUS; dy++)
                        {
                            for (int dx = -SEARCH_RADIUS; dx <= SEARCH_RADIUS; dx++)
                            {
                                int sx = xInt + dx;
                                int sy = yInt + dy;
                                if (sx < 0 || sx >= IMG_WIDTH || sy < 0 || sy >= IMG_HEIGHT) continue;

                                int sIdx = sy * IMG_WIDTH + sx;
                                int rawSample = depthData[sIdx] & 0xFFFF; //Forzar interpretación sin signo
                                int sample = rawSample >> 3;

                                if (sample >= MIN_DEPTH && sample <= MAX_DEPTH && sample < depthMm)
                                {
                                    depthMm = sample;
                                }
                            }
                        }

                        //Validar depth válido
                        if (depthMm == int.MaxValue || depthMm < MIN_DEPTH || depthMm > MAX_DEPTH)
                            continue;

                        //Convertir a 3D
                        SkeletonPoint world = sensor.CoordinateMapper.MapDepthPointToSkeletonPoint(
                            DepthImageFormat.Resolution640x480Fps30,
                            new DepthImagePoint { X = xInt, Y = yInt, Depth = depthMm }
                        );

                        //Guardar 2D y 3D
                        currentCentroids.Add(new PointF(cx, cy));
                        current3DPoints.Add(world);
                    }
                }

                //Guardar centroides (thread-safe)
                lock (centroidsLock)
                {
                    detectedCentroids = currentCentroids;
                    detected3DPoints = current3DPoints;
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

                //Dibujar círculos sobre irPixels en las posiciones detectadas
                foreach (PointF centroid in currentCentroids)
                {
                    DrawCircle(irPixels, (int)centroid.X, (int)centroid.Y, 8, 0, 255, 0); //verde
                }

                //Cleanup
                irMat.Dispose();
                kernel.Dispose();
                contours.Dispose();
                hierarchy.Dispose();

                // Escribir en el buffer IR
                Bitmap writeBuffer = useIrA ? irBufferA : irBufferB;

                BitmapData bmpData = writeBuffer.LockBits(
                    new Rectangle(0, 0, IMG_WIDTH, IMG_HEIGHT),
                    ImageLockMode.WriteOnly,
                    writeBuffer.PixelFormat);

                System.Runtime.InteropServices.Marshal.Copy(irPixels, 0, bmpData.Scan0, irPixels.Length);
                writeBuffer.UnlockBits(bmpData);

                Bitmap displayBuffer = writeBuffer;
                useIrA = !useIrA;

                try
                {
                    irPictureBox.Invoke((MethodInvoker)delegate
                    {
                        irPictureBox.Image = displayBuffer;
                    });
                }
                catch { }

                // === 3. Procesar Depth ===

                for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++)
                {
                    short pixel = depthData[i];
                    int depthInMm = (pixel & 0xFFFF) >> 3;

                    byte intensity;
                    if (depthInMm < MIN_DEPTH || depthInMm > MAX_DEPTH)
                    {
                        intensity = 0;
                    }
                    else
                    {
                        double normalized = 1.0 - ((double)(depthInMm - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH));
                        intensity = (byte)(normalized * 255);
                    }

                    depthPixels[i * 4] = intensity;
                    depthPixels[i * 4 + 1] = intensity;
                    depthPixels[i * 4 + 2] = intensity;
                    depthPixels[i * 4 + 3] = 255;
                }

                Bitmap writeDepthBuffer = useDepthA ? depthBufferA : depthBufferB;

                BitmapData bmpDataDepth = writeDepthBuffer.LockBits(
                    new Rectangle(0, 0, IMG_WIDTH, IMG_HEIGHT),
                    ImageLockMode.WriteOnly,
                    writeDepthBuffer.PixelFormat);

                System.Runtime.InteropServices.Marshal.Copy(depthPixels, 0, bmpDataDepth.Scan0, depthPixels.Length);
                writeDepthBuffer.UnlockBits(bmpDataDepth);

                Bitmap displayDepthBuffer = writeDepthBuffer;
                useDepthA = !useDepthA;

                try
                {
                    depthPictureBox.Invoke((MethodInvoker)delegate
                    {
                        depthPictureBox.Image = displayDepthBuffer;
                    });
                }
                catch { }
            }
        }
    }
}