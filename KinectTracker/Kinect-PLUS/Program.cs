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

        // Detección de blobs
        static System.Collections.Generic.List<PointF> detectedCentroids = new System.Collections.Generic.List<PointF>();
        static readonly object centroidsLock = new object();

        // Parámetros de detección
        const int MIN_BLOB_AREA = 30;    // pixeles mínimos
        const int MAX_BLOB_AREA = 1000;   // pixeles máximos

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

            //sensor.ColorFrameReady += Sensor_ColorFrameReady;
            //sensor.DepthFrameReady += Sensor_DepthFrameReady;
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

        static void Sensor_ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame frame = e.OpenColorImageFrame())
            {
                if (frame == null) return; //Verificamos que hay frame y ventana

                if (irPictureBox == null || irPictureBox.IsDisposed || !irPictureBox.IsHandleCreated)
                    return;

                frame.CopyPixelDataTo(colorPixels); //Cada pixel son 2 bytes (uint16, valores 0-65535)
                //Pasamos de IR (16 bits) a algo visualizable (8 bits)

                
            }
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
                // (todo lo que tenías en Sensor_ColorFrameReady DESPUÉS del CopyPixelDataTo)
                // - Bucle threshold
                // - Detección de blobs
                // - Dibujar círculos
                // - Actualizar irPictureBox

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
                //Esta línea da bastante problema
                byte[] grayPixels = new byte[IMG_WIDTH * IMG_HEIGHT];

                //Extraemos solo el canal R (que es igual a B y G en escala de grises)
                for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++)
                {
                    grayPixels[i] = irPixels[i * 4 + 2]; //R channel, tienen los 3 el mismo valor igualmente tras el threshold
                }

                System.Runtime.InteropServices.Marshal.Copy(grayPixels, 0, irMat.DataPointer, grayPixels.Length); //Copiamos al mat

                //Morphology open
                Mat kernel = CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(3, 3), new Point(-1, -1));
                CvInvoke.MorphologyEx(irMat, irMat, MorphOp.Open, kernel, new Point(-1, -1), 1, BorderType.Default, new MCvScalar());

                //Contornos
                VectorOfVectorOfPoint contours = new VectorOfVectorOfPoint();
                Mat hierarchy = new Mat();
                CvInvoke.FindContours(irMat, contours, hierarchy, RetrType.External, ChainApproxMethod.ChainApproxSimple);

                //Calcular centroides y filtrar por área
                List<PointF> currentCentroids = new List<PointF>();

                for (int i = 0; i < contours.Size; i++)
                {
                    using (VectorOfPoint contour = contours[i])
                    {
                        double area = CvInvoke.ContourArea(contour);

                        //Filtrar por área
                        if (area < MIN_BLOB_AREA || area > MAX_BLOB_AREA)
                            continue;

                        //Calcular centroide via momentos
                        Moments moments = CvInvoke.Moments(contour);
                        if (moments.M00 > 0)
                        {
                            float cx = (float)(moments.M10 / moments.M00);
                            float cy = (float)(moments.M01 / moments.M00);
                            currentCentroids.Add(new PointF(cx, cy));
                        }
                    }
                }

                //5. Guardar centroides (thread-safe)
                lock (centroidsLock)
                {
                    detectedCentroids = currentCentroids;
                }

                //Debug: mostrar coordenadas
                if (currentCentroids.Count > 0 && DateTime.Now.Millisecond < 50) //limita output
                {
                    Console.WriteLine($"\n[{DateTime.Now:HH:mm:ss}] Detectados {currentCentroids.Count} blobs:");
                    foreach (var c in currentCentroids)
                    {
                        Console.WriteLine($"  ({c.X:F1}, {c.Y:F1})");
                    }
                }

                //6. Dibujar círculos sobre irPixels en las posiciones detectadas
                foreach (PointF centroid in currentCentroids)
                {
                    DrawCircle(irPixels, (int)centroid.X, (int)centroid.Y, 8, 0, 255, 0); //verde
                }

                //7. Cleanup
                irMat.Dispose();
                kernel.Dispose();
                contours.Dispose();
                hierarchy.Dispose();


                // Escribir en el buffer que NO está siendo mostrado, para cambiar a él periodicamente y evitar editarlo mientras se muestra (da error)
                Bitmap writeBuffer = useIrA ? irBufferA : irBufferB;

                BitmapData bmpData = writeBuffer.LockBits( //Creamos acceso directo a la memoria del bitmap para escribir los pixeles
                    new Rectangle(0, 0, IMG_WIDTH, IMG_HEIGHT),
                    ImageLockMode.WriteOnly,
                    writeBuffer.PixelFormat);

                //Copiamos los pixeles IR al bitmap
                System.Runtime.InteropServices.Marshal.Copy(irPixels, 0, bmpData.Scan0, irPixels.Length);
                writeBuffer.UnlockBits(bmpData);

                // Cambiar al buffer recién escrito
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
                // (todo lo que tenías en Sensor_DepthFrameReady DESPUÉS del CopyPixelDataTo)
                // - Bucle conversión depth a visualización
                // - Actualizar depthPictureBox

                //Pasamos depth (16 bits) a imagen visualizable (8 bits BGRA)
                for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++)
                {
                    short pixel = depthData[i];
                    int depthInMm = pixel >> 3; //Shift 3 bits a la derecha para obtener distancia real en mm

                    byte intensity;
                    if (depthInMm < MIN_DEPTH || depthInMm > MAX_DEPTH)
                    {
                        intensity = 0; //Fuera de rango = negro
                    }
                    else
                    {
                        //Mapeo: cerca=blanco (255), lejos=oscuro (0)
                        //Normalizamos al rango 0-255 invertido
                        double normalized = 1.0 - ((double)(depthInMm - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH));
                        intensity = (byte)(normalized * 255);
                    }

                    depthPixels[i * 4] = intensity;     //Blue
                    depthPixels[i * 4 + 1] = intensity; //Green
                    depthPixels[i * 4 + 2] = intensity; //Red
                    depthPixels[i * 4 + 3] = 255;       //Alpha
                }

                // Escribir en el buffer que NO está siendo mostrado (mismo principio que IR)
                Bitmap writeDepthBuffer = useDepthA ? depthBufferA : depthBufferB;

                BitmapData bmpDataDepth = writeBuffer.LockBits( //Acceso directo a memoria del bitmap
                    new Rectangle(0, 0, IMG_WIDTH, IMG_HEIGHT),
                    ImageLockMode.WriteOnly,
                    writeBuffer.PixelFormat);

                //Copiamos los pixeles Depth al bitmap
                System.Runtime.InteropServices.Marshal.Copy(depthPixels, 0, bmpData.Scan0, depthPixels.Length);
                writeBuffer.UnlockBits(bmpData);

                // Cambiar al buffer recién escrito
                Bitmap displayDepthBuffer = writeBuffer;
                useDepthA = !useDepthA;

                try
                {
                    depthPictureBox.Invoke((MethodInvoker)delegate
                    {
                        depthPictureBox.Image = displayBuffer;
                    });
                }
                catch { }

            }
        }

        static void Sensor_DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame frame = e.OpenDepthImageFrame())
            {
                if (frame == null) return; //Verificamos que hay frame y ventana

                if (depthPictureBox == null || depthPictureBox.IsDisposed || !depthPictureBox.IsHandleCreated)
                    return;

                frame.CopyPixelDataTo(depthData); //Cada pixel son 16 bits
                //Bits 0-2 = índice de jugador (no usado), bits 3-15 = distancia en mm
            }
        }
    }
}