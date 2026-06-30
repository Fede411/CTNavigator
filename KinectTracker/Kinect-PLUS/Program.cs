using System;
using System.Drawing;
using System.Windows.Forms;
using Microsoft.Kinect;
using System.Collections.Generic;

namespace KinectTracker
{
    class Program
    {
        [STAThread]
        static void Main(string[] args)
        {
            Console.WriteLine("=== Kinect IR + Depth Viewer ===\n");


            Console.WriteLine("Modo: 1 = Normal, 2 = Perfilado, 3 = Calibración");
            string opt = Console.ReadLine();

            OperationMode mode = OperationMode.Normal;
            float knownDistance = 0;

            if (opt == "2")
            {
                mode = OperationMode.Profiling;
                Console.WriteLine("Distancia conocida de la bola (mm):");
                float.TryParse(Console.ReadLine(), out knownDistance);
            }

            if (opt == "3")
            {
                mode = OperationMode.Calibration;
                Console.WriteLine("¡A calibrar se ha dicho! (Pusla ESPACIO para guardar imagenes en C:/calib)");
            }


            ViewerWindow viewer = new ViewerWindow();
            KinectConfig kinect = new KinectConfig(viewer, mode, knownDistance);

            if (!kinect.Start())
            {
                Console.WriteLine("\nNo se pudo iniciar Kinect. Presiona ENTER");
                Console.ReadLine();
                return;
            }

            Console.WriteLine("\nAbriendo ventana de visualizacion...");
            Console.WriteLine("Cierra la ventana para terminar\n");

            //Mostrar ventana (bloquea hasta cerrarse)
            viewer.ShowWindow();

            //Limpieza
            kinect.Stop();
            Console.WriteLine("\nKinect detenida. Presiona ENTER");
            Console.ReadLine();
        }         
    }
}