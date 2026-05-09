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

            ViewerWindow viewer = new ViewerWindow();
            KinectConfig kinect = new KinectConfig(viewer);

            if (!kinect.Initialize())
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