using System;
using System.Drawing;
using System.Windows.Forms;
using Microsoft.Kinect;
using System.Collections.Generic;

namespace KinectTracker
{//Programa principal. Inicializa Kinect, crea la ventana de visualización y gestiona el flujo de ejecución.
    class Program
    {
        [STAThread]
        static void Main(string[] args)
        {
            Console.WriteLine("=== Kinect IR + Depth Viewer ===\n");


            OperationMode mode = OperationMode.Normal;
            float knownDistance = 0;
            KinectConfig.ProfileKind profileKind = KinectConfig.ProfileKind.Jitter;

            if (args.Length > 0)
            {
                //Modo por argumento (lanzado desde Slicer)
                switch (args[0].ToLower())
                {
                    case "normal": mode = OperationMode.Normal; break;
                    case "profiling": mode = OperationMode.Profiling; break;
                    case "calibration": mode = OperationMode.Calibration; break;
                }
                //Perfilado necesita la distancia; segundo argumento opcional
                if (mode == OperationMode.Profiling && args.Length > 1)
                    float.TryParse(args[1], out knownDistance);
                //Submodo opcional como tercer argumento: "jitter" o "bias"
                if (mode == OperationMode.Profiling && args.Length > 2 &&
                    args[2].ToLower() == "bias")
                    profileKind = KinectConfig.ProfileKind.Bias;
            }
            else
            {
                //Menú de consola (ejecución manual)
                Console.WriteLine("Modo: 1 = Normal, 2 = Perfilado, 3 = Calibración");
                string opt = Console.ReadLine();
                if (opt == "2")
                {
                    mode = OperationMode.Profiling;
                    Console.WriteLine("Submodo: 1 = Jitter (1 bola)  |  2 = Sesgo de distancia (2 bolas)  |  3 = Mapping (barrido del volumen)");
                    Console.WriteLine("Al tener lista la escena, pulsa ESPACIO para empezar la captura.");
                    string sub = Console.ReadLine();
                    if (sub == "2") profileKind = KinectConfig.ProfileKind.Bias;
                    else if (sub == "3") profileKind = KinectConfig.ProfileKind.Mapping;
                    else profileKind = KinectConfig.ProfileKind.Jitter;

                    if (profileKind == KinectConfig.ProfileKind.Bias)
                        Console.WriteLine("Distancia REAL entre las 2 bolas (mm, medida con calibre):");
                    else if (profileKind == KinectConfig.ProfileKind.Mapping)
                        Console.WriteLine("Mapping: mueve el instrumento por el volumen. ESPACIO empieza, ESPACIO otra vez para y cierra.");
                    else
                        Console.WriteLine("Distancia aproximada de la bola al sensor (mm, 0 si no aplica):");

                    if (profileKind != KinectConfig.ProfileKind.Mapping)
                        float.TryParse(Console.ReadLine(), out knownDistance);
                }
                if (opt == "3")
                {
                    mode = OperationMode.Calibration;
                    Console.WriteLine("¡A calibrar se ha dicho! (Pusla ESPACIO para guardar imagenes en C:/calib)");
                }
            }


            ViewerWindow viewer = new ViewerWindow();
            KinectConfig kinect = new KinectConfig(viewer, mode, knownDistance, profileKind);

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