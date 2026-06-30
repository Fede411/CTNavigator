using System;
using System.Collections.Generic;
using System.IO;

namespace KinectTracker
{
    public class BallProfiler
    {
        private List<float> radii = new List<float>();
        private List<float> xs = new List<float>();
        private List<float> ys = new List<float>();
        private List<float> zs = new List<float>();
        private float knownDistance;

        public BallProfiler(float knownDistance)
        {
            this.knownDistance = knownDistance;
        }

        public void Observe(float radiusPx, float x, float y, float z)
        {
            radii.Add(radiusPx);
            xs.Add(x);
            ys.Add(y);
            zs.Add(z);
        }

        //Media y desviación estándar de un lista
        private (double mean, double std) Stats(List<float> data)
        {
            int n = data.Count;
            double sum = 0;
            for (int i = 0; i < n; i++) sum += data[i];
            double mean = sum / n;

            double sqSum = 0;
            for (int i = 0; i < n; i++)
            {
                double diff = data[i] - mean;
                sqSum += diff * diff;
            }
            double std = Math.Sqrt(sqSum / n);

            return (mean, std);
        }

        public void Summary(string csvPath)
        {
            if (radii.Count == 0)
            {
                Console.WriteLine("\nBallProfiler: sin muestras (la bola nunca se vio sola).");
                return;
            }

            var rStat = Stats(radii);
            var xStat = Stats(xs);
            var yStat = Stats(ys);
            var zStat = Stats(zs);

            Console.WriteLine($"\n=== BallProfiler ({radii.Count} muestras, distancia conocida: {knownDistance} mm) ===");
            Console.WriteLine($"Radio px : media {rStat.mean:F2}  std {rStat.std:F2}");
            Console.WriteLine($"X mm     : media {xStat.mean:F1}  std {xStat.std:F2}");
            Console.WriteLine($"Y mm     : media {yStat.mean:F1}  std {yStat.std:F2}");
            Console.WriteLine($"Z mm     : media {zStat.mean:F1}  std {zStat.std:F2}");

            if (knownDistance > 0)
            {
                double errorZ = zStat.mean - knownDistance;
                Console.WriteLine($"Error Z  : {errorZ:F1} mm (medida - real)");
            }

            // Volcado CSV: una fila por muestra, para graficar tamaño<->Z fuera
            using (StreamWriter sw = new StreamWriter(csvPath))
            {
                sw.WriteLine("radiusPx,x_mm,y_mm,z_mm");
                for (int i = 0; i < radii.Count; i++)
                    sw.WriteLine($"{radii[i]},{xs[i]},{ys[i]},{zs[i]}");
            }
            Console.WriteLine($"CSV volcado en: {csvPath}");
        }

        // === Captura de pares de calibración estéreo (a demanda, una por pose) ===
        // Repurposed: vuelca tableros IR en gris para Stereo Camera Calibrator de MATLAB.
        // El perfilado de precisión estéreo (FLE, TRE) se completará en el paso 6.
        private int parCalib = 0;

        public void GuardarParCalib(byte[] grisA, byte[] grisB, string dir)
        {
            Directory.CreateDirectory(dir);
            ImageUtils.GuardarPNG(grisA, $@"{dir}\A_{parCalib:D2}.png");
            ImageUtils.GuardarPNG(grisB, $@"{dir}\B_{parCalib:D2}.png");
            Console.WriteLine($"[CALIB] par {parCalib} guardado");
            parCalib++;
        }
    }
}