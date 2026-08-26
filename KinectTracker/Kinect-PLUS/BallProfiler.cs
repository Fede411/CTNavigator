using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;

namespace KinectTracker
{//Métodos para caracterizar el FLE de precisión y exactitud del sistema, además de su área de trabajo. Exporta todo a .csv para análisis posterior.

    //Observe: Bola individual: FLE de precisión (jitter) y error absoluto (exactitud)
    //ObservePair: Par de bolas: FLE de exactitud (sesgo) y jitter de distancia
    //ObserveMap: Mapa de la zona de trabajo (tip, centroide, residual, nBolas, hadPose)
    //Stats: cálculo estadístico
    //Summary: resumen estadístico impreso en consola y volcado de raws a CSV
    //MapSummary: resumen de mapa de trabajo impreso en consola y volcado de raws a CSV


    public class BallProfiler
    {
        //Valores de una bola individual (FLE de precision, jitter)
        private List<float> radii = new List<float>();
        private List<float> xs = new List<float>();
        private List<float> ys = new List<float>();
        private List<float> zs = new List<float>();
        private float knownDistance;

        //Valores de un par de bolas (FLE de exactitud, sesgo)
        private List<float> pairDist = new List<float>();
        private float knownPairDistance = 0f; //Medida con calibre

        //Mapa de la zona de trabajo
        private struct MapRow { public float tx, ty, tz, cx, cy, cz, res; public int n; public bool pose; }
        private List<MapRow> mapRows = new List<MapRow>();

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

        public void SetKnownPairDistance(float mm)
        {
            knownPairDistance = mm;
        }

        public void ObservePair(float x1, float y1, float z1, //da sesgo de distancia y jitter de distancia
                                float x2, float y2, float z2)
        {
            float dx = x1 - x2, dy = y1 - y2, dz = z1 - z2;
            float dist = (float)Math.Sqrt(dx * dx + dy * dy + dz * dz);
            pairDist.Add(dist);
        }

        //Registra una fila del mapa de trabajo. hadPose distingue tracking efectivo
        public void ObserveMap(Vector3 tip, Vector3 cent, float residual, int nBalls, bool hadPose)
        {
            mapRows.Add(new MapRow
            {
                tx = tip.X,
                ty = tip.Y,
                tz = tip.Z,
                cx = cent.X,
                cy = cent.Y,
                cz = cent.Z,
                res = residual,
                n = nBalls,
                pose = hadPose
            });
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
            //Si no encuentra bolas
            if (pairDist.Count == 0)
            {
                Console.WriteLine("\nBallProfiler: sin muestras.");
                return;
            }

            //Estadística de bola individual
            if (radii.Count > 0)
            {
                var xStat = Stats(xs);
                var yStat = Stats(ys);
                var zStat = Stats(zs);

                Console.WriteLine($"\n=== BallProfiler ({radii.Count} muestras, distancia conocida: {knownDistance} mm) ===");
                Console.WriteLine($"X mm     : media {xStat.mean:F1}  std {xStat.std:F2}");
                Console.WriteLine($"Y mm     : media {yStat.mean:F1}  std {yStat.std:F2}");
                Console.WriteLine($"Z mm     : media {zStat.mean:F1}  std {zStat.std:F2}");

                double jitterLat = Math.Sqrt(xStat.std * xStat.std + yStat.std * yStat.std);
                double jitterAx = zStat.std;
                Console.WriteLine($"\nJitter lateral (XY): {jitterLat:F2} mm");
                Console.WriteLine($"Jitter axial   (Z) : {jitterAx:F2} mm");
                if (jitterLat > 0)
                    Console.WriteLine($"Anisotropia (axial/lateral): {jitterAx / jitterLat:F1}x");

                if (knownDistance > 0)
                {
                    double errorZ = zStat.mean - knownDistance;
                    Console.WriteLine($"Error Z (profundidad absoluta): {errorZ:F1} mm (medida - real)");
                }
            }
            else
            {
                Console.WriteLine($"\n=== BallProfiler (modo par, sin muestras de bola individual) ===");
            }

            //Sesgo y jitter de la distancia entre dos bolas 
            if (pairDist.Count > 0)
            {
                var pStat = Stats(pairDist);
                Console.WriteLine($"\n=== Distancia entre bolas ({pairDist.Count} muestras) ===");
                Console.WriteLine($"Distancia medida: media {pStat.mean:F2}  std {pStat.std:F2} mm");
                Console.WriteLine($"Jitter de distancia (precision): {pStat.std:F2} mm");
                if (knownPairDistance > 0)
                {
                    double sesgo = pStat.mean - knownPairDistance;
                    double sesgoPct = 100.0 * sesgo / knownPairDistance;
                    Console.WriteLine($"Distancia real (calibre): {knownPairDistance:F2} mm");
                    Console.WriteLine($"Sesgo de distancia (exactitud): {sesgo:+0.00;-0.00} mm ({sesgoPct:+0.0;-0.0} %)");
                }
            }

            //Volcado CSV de bola individual
            if (radii.Count > 0)
            {
                using (StreamWriter sw = new StreamWriter(csvPath))
                {
                    sw.WriteLine("radiusPx,x_mm,y_mm,z_mm");
                    for (int i = 0; i < radii.Count; i++)
                        sw.WriteLine($"{radii[i]};{xs[i]};{ys[i]};{zs[i]}");
                }
                Console.WriteLine($"CSV volcado en: {csvPath}");
            }

            //Volcado CSV de pares
            if (pairDist.Count > 0)
            {
                string pairCsv = csvPath.Replace(".csv", "_pairs.csv");
                using (StreamWriter sw = new StreamWriter(pairCsv))
                {
                    sw.WriteLine("pair_dist_mm,known_mm");
                    for (int i = 0; i < pairDist.Count; i++)
                        sw.WriteLine(string.Format(System.Globalization.CultureInfo.InvariantCulture,
                        "{0},{1}", pairDist[i], knownPairDistance));
                }
                Console.WriteLine($"CSV de pares volcado en: {pairCsv}");
            }
        }
        public void MapSummary(string csvPath)
        {
            if (mapRows.Count == 0)
            {
                Console.WriteLine("\nMapping: sin muestras.");
                return;
            }

            int conPose = mapRows.FindAll(r => r.pose).Count;
            Console.WriteLine($"\n=== Mapping ({mapRows.Count} frames, {conPose} con pose, {mapRows.Count - conPose} solo visibilidad) ===");

            Directory.CreateDirectory(Path.GetDirectoryName(csvPath));
            using (StreamWriter sw = new StreamWriter(csvPath))
            {
                foreach (var r in mapRows)
                    sw.WriteLine(string.Format(System.Globalization.CultureInfo.InvariantCulture,
                        "{0},{1},{2},{3},{4},{5},{6},{7},{8}",
                        r.tx, r.ty, r.tz, r.cx, r.cy, r.cz, r.res, r.n, (r.pose ? 1 : 0)));
            }
            Console.WriteLine($"CSV del mapa volcado en: {csvPath}");
        }

        //Exporta tableros IR en gris para Stereo Camera Calibrator de MATLAB.
   
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