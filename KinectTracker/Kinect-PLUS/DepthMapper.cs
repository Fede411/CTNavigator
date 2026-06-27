using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Data.SqlTypes;

namespace KinectTracker
{
    public class DepthMapper
    {
        private KinectSensor sensor;

        public DepthMapper(KinectSensor sensor) //Constructor
        {
            this.sensor = sensor;
        }

        public int FindValidDepth(int x, int y, short[] depthData, float blobRadius)
        {
            // Barrido radial: anillo fino a anillo fino, de dentro hacia afuera.
            // La esfera retrorreflectante satura el centro (agujero de bloom); su profundidad
            // válida vive en una cáscara fina en el borde. La pared está detrás y más afuera.
            // Nos quedamos con la PRIMERA banda frontal válida = lo más cercano = la esfera.
            int rInner = Math.Max(2, (int)(blobRadius * 0.6f));
            int rMax = (int)(blobRadius * 3.0f) + 15;   // generoso: barremos, no muestreamos todo de golpe

            int holesSeen = 0;   // agujeros acumulados (firma de bloom/saturación antes de la cáscara)

            for (int r = rInner; r <= rMax; r++)
            {
                List<int> ring = new List<int>();

                // recorrer solo el anillo fino de radio [r, r+1)
                for (int dy = -r; dy <= r; dy++)
                {
                    for (int dx = -r; dx <= r; dx++)
                    {
                        int distSq = dx * dx + dy * dy;
                        if (distSq < r * r || distSq >= (r + 1) * (r + 1)) continue;

                        int sx = x + dx, sy = y + dy;
                        if (sx < 0 || sy < 0 || sx >= Constants.IMG_WIDTH || sy >= Constants.IMG_HEIGHT) continue;

                        int sample = (depthData[sy * Constants.IMG_WIDTH + sx] & 0xFFFF) >> 3;

                        if (sample == 0) { holesSeen++; continue; }                                  // agujero (saturación)
                        if (sample < Constants.MIN_DEPTH || sample > Constants.MAX_DEPTH) continue;   // fuera de rango
                        ring.Add(sample);
                    }
                }

                if (ring.Count < Constants.DEPTH_MIN_SAMPLES) continue;   // este anillo no tiene cáscara, sigue hacia afuera

                // ¿hay un grupo frontal compacto en este anillo?
                ring.Sort();
                int zMin = ring[0];
                int cluster = 0;
                for (int i = 0; i < ring.Count && ring[i] <= zMin + 100; i++) cluster++;

                if (cluster < Constants.DEPTH_MIN_SAMPLES) continue;      // banda dispersa, no es cáscara limpia

                // Primera banda frontal válida encontrada.
                if (Constants.VERBOSE_DEPTH)
                    Console.WriteLine($"[Corona] cáscara r={r} | holesSeen={holesSeen} validos={ring.Count} zMin={zMin}");

                // Gate de saturación: si no hubo bloom antes de la banda, es pared, no esfera.
                //if (holesSeen < 10) return -1;

                return zMin;
            }

            if (Constants.VERBOSE_DEPTH)
                Console.WriteLine($"[Corona] sin cáscara | holesSeen={holesSeen}");
            return -1;
        }
        //Convierte (x, y) en pixeles + depth en mm a coordenadas 3D del mundo
        public SkeletonPoint ConvertTo3D(int x, int y, int depthMm)
        {
            return sensor.CoordinateMapper.MapDepthPointToSkeletonPoint(
                DepthImageFormat.Resolution640x480Fps30,
                new DepthImagePoint { X = x, Y = y, Depth = depthMm }
            );
        }

        public DepthImagePoint ConvertTo2D(float x, float y, float z)
        {
            SkeletonPoint sp = new SkeletonPoint();
            sp.X = x / 1000f;
            sp.Y = y / 1000f;
            sp.Z = z / 1000f;

            return sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(
                sp, DepthImageFormat.Resolution640x480Fps30);
        }
    }
}