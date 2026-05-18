using Microsoft.Kinect;
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

        public int FindValidDepth(int x, int y, short[] depthData)
        {
            // Búsqueda en corona (ring search) alrededor del centroide, excluye el centro saturado por IR,
            // muestrea el borde de la esfera
            // Técnica basada en Keller 2023 / STTAR (Martin-Gomez 2023)
            int depthMm = int.MaxValue;
            List<int> samples = new List<int>();

            for (int dy = -Constants.DEPTH_R_OUTER; dy <= Constants.DEPTH_R_OUTER; dy++)
            {
                for (int dx = -Constants.DEPTH_R_OUTER; dx <= Constants.DEPTH_R_OUTER; dx++)
                {
                    int distSq = dx*dx + dy*dy;

                    if (distSq < Constants.DEPTH_R_INNER * Constants.DEPTH_R_INNER || distSq > Constants.DEPTH_R_OUTER * Constants.DEPTH_R_OUTER)
                        continue;

                    int sx = x + dx;
                    int sy = y + dy;

                    if (sx < 0 || sy < 0 || sx >= Constants.IMG_WIDTH  || sy >= Constants.IMG_HEIGHT) continue;

                    int sIdx = sy * Constants.IMG_WIDTH + sx;
                    int rawSample = depthData[sIdx] & 0xFFFF; //Forzar interpretación sin signo
                    int sample = rawSample >> 3;

                    if (sample >= Constants.MIN_DEPTH && sample <= Constants.MAX_DEPTH && sample < depthMm)
                    {
                        samples.Add(sample);
                    }

                }
            }

            if (samples.Count < Constants.DEPTH_MIN_SAMPLES)
                return -1;

            samples.Sort();
            //return samples[samples.Count/2];

            int zMin = samples[0];

            // Contar cuántos samples están cerca del frente (validación extra)
            int clusterCount = 0;
            for (int i = 0; i < samples.Count; i++)
            {
                if (samples[i] <= zMin + 100)
                    clusterCount++;
                else
                    break;
            }

            if (clusterCount < Constants.DEPTH_MIN_SAMPLES)
                return -1;

            return zMin;
        }

        //Convierte (x, y) en pixeles + depth en mm a coordenadas 3D del mundo
        public SkeletonPoint ConvertTo3D(int x, int y, int depthMm) {
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
