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
            // Búsqueda en corona (ring search) alrededor del centroide, excluye el centro saturado por IR,
            // muestrea el borde de la esfera; Técnica basada en Keller 2023 / STTAR (Martin-Gomez 2023), modificada con radio dinámico según el tamaño del blob
            int depthMm = int.MaxValue;
            List<int> samples = new List<int>();

            int depth_r_inner = Math.Max(2, (int)(blobRadius * 0.6f));

            int depth_r_outer = (int)(blobRadius * 1.5f) +3;

            for (int dy = -depth_r_outer; dy <= depth_r_outer; dy++)
            {
                for (int dx = -depth_r_outer; dx <= depth_r_outer; dx++)
                {
                    int distSq = dx*dx + dy*dy;

                    if (distSq < depth_r_inner * depth_r_inner || distSq > depth_r_outer * depth_r_outer)
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
