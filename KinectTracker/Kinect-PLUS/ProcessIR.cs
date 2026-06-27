using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using Microsoft.Kinect;

namespace KinectTracker
{
    public static class IRProcessor
    {
        public static (List<PointF> centroids, List<Vector3> points3D) Process(
            byte[] colorPixels, byte[] irPixels, short[] depthData,
            BlobDetector blobDetector, DepthMapper depthMapper, out RejectionCounts rejections, out float survivorRadius)
        {
            survivorRadius = -1f;
            // Threshold IR
            for (int i = 0; i < Constants.IMG_WIDTH * Constants.IMG_HEIGHT; i++)
            {
                int irValue = colorPixels[i * 2] | (colorPixels[i * 2 + 1] << 8);
                byte intensity = (byte)(irValue >> 8);

                if (intensity < Constants.THRESHOLD)
                {
                    irPixels[i * 4] = 0;
                    irPixels[i * 4 + 1] = 0;
                    irPixels[i * 4 + 2] = 0;
                }
                else
                {
                    irPixels[i * 4] = intensity;
                    irPixels[i * 4 + 1] = intensity;
                    irPixels[i * 4 + 2] = intensity;
                }
                irPixels[i * 4 + 3] = 255;
            }

            // Detección de blobs
            List<Blob2DInfo> blobCentroids = blobDetector.DetectBlobs(irPixels, out rejections);

            List<PointF> currentCentroids = new List<PointF>();
            List<Vector3> current3DPoints = new List<Vector3>();

            foreach (Blob2DInfo centroid in blobCentroids)
            {
                int xInt = (int)centroid.Centroid.X;
                int yInt = (int)centroid.Centroid.Y;

                if (xInt < 0 || xInt >= Constants.IMG_WIDTH || yInt < 0 || yInt >= Constants.IMG_HEIGHT)
                    continue;

                // La profundidad la da la corona (FindValidDepth + zMin), que lee la esfera de forma fiable.
                int depthMm = depthMapper.FindValidDepth(xInt, yInt, depthData, centroid.RadiusPx);
                if (depthMm < 0) { rejections.ByNoDepth++; continue; }

                // -------------------------------------------------------------------------------------
                // Validación Z-size DESACTIVADA a propósito (TFM).
                // El diámetro del blob (diaPx) es demasiado ruidoso con THRESHOLD=250: el umbral recorta
                // la esfera a su núcleo brillante, así que diaPx mide ~5 px frente a los ~9 px reales a
                // 80 cm y oscila frame a frame (2,3–8,8 px observados). zSize derivado de ahí salta entre
                // ~830 y ~3240 mm para la misma esfera quieta, por lo que el filtro rechazaba detecciones
                // cuya corona era correcta (p.ej. corona=801 vs zSize=1467). Reactivar solo si se logra un
                // binarizado que capture la esfera completa de forma estable.
                //
                // float diaPx = 2f * centroid.RadiusPx;
                // if (diaPx > 0)
                // {
                //     int zSize = (int)(Constants.FOCAL_PX * Constants.SPHERE_MM / diaPx);
                //     if (Math.Abs(depthMm - zSize) > Constants.Z_SIZE_TOL)
                //         { rejections.ByZSize++; continue; }
                // }
                // -------------------------------------------------------------------------------------

                SkeletonPoint world = depthMapper.ConvertTo3D(xInt, yInt, depthMm);
                Vector3 worldMm = new Vector3(world.X * 1000f, world.Y * 1000f, world.Z * 1000f);

                currentCentroids.Add(centroid.Centroid);
                current3DPoints.Add(worldMm);
                survivorRadius = centroid.RadiusPx;
            }

            return (currentCentroids, current3DPoints);
        }
    }
}