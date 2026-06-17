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

                int depthMm = depthMapper.FindValidDepth(xInt, yInt, depthData, centroid.RadiusPx);
                if (depthMm < 0) { rejections.ByNoDepth++; continue; }

                //Validación de profundidad medida con tamaño de blob
                float diaPx = 2f * centroid.RadiusPx;
                if (diaPx > 0)
                {
                    int zSize = (int)(Constants.FOCAL_PX * Constants.SPHERE_MM / diaPx);
                    if (Math.Abs(depthMm - zSize) > Constants.Z_SIZE_TOL)
                        { rejections.ByZSize++; continue; }  // Z incompatible con el tamaño
                }

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