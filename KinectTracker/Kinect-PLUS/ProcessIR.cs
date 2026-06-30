using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using Microsoft.Kinect;

namespace KinectTracker
{
    public static class IRProcessor
    {
        public static List<PointF> Process(
    byte[] colorPixels, byte[] irPixels,
    BlobDetector blobDetector, out RejectionCounts rejections)
        {
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

            // Detección de blobs (solo centroides 2D; la Z la dará la triangulación estéreo)
            List<Blob2DInfo> blobs = blobDetector.DetectBlobs(irPixels, out rejections);

            List<PointF> currentCentroids = new List<PointF>();
            foreach (Blob2DInfo blob in blobs)
            {
                int xInt = (int)blob.Centroid.X;
                int yInt = (int)blob.Centroid.Y;
                if (xInt < 0 || xInt >= Constants.IMG_WIDTH || yInt < 0 || yInt >= Constants.IMG_HEIGHT)
                    continue;
                currentCentroids.Add(blob.Centroid);
            }

            return currentCentroids;
        }
    }
}