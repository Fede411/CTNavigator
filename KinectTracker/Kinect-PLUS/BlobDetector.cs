using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Drawing;

namespace KinectTracker
{
    public class BlobDetector
    {
        public List<PointF> DetectBlobs(byte[] irPixels)
        {
            List<PointF> centroids = new List<PointF>();

            Mat irMat = new Mat(Constants.IMG_HEIGHT, Constants.IMG_WIDTH, DepthType.Cv8U, 1); //Matrix, equivalente al bitmap pero en OpenCV
            byte[] grayPixels = new byte[Constants.IMG_WIDTH * Constants.IMG_HEIGHT];

            //Extraemos solo el canal R (que es igual a B y G en escala de grises)
            for (int i = 0; i < Constants.IMG_WIDTH * Constants.IMG_HEIGHT; i++)
            {
                grayPixels[i] = irPixels[i * 4 + 2]; //R channel
            }

            System.Runtime.InteropServices.Marshal.Copy(grayPixels, 0, irMat.DataPointer, grayPixels.Length); //Copiamos al mat

            //Morphology close
            Mat kernel = CvInvoke.GetStructuringElement(ElementShape.Ellipse, new Size(4, 4), new Point(-1, -1));
            CvInvoke.MorphologyEx(irMat, irMat, MorphOp.Close, kernel, new Point(-1, -1), 1, BorderType.Default, new MCvScalar());

            //Contornos
            VectorOfVectorOfPoint contours = new VectorOfVectorOfPoint();
            Mat hierarchy = new Mat();
            CvInvoke.FindContours(irMat, contours, hierarchy, RetrType.External, ChainApproxMethod.ChainApproxSimple);

            //Calcular centroides y filtrar por área, circularidad y aspect ratio
            List<PointF> currentCentroids = new List<PointF>();
            List<SkeletonPoint> current3DPoints = new List<SkeletonPoint>();

            for (int i = 0; i < contours.Size; i++)
            {
                using (VectorOfPoint contour = contours[i])
                {
                    double area = CvInvoke.ContourArea(contour);

                    //Filtro por área
                    if (area < Constants.MIN_BLOB_AREA || area > Constants.MAX_BLOB_AREA)
                        continue;

                    //Filtro circularidad (esferas son redondas, ~1.0)
                    //Fórmula: 4π × área / perímetro²
                    double perimeter = CvInvoke.ArcLength(contour, true);
                    double circularity = perimeter > 0 ? 4 * Math.PI * area / (perimeter * perimeter) : 0;
                    if (circularity < Constants.MIN_CIRCULARITY) continue;

                    //Filtro aspect ratio (esferas son aprox cuadradas en bounding box)
                    Rectangle bbox = CvInvoke.BoundingRectangle(contour);
                    double aspect = (double)bbox.Width / Math.Max(1, bbox.Height);
                    if (aspect < Constants.MIN_ASPECT || aspect > Constants.MAX_ASPECT) continue;

                    //Calcular centroide via momentos
                    Moments moments = CvInvoke.Moments(contour);
                    if (moments.M00 == 0) continue;

                    float cx = (float)(moments.M10 / moments.M00);
                    float cy = (float)(moments.M01 / moments.M00);

                    centroids.Add(new PointF(cx, cy));
                }
            }

            //Cleanup
            irMat.Dispose();
            kernel.Dispose();
            contours.Dispose();
            hierarchy.Dispose();

            return centroids;
        }
    }
}