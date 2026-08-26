using System.Drawing;

namespace KinectTracker
{//Almacena información de un blob detectado en 2D.
    public struct Blob2DInfo
    {
        public PointF Centroid;
        public float RadiusPx;
        public float Circularity;
    }
}

