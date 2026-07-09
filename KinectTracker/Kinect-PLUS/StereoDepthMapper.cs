using System;
using System.Drawing;
using System.Numerics;

namespace KinectTracker
{   // Parámetros de calibración estéreo (MATLAB Stereo Camera Calibrator).
    // Cámara A = referencia (origen). B desplazada según R, T.
    // Baseline ~368,7 mm. Error de reproyección medio 0,35 px.
    public static class StereoCalib
    {
        // Intrínsecos cámara A
        public const float fxA = 589.894313f, fyA = 589.598588f;
        public const float cxA = 312.486353f, cyA = 246.707122f;

        // Intrínsecos cámara B
        public const float fxB = 596.684324f, fyB = 595.807660f;
        public const float cxB = 315.670379f, cyB = 241.163807f;

        // Distorsión radial (k1, k2) de cada cámara
        public static readonly float[] distA = { -0.057811f, 0.188655f };
        public static readonly float[] distB = { -0.097193f, 0.330121f };

        // Rotación de B respecto a A (R) y traslación T (mm)
        // OJO con la convención de MATLAB: ver nota en el triangulador
        public static readonly Matrix4x4 R = new Matrix4x4(
            0.999882f, -0.006699f, 0.013799f, 0f,
            0.007324f, 0.998926f, -0.045753f, 0f,
            -0.013478f, 0.045848f, 0.998857f, 0f,
            0f, 0f, 0f, 1f);

        public static readonly Vector3 T = new Vector3(-315.187789f, -4.572749f, 7.712357f);
    }

    public class StereoDepthMapper
	{
        private readonly Matrix4x4 Rt;        // R transpuesta: lleva direcciones de B al marco de A
        private readonly Vector3 origenB;     // centro óptico de B, expresado en el marco de A

        public StereoDepthMapper()
        {
            // Rᵀ — para rotar las direcciones de los rayos de B al marco de A
            Rt = Matrix4x4.Transpose(StereoCalib.R);

            // origen de B en marco A = -(Rᵀ · T)
            origenB = -Mat2Vec(Rt, StereoCalib.T);
        }

        public Vector3 Pixel2Line(float u, float v, float fx, float fy, float cx, float cy)
        {
            float x = (u - cx) / fx;
            float y = (v - cy) / fy;
            float z = 1;

            return new Vector3(x, y, z);
        }

        public (float xcorr, float ycorr) RemoveDistortion(float x, float y, float[] dist) //corrige la distorsión radial de un punto (x,y) usando los parámetros de distorsión k1 y k2
        {
            float distX = x;
            float idealX = x;
            float distY = y;
            float idealY = y;
            float k1 = dist[0];
            float k2 = dist[1];


            float tol = 1e-6f; 
            int maxIter = 20;
            int iter = 0;
            float err = 100f;

            while ((err > tol) && (iter < maxIter))
            {
                float prevX = idealX;
                float prevY = idealY;

                float r2 = idealX * idealX + idealY * idealY; //radio^2 del punto ideal actual
                float factor = 1 + k1*r2 + k2*r2*r2; //factor de distorsión radial

                idealX = distX/factor;
                idealY = distY / factor;

                float dx = idealX - prevX;
                float dy = idealY - prevY;

                err = (float)Math.Sqrt(dx * dx + dy * dy);
                iter += 1;
            }

            return (idealX, idealY); 
        }

        public Vector3 Triangulate(Vector3 d1, Vector3 d2, out float gap) {//midpoint method
            Vector3 P1 = Vector3.Zero;   // origen del rayo A (cámara A = origen)
            Vector3 P2 = origenB;        // origen del rayo B

            Vector3 w0 = P1 - P2;

            float a = Vector3.Dot(d1, d1);
            float b = Vector3.Dot(d1, d2);
            float c = Vector3.Dot(d2, d2);
            float d = Vector3.Dot(d1, w0);
            float e = Vector3.Dot(d2, w0);

            float denom = a * c - b * b;

            float t1 = (b * e - c * d) / denom;
            float t2 = (a * e - b * d) / denom;

            Vector3 puntoA = P1 + t1 * d1;
            Vector3 puntoB = P2 + t2 * d2;

            gap = Vector3.Distance(puntoA, puntoB);
            return (puntoA + puntoB) / 2f;
        }

        public Vector3 Reconstruct(PointF enA, PointF enB, out float gap)
        {
            //Cámara A: píxel -> rayo -> sin distorsión
            Vector3 rayoA = Pixel2Line(enA.X, enA.Y, StereoCalib.fxA, StereoCalib.fyA, StereoCalib.cxA, StereoCalib.cyA);
            var (axc, ayc) = RemoveDistortion(rayoA.X, rayoA.Y, StereoCalib.distA);
            Vector3 d1 = new Vector3(axc, ayc, 1f);   // dirección en marco A (ya es el marco de referencia)

            //Cámara B: '' -> rotar a marco A
            Vector3 rayoB = Pixel2Line(enB.X, enB.Y, StereoCalib.fxB, StereoCalib.fyB, StereoCalib.cxB, StereoCalib.cyB);
            var (bxc, byc) = RemoveDistortion(rayoB.X, rayoB.Y, StereoCalib.distB);
            Vector3 dirB_propia = new Vector3(bxc, byc, 1f);   // dirección en marco de B
            Vector3 d2 = Mat2Vec(Rt, dirB_propia);              // rotada al marco de A

            //Triangular
            return Triangulate(d1, d2, out gap);
        }

        private static Vector3 Mat2Vec(Matrix4x4 m, Vector3 v)
        {
            return new Vector3(
                m.M11 * v.X + m.M12 * v.Y + m.M13 * v.Z,
                m.M21 * v.X + m.M22 * v.Y + m.M23 * v.Z,
                m.M31 * v.X + m.M32 * v.Y + m.M33 * v.Z);
        }

        // Proyecta un punto 3D del marco A a píxel de la cámara A (pinhole, sin distorsión)
        public static (int x, int y) Project2D(Vector3 p)
        {
            float u = StereoCalib.fxA * p.X / p.Z + StereoCalib.cxA;
            float v = StereoCalib.fyA * p.Y / p.Z + StereoCalib.cyA;
            return ((int)u, (int)v);
        }
    }
}
