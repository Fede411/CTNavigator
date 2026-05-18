using MathNet.Numerics;
using System;
using System.Numerics;

namespace KinectTracker
{

    public class KalmanPoseFilter
    {
        private double[] state = new double[6]; //x, y, z, vx, vy y vz
        private double[] P = new double[6]; //Varianza de cada estado

        //Parámetros de ruido
        private double processNoise;    // Q, ruido del proceso (modelo de movimiento), afecta la rapidez de respuesta a cambios
        private double measurementNoise; // R, ruido de la medición (detecciones), afecta la confianza en las detecciones vs. el modelo

        // Rotación (filtro paso bajo)
        private Quaternion lastRotation = Quaternion.Identity;
        private float rotationAlpha; // factor de suavizado (0-1)

        private bool initialized = false;
        private DateTime lastTime;

        public Vector3 FilteredPosition { get; private set; }
        public Quaternion FilteredRotation { get; private set; }
        public KalmanPoseFilter(double processNoise = 100.0, double measurementNoise = 25.0, float rotationAlpha = 0.3f)
        {
            this.processNoise = processNoise;
            this.measurementNoise = measurementNoise;
            this.rotationAlpha = rotationAlpha;

            for (int i = 0; i < 6; i++)
                P[i] = 1000.0; // incertidumbre inicial alta
        }

        public void Predict(DateTime now)
        {
            if (!initialized) return;

            double dt = (now - lastTime).TotalSeconds;
            lastTime = now;

            // Avanzar posición según velocidad
            // x_nuevo = x + vx * dt (igual para y, z)
            for (int i = 0; i < 3; i++)
            {
                state[i] += state[i + 3] * dt;
            }

            // Aumentar incertidumbre
            for (int i = 0; i < 6; i++)
            {
                P[i] += processNoise;
            }

            FilteredPosition = new Vector3((float)state[0], (float)state[1], (float)state[2]);
        }

        public void Update(Vector3 measuredPosition, Quaternion measuredRotation, DateTime now) {

            double dt = (now - lastTime).TotalSeconds;
            if (dt <= 0) dt = 0.033; // fallback a 30fps

            if (!initialized)
            {
                // Primera medición: inicializar estado directamente
                state[0] = measuredPosition.X;
                state[1] = measuredPosition.Y;
                state[2] = measuredPosition.Z;
                state[3] = 0; state[4] = 0; state[5] = 0; // velocidad inicial cero
                lastRotation = measuredRotation;
                FilteredRotation = measuredRotation;
                initialized = true;
                lastTime = now;
                FilteredPosition = measuredPosition;
                return;
            }

            for (int i = 0; i < 3; i++)
            {
                double K = P[i]/(P[i]+measurementNoise);

                // Error en posición
                double measured = (i == 0) ? measuredPosition.X : (i == 1) ? measuredPosition.Y : measuredPosition.Z;
                double error = measured - state[i];

                // Corregir posición
                state[i] += K * error;

                // Corregir velocidad: error/dt es la velocidad "medida"
                state[i + 3] = error / dt;

                // Reducir incertidumbre
                P[i] = (1 - K) * P[i];
            }

            FilteredPosition = new Vector3((float)state[0], (float)state[1], (float)state[2]);

            // Rotación: filtro paso bajo con Slerp
            lastRotation = Quaternion.Slerp(lastRotation, measuredRotation, rotationAlpha);
            FilteredRotation = lastRotation;

            lastTime = now;
        }
    }
}
