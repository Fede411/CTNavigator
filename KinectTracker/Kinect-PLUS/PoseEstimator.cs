using System;
using System.Numerics;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;

namespace KinectTracker
{
	public class PoseEstimator
	{
		public static (Matrix<double> R, Vector3 t, float error) ComputePose(Vector3[] modelPoints, Vector3[] detectedPoints, 
			int[] correspondences)
		{
            //Inicializamos
            Matrix<double> R = Matrix<double>.Build.DenseIdentity(3);
            Vector3 t = Vector3.Zero;
			float error = 0.0f;
            int n = modelPoints.Length;

            //Para poder sacar los centroides y matchear, primero ordenamos los puntos detectados
            //según el orden de los puntos del modelo

            Vector3[] matched = new Vector3[n];
            for (int i = 0; i < n; i++)
            {
                matched[i] = detectedPoints[correspondences[i]];
            }

            //Calcular los centroides
            Vector3 centroidModel = Vector3.Zero;
            Vector3 centroidDetected = Vector3.Zero;

            for (int i = 0; i < n; i++)
            {
                centroidModel += modelPoints[i];
                centroidDetected += matched[i];
            }
            centroidModel /= n;
            centroidDetected /= n;

            //Centralizar los puntos
            Vector3[] modelPointsCentered = new Vector3[n];
            Vector3[] detectedPointsCentered = new Vector3[n];

            for (int i = 0; i < n; i++)
            {
                modelPointsCentered[i] = modelPoints[i] - centroidModel;
                detectedPointsCentered[i] = matched[i] - centroidDetected;
            }

            //Horn's method y SVD 
            Matrix<double> H = Matrix<double>.Build.Dense(3, 3, 0.0);

            for (int i = 0; i < n; i++)
            {
                var p = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(
                    new double[] { modelPointsCentered[i].X, modelPointsCentered[i].Y, modelPointsCentered[i].Z });
                var q = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(
                    new double[] { detectedPointsCentered[i].X, detectedPointsCentered[i].Y, detectedPointsCentered[i].Z });

                H += p.OuterProduct(q);
            }

            var svd = H.Svd(true);

            R = svd.VT.Transpose() * svd.U.Transpose();

            if (R.Determinant() < 0)//Si la rotación es improper (reflexión), corregimos
            {
                Matrix<double> V = svd.VT.Transpose();
                V.SetColumn(2, V.Column(2) * -1);
                R = V * svd.U.Transpose();
            }

            //Sacamos vector de translacion (hay que pasar a coordenadas de MathNet para las operaciones)
            var cmModel = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(
                new double[] { centroidModel.X, centroidModel.Y, centroidModel.Z });
            var cmDetected = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(
                new double[] { centroidDetected.X, centroidDetected.Y, centroidDetected.Z });

            var tVec = cmDetected - R * cmModel;

            t = new Vector3((float)tVec[0], (float)tVec[1], (float)tVec[2]);

            //Sacamos RMS
            double sumSq = 0;
            for (int i = 0; i < n; i++)
            {
                var p = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(
                    new double[] { modelPoints[i].X, modelPoints[i].Y, modelPoints[i].Z });
                var q = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(
                    new double[] { matched[i].X, matched[i].Y, matched[i].Z });

                var diff = R * p + tVec - q;
                sumSq += diff.DotProduct(diff);
            }
            error = (float)Math.Sqrt(sumSq / n);


            return (R, t, error);

        }
	}
}
