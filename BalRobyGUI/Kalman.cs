using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BalRobyGUI
{
    public class Kalman
    {
        #region Constants

        public decimal N { get; set; }
        private const double Q_angle = 0.05;
        private const double Q_gyro = 0.05;
        private const double R_angle = 0.03;

        #endregion

        #region Public  Members

        public double[] X;
        public double[] K;
        public double angle { get; set; }
        public double q_bias { get; set; }
        
        #endregion

        #region Private Members

        private double[,] P;
        private double[,] Q;
        private double rate = 0;
        private double angle_err = 0;
        private double C0 = 0;

        #endregion

        #region Consutructor

        public Kalman()
        {
            N = 2;
            P = new double[2,2];
            Q = new double[2,2];
            X = new double[2];
            K = new double[2];
            angle = 0;
            q_bias = 0;
        }

        public Kalman(decimal n)
        {
            N = n;
            P = new double[(int)N,(int)N];
            Q = new double[(int)N,(int)N];
            X = new double[(int)N];
            K = new double[(int)N];
            angle = 0;
            q_bias = 0;
        }

        #endregion

        #region Public Methods

        public void Init()
        {
            for(int i=0; i<N; i++)
            {
                for(int j=0; j<N; j++)
                {
                    P[i, j] = 0;
                    Q[i, j] = 0;
                }
            }
            for(int i=0; i < N; i++)
            {
                P[i, i] = 1;
                Q[i, i] = 0.5;
            }
        }

        public void Init(double[] p, double[] q)
        {
            if (p.Length != N || q.Length != N) throw new IndexOutOfRangeException("Wrong matrix initialization size!");
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < N; j++)
                {
                    P[i, j] = 0;
                    Q[i, j] = 0;
                }
            }
            for (int i = 0; i < N; i++)
            {
                P[i, i] = p[i];
                Q[i, i] = q[i];
            }
            angle = 0;
            q_bias = 0;
        }

        public double[] kalmanUpdate(double angle_m, double gyro_m, double dt)
        {
            // V 2.0 STM32
            // Update xhat - Project the state ahead
            // Step #1
            double q = gyro_m - q_bias;
            angle += q * dt;

            // Update the estimation error covariance - Project the error covariance ahead
            // Step #2
            P[0, 0] += dt * (dt * P[1, 1] - P[0, 1] - P[1, 0] + Q_angle);
            P[0, 1] -= dt * P[1, 1];
            P[1, 0] -= dt * P[1, 1];
            P[1, 1] += Q_gyro * dt;

            // Discrete Kalman Filter, measurement update equations - Measurement update
            // Calculate Kalman Gain
            // Step #4
            double S = P[0, 0] + R_angle;
            // Step #5
            K[0] = P[0, 0] / S;
            K[1] = P[1, 0] / S;

            // Calculate angle and bias - Update estimate measurement zk
            // Step #3
            angle_err = angle_m - angle;

            // Step #6
            angle += K[0] * angle_err;
            q_bias += K[1] * angle_err;

            // Calculate estimation error covariance - Update error covariance
            // Step #7
            P[0, 0] -= K[0] * P[0, 0];
            P[0, 1] -= K[0] * P[0, 1];
            P[1, 0] -= K[1] * P[0, 0];
            P[1, 1] -= K[1] * P[0, 1];

            // V 1.0 Initial
/*            stateUpdate(gyro_m, dt);

            /*
            angle_err = angle_m - angle;
            C0 = 1;

            double PCt0 = C0 * P[0, 0];
            double PCt1 = C0 * P[1, 0];

            double E = R_angle + C0 * PCt0;

            K[0] = PCt0 / E;
            K[1] = PCt1 / E;

            double t0 = PCt0;1
            double t1 = C0 * P[0, 1];

            P[0, 0] -= K[0] * t0;
            P[0, 1] -= K[0] * t1;
            P[1, 0] -= K[1] * t0;
            P[1, 1] -= K[1] * t1;

            angle += K[0] * angle_err;
            q_bias += K[1] * angle_err;
            X[0] = angle;
            X[1] = q_bias;
            */
 /*           double y = angle_m - angle;
            double S = P[0, 0] + R_angle;

            K[0] = P[0, 0] / S;
            K[1] = P[1, 1] / S;

            angle += K[0] * y;
            q_bias += K[1] * y;

            double P00_temp = P[0, 0];
            double P01_temp = P[0, 1];

            P[0, 0] -= K[0] * P00_temp;
            P[0, 1] -= K[0] * P01_temp;
            P[1, 0] -= K[1] * P00_temp;
            P[1, 1] -= K[1] * P01_temp;
*/
            X[0] = angle;
            X[1] = q_bias;
            return X;
        }

        public double getAngle()
        {
            return X[0];
        }

        public double getGyroBias()
        {
            return X[1];
        }

        public double[] getKalmanState()
        {
            return X;
        }

        #endregion

        #region Private Methods

        private void stateUpdate(double q_m, double dt)
        {
            // V1.0 Original
            /*double q = q_m - q_bias;
            double[,] Pdot = new double[2, 2];

            Pdot[0, 0] = Q_angle - P[0, 1] - P[1, 0];
            Pdot[0, 1] = -P[1, 1];
            Pdot[1, 0] = -P[1, 1];
            P[1, 1] = Q_gyro;

            rate = q;
            angle += q * dt;

            P[0, 0] += Pdot[0, 0] * dt;
            P[0, 1] += Pdot[0, 1] * dt;
            P[1, 0] += Pdot[1, 0] * dt;
            P[1, 1] += Pdot[1, 1] * dt;*/

            // V 1.1 Kalman STM32
            double q = q_m - q_bias;
            angle += q * dt;

            //P[0, 0] += (P[1, 1] * dt - P[0, 1] - P[1, 0] + Q_angle) * dt;
            P[0, 0] += (Q_angle - P[0, 1] - P[1, 0]) * dt;
            P[0, 1] -= P[1, 1] * dt;
            P[1, 0] -= P[1, 1] * dt;
            P[1, 1] += Q_gyro * dt;
        }

        #endregion
    }
}
