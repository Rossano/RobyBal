using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Threading;
using System.IO.Ports;
using System.Threading;

namespace BalRobyGUI
{
    class application_core: IDisposable
    {
        private const double accSens = 0.061;
        private const double gyroSens = 70.0;
        private const double alpha = 0.9;
        private stm32_class stm32;
        //public ProducerConsumerQueue<MessageData> responses;
        public Queue<MessageData> responses;
        //private ProducerConsumerQueue<Quaternion> quatBuffer;
        //private System.Threading.Timer appliTick;
        private DispatcherTimer appliTick;
 //       private Quaternion q;
        public double roll { get; private set; }
        public double pitch { get; private set; }
        public double yaw { get; private set; }
        public double dt { get; private set; }
        public int[] acc = new int[3];
        private int[] gyro = new int[3];
        private int[] gyroCal = new int[3];
        public double gyroBias { get; private set; }
        public double Force { get; private set; }
        public double pwmA { get; private set; }
        public double pwmB { get; private set; }
        public double[] K;
        private Thread readThread;

        //       public application_core(string com, ProducerConsumerQueue<Quaternion> quatQueue)
        public application_core(string com)
        {
            try
            {
                //responses = new ProducerConsumerQueue<MessageData>();
                responses = new Queue<MessageData>();
                K = new double[4];
 //               quatBuffer = quatQueue;
                stm32 = new stm32_class(com, responses);
                appliTick = new DispatcherTimer();
                appliTick.Interval = new TimeSpan(0, 0, 0, 0, 250);
                appliTick.Tick += new EventHandler(Tick_EvtHandler);
 //               appliTick.IsEnabled = true;
 //               readThread = new Thread(updateThread);
 //               readThread.Start();
//                appliTick.Start();
            }
            catch (Exception ex)
            {
                throw ex;
            }
        }

        public void Dispose()
        {
            responses.Clear();
            stm32.Dispose();
        }

        public void RequestValues()
        {
            string str = ReservedWord.request_data;
            stm32.SendCommand("get_val");// str);
        }


        private void Tick_EvtHandler(object sender, EventArgs e)
        {
            try
            {
                /*DispatcherTimer watchDog = new DispatcherTimer();
                watchDog.Interval = new TimeSpan(0, 0, 5);
                watchDog.Tick += new EventHandler(watchDog_Handler);
                watchDog.IsEnabled = true;
                watchDog.Start();*/
                /*Queue<MessageData>.Enumerator foo = responses.GetEnumerator();                
                //foreach(MessageData msg in foo)
                while(foo.MoveNext())
                {
                    ParseMessage(foo.Current);
                }*/
                /*
                if (responses.Count != 0)
                {
                    var copyQueue = new ProducerConsumerQueue<MessageData>(responses);
                    lock (copyQueue)
                    {
                        foreach(MessageData msg in copyQueue.InnerQueue)
                        {
                            ParseMessage(msg);
                            Quaternion q = new Quaternion(0.0, roll, pitch, yaw);
                            quatBuffer.Enqueue(q);
                        }
                    }
                    responses.Clear();
                }*/
            }
            catch (Exception ex)
            {
                throw ex;
            }
        }

        private void watchDog_Handler(object sender, EventArgs e)
        {
            throw new ObjectDisposedException("STM32 is out of line!");
        }

        public bool ParseMessage(MessageData msg)
        {
            try
            {
                string[] tokens;
                char[] separators = { ',', ':', ' ' }; 
                tokens = msg.msg.Split(separators, StringSplitOptions.RemoveEmptyEntries);
                // Check if the input string is well formed and complete
                if(!(tokens[0].ToLower().Equals(ReserwedWords.start) && tokens[8].ToLower().Equals(ReserwedWords.end)))
                {
                    return false;
                }
                int delta = Convert.ToInt16(tokens[1]);

                /*
                 * Code for the first implementation with all data coming out
                 * 
                acc[0] = Convert.ToInt16(tokens[2]);
                acc[1] = Convert.ToInt16(tokens[3]);
                acc[2] = Convert.ToInt16(tokens[4]);
                gyro[0] = Convert.ToInt16(tokens[5]);
                gyro[1] = Convert.ToInt16(tokens[6]);
                gyro[2] = Convert.ToInt16(tokens[7]);

                dt = (double)delta / 1000;
               // roll = alpha * GetRoll() + (1 - alpha) * ((double)(gyro[0] - gyroCal[0]) / gyroSens * (double)dt * 180 / Math.PI + roll);
                //pitch = alpha * GetPitch() + (1 - alpha) * ((double)(gyro[2] - gyroCal[2]) / gyroSens * dt * 180 / Math.PI + pitch);
                roll = alpha * GetRoll() + (1 - alpha) * ((double)(gyro[0] - gyroCal[0]) / gyroSens * (double)dt + roll);
                pitch = alpha * GetPitch() + (1 - alpha) * ((double)(gyro[2] - gyroCal[2]) / gyroSens * dt  + pitch);
                yaw = alpha * GetYaw() + (1 - alpha) * ((double)(gyro[1] - gyroCal[1]) / gyroSens * (float)dt);
                */

                /*
                 * Code for the Application GUI
                 */
                dt = (double)delta / 1000;

                try
                {
                    pitch = Convert.ToDouble(tokens[2]);
                    gyroBias = Convert.ToDouble(tokens[3]);
                    Force = Convert.ToDouble(tokens[4]);
                    pwmA = Convert.ToDouble(tokens[5]);
                    pwmB = Convert.ToDouble(tokens[6]);
                }
                catch (Exception ex)
                {

                    return false;
                }

                return true;
            }
            catch
            {
                return false;
            }
        }

        private float GetRoll()
        {
            double foo = (float)Math.Atan2(acc[0] / Math.Sqrt(acc[1] ^ 2 + acc[2] ^ 2), acc[0]);
           // double f1 = acc[0] / Math.Sqrt(acc[1] * acc[1] + acc[2] * acc[2]);
            //double f2 = Math.Atan2(f1, acc[0]);            
            try
            {
                double f1 = (double)acc[1] * accSens;
                double f3 = (double)acc[2] * accSens;
                //f2 = Math.Atan2((double)(acc[1] / 1024), (double)(acc[2] / 1024));
                double f2 = Math.Atan2(f1, f3);
            }
            catch (Exception)
            {
                throw new DivideByZeroException("Roll calculation");
            }
            //foo = f2 * 180 / Math.PI;
            return (float)foo;
        }

        private float GetPitch()
        {
            double foo = (float)Math.Atan2(acc[1] / Math.Sqrt(acc[0] ^ 2 + acc[2] ^ 2), acc[1]) * 180 / (float)Math.PI;
            try
            {
                double f1 = (double)acc[0] * accSens;
                double f2 = (double)acc[1] * accSens;
                double f3 = (double)acc[2] * accSens;
                //foo = 180 / Math.PI * Math.Atan2(-f1, Math.Sqrt(f2 * f2 + f3 * f3));
                foo = Math.Atan2(-f1, Math.Sqrt(f2 * f2 + f3 * f3));
            }
            catch(Exception e)
            {
                throw new DivideByZeroException("Pitch calculation");
            }
            return (float)foo;
        }

        private float GetYaw()
        {
            double foo = -(float)Math.Atan2(Math.Sqrt(acc[0] ^ 2 + acc[1] ^ 2) / acc[2], acc[2]) * 180 / Math.PI;
            double f1 = (double)acc[0] * accSens;
            double f2 = (double)acc[1] * accSens;
            double f3 = (double)acc[2] * accSens;
            //foo = -180 / Math.PI * Math.Atan2(Math.Sqrt(acc[0] * acc[0] + acc[1] * acc[1]), acc[2]);
            foo = Math.Atan2(Math.Sqrt(f1 * f1 + f2 * f2), f3);
            return (float)foo;
        }
        
        public int getAccX()
        {
            return acc[0];
        }

        public int getAccY()
        {
            return acc[1];
        }

        public int getAccZ()
        {
            return acc[2];
        }

        public int getGyroX()
        {
            return gyro[0];
        }

        public int getGyroY()
        {
            return gyro[1];
        }

        public int getGyroZ()
        {
            return gyro[2];
        }

        public bool gyroCalibration()
        {
            if (stm32 == null)
            {
                return false;
            }
            try
            {
                long[] tmp = new long[3];
                tmp[0] = gyroCal[0] = 0;
                tmp[1] = gyroCal[1] = 0;
                tmp[2] = gyroCal[2] = 0;
                int num = 10;
                for (int i=0; i < num; i++)
                {
                    string str = stm32._port.ReadLine();
                    string[] tokens;
                    char[] separators = { ',', ':', ' ', '\r', '\n' }; // ReserwedWords.Syntax.Separator.ToArray()
                    tokens = str.Split(separators, StringSplitOptions.RemoveEmptyEntries);
                    // Check if the input string is well formed and complete
                    if (!(tokens[0].ToLower().Contains(ReserwedWords.start) && tokens[8].ToLower().Equals(ReserwedWords.end)))
                    {
                        return false;
                    }
                    int delta = Convert.ToInt16(tokens[1]);
                    acc[0] = Convert.ToInt16(tokens[2]);
                    acc[1] = Convert.ToInt16(tokens[3]);
                    acc[2] = Convert.ToInt16(tokens[4]);
                    gyro[0] = Convert.ToInt16(tokens[5]);
                    gyro[1] = Convert.ToInt16(tokens[6]);
                    gyro[2] = Convert.ToInt16(tokens[7]);
                    tmp[0] += gyro[0];
                    tmp[1] += gyro[1];
                    tmp[2] += gyro[2];
                }
                gyroCal[0] = (int)tmp[0] / num;
                gyroCal[1] = (int)tmp[1] / num;
                gyroCal[2] = (int)tmp[2] / num;
                return true;
            }
            catch(Exception ex)
            {
                throw ex;
            }
        }

        public void getControllerValues()
        {
            string cmd = ReservedWord.controller_get;
            stm32.SendCommand(cmd);
        }

        public void setControllerValues(double k1, double k2, double k3, double k4)
        {
            StringBuilder sb = new StringBuilder(ReservedWord.controller_set);
            sb.Append(ReservedWord.sep).Append(k1).Append(ReservedWord.sep).Append(k2);
            sb.Append(ReservedWord.sep).Append(k3).Append(ReservedWord.sep).Append(k4);
            stm32.SendCommand(sb.ToString());
        }

        public bool parseMsgValues(MessageData msg)
        {            
            try
            {
                string[] tokens;
                char[] separators = { ',', ':', ' ', ':' };
                tokens = msg.msg.Split(separators, StringSplitOptions.RemoveEmptyEntries);
                // Check if the input string is well formed and complete
                int index = tokens[6].IndexOf(ReserwedWords.end);
                if (index > 0)
                {
                    tokens[6].Substring(0, index + 1);
                }
                ///if (!(tokens[0].ToLower().Equals(ReservedWord.mpu) && tokens[7].ToLower().Equals(ReserwedWords.end)))
                if (!(tokens[0].ToLower().Equals(ReservedWord.mpu) && tokens[6].ToLower().EndsWith(ReserwedWords.end)))
                {
                    return false;
                }
                int delta = Convert.ToInt16(tokens[1]);
                
                dt = (double)delta / 1000;

                try
                {                    
                    pitch = Convert.ToDouble(tokens[2], System.Globalization.CultureInfo.InvariantCulture);
                    gyroBias = Convert.ToDouble(tokens[3], System.Globalization.CultureInfo.InvariantCulture);
                    Force = Convert.ToDouble(tokens[4], System.Globalization.CultureInfo.InvariantCulture);
                    pwmA = Convert.ToDouble(tokens[5], System.Globalization.CultureInfo.InvariantCulture);
                    tokens[6] = tokens[6].Remove(tokens[6].Length - 1);
                    pwmB = Convert.ToDouble(tokens[6], System.Globalization.CultureInfo.InvariantCulture);
                }
                catch (Exception ex)
                {
                    return false;
                }

                return true;
            }
            catch
            {
                return false;
            }
        }

        public bool parseMsgContrGet(MessageData msg)
        {
            try
            {
                string[] tokens;
                char[] separators = { ',', ':', ' ' };
                tokens = msg.msg.Split(separators, StringSplitOptions.RemoveEmptyEntries);
                // Check if the input string is well formed and complete
                if (!(tokens[0].ToLower().Equals(ReservedWord.mpu) && tokens[5].ToLower().Equals(ReserwedWords.end)))
                {
                    return false;
                }                

                try
                {
                    K[0] = Convert.ToDouble(tokens[1]);
                    K[1] = Convert.ToDouble(tokens[2]);
                    K[2] = Convert.ToDouble(tokens[3]);
                    K[3] = Convert.ToDouble(tokens[4]);
                }
                catch (Exception ex)
                {

                    return false;
                }

                return true;
            }
            catch
            {
                return false;
            }
        }

        public void controllerToggle()
        {
            string cmd = ReservedWord.controller_toggle;
            stm32.SendCommand(cmd);
        }

        public void controllerSet(bool active)
        {
            string cmd;
            if (active)
            {
                cmd = ReservedWord.controller_get + " 1";
            }
            else
            {
                cmd = ReservedWord.controller_set + " 0";
            }
            stm32.SendCommand(cmd);
        }

        /*private void updateThread()
        {
            MessageData msg = new MessageData();
            while (true)
            {
                msg = responses.Dequeue();
                ParseMessage(msg);
                Quaternion q = new Quaternion(0.0, roll, pitch, yaw);
                quatBuffer.Enqueue(q);
            }
        }*/
    }
}
