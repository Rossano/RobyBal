using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.ComponentModel;
using System.IO.Ports;
using System.Windows.Threading;
using System.Windows.Media;
using InteractiveDataDisplay.WPF;
using Xceed.Wpf.Toolkit;

namespace BalRobyGUI
{
    /// <summary>
    /// Logique d'interaction pour MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window //, INotifyPropertyChanged
    {
        private application_core _core;
        private string _comport;
        private DispatcherTimer _tick;
        private bool isConnected = false;
        private double R = 0.005;
        private double Q = 0.005;
        private DateTime _time;
        public GraphWindow _graph;
        public Kalman kalman;
        public double deltaT;
        private double K1;
        private double K2;
        private double K3;
        private double K4;
        private bool useKalman = false;
                       
        private string pitchStr;
 
        public string[] Labels { get; set; }
 
        private int _maxMEMS;
        public int MaxMEMS
        {
            get { return _maxMEMS; }
            set { _maxMEMS = value; this.OnPropertyChanged("MaxMEMS"); }
        }

        private int _minMEMS;
        public int MinMEMS
        {
            get { return _minMEMS; }
            set { _minMEMS = value; this.OnPropertyChanged("MinMEMS"); }
        }

        private int i = 0;
        private double[] data;

        public MainWindow()
        {
            string[] comlist = SerialPort.GetPortNames();
            InitializeComponent();            
            foreach (string str in comlist)
            {
                lbCom.Items.Add(str);
            }
            this.DataContext = this;
            _time = new DateTime();
            double[] x1 = Enumerable.Range(0, 90).Select(i => i / 3.0).ToArray();
            double[] y1 = x1.Select(v => 7 * (Math.Abs(v) < 1e-10 ? 1 : Math.Sin(v) / v) + 1).ToArray();

            kalman = new Kalman();

            _graph = new GraphWindow();
            _graph.Show();
            
            MaxMEMS = 1024;
            MinMEMS = -1024;
           
            TimeSpan ts = new TimeSpan(0, 0, 0, 0, 4*250);
            _tick = new DispatcherTimer();
            _tick.Interval = ts;
            _tick.Tick += new EventHandler(TickHandler);
            
            try
            {
                
            }
            catch (Exception ex)
            {
                ErrorMsg(ex.Message);
            }
        }        

        private void TickHandler(object sender, EventArgs e)
        {
            try
            {
                i++;
                _time += _tick.Interval;
                //               _core.RequestValues();
                _core.RequestValues();

                if (_core.responses.Count > 0)
                {
                    int count = _core.responses.Count;
                    //do
                    int i;
                    for (i = 0; i < count; i++)
                    {
                        MessageData msg = _core.responses.Dequeue();
                        //if (_core.parseMsgValues(msg) == false)
                        //{
                        //    return;
                        //}
                        if(!_core.ParseMessage(msg))
                        {
                            return;
                        }
                        Console.WriteLine(msg.msg);
//                        rollStr = _core.roll.ToString();
                        pitchStr = _core.pitch.ToString();
 //                       yawStr = _core.yaw.ToString();
 //                      lbRoll.Content = rollStr;
                        lbPitch.Content = pitchStr;
 //                       lbYaw.Content = yawStr;
                        RotateTransform rtRoll = new RotateTransform(_core.roll * 180 / Math.PI, 25, 60);
                        rtRoll.CenterX = 25;
                        rtRoll.CenterY = 60;
 //                       RollRect.RenderTransform = rtRoll;
                        RotateTransform rtPitch = new RotateTransform(_core.pitch * 180 / Math.PI, 25, 60);
                        rtPitch.CenterX = 25;
                        rtPitch.CenterY = 60;
                        PitchRect.RenderTransform = rtPitch;
                        RotateTransform rtYaw = new RotateTransform(_core.yaw * 180 / Math.PI, 25, 60);
                        rtYaw.CenterX = 25;
                        rtYaw.CenterY = 60;
                        //                       YawRect.RenderTransform = rtYaw;
                        Force.Content = _core.Force.ToString();
                        PWMLeft.Content = _core.pwmA.ToString();
                        PWMRight.Content = _core.pwmB.ToString();

                        ///
                        /// Kalman Filter
                        /// 
                        double x; 
                        double angle;
                        double bias;
                        if (useKalman)
                        {
                            x = _core.getAccX() / _core.getAccZ();
                            kalman.kalmanUpdate(_core.pitch, _core.getGyroZ() / 70.0, _core.dt);
                            angle = kalman.getAngle();
                            bias = kalman.getGyroBias();
                        }
                        else
                        {
                            angle = _core.pitch;
                            bias = _core.gyroBias;
                        }
                        double angleErr = angle - _core.pitch;
                        DataPoint dp = new DataPoint();
                        dp._time = new DateTime();
                        dp._time = _time;
                        /*dp.d1 = _core.roll;
                        dp.d2 = _core.pitch;
                        dp.d3 = _core.yaw;*/
                        if (!(double.IsNaN(angle) || double.IsInfinity(angle) || double.IsNaN(bias) || double.IsInfinity(bias) || double.IsNaN(angleErr) || double.IsInfinity(angleErr)))
                        {
                            dp.d1 = 180 / Math.PI * angle;
                            dp.d2 = bias;
                            dp.d3 = 180 / Math.PI * angleErr;

                            if (useKalman)
                            {
                                _graph.newPoint(180 / Math.PI * kalman.getAngle(), kalman.getGyroBias(), 180 / Math.PI * (kalman.getAngle() - _core.roll));
                            }
                            else
                            {
                                _graph.newPoint(180 / Math.PI * _core.pitch, _core.gyroBias, (_core.Force));
                            }
                            _graph.Plot(0);
                            _graph.Plot(1);
                            _graph.Plot(2);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                ErrorMsg(ex.Message);
            }
        }


        #region INotifyPropertyChanged members

        public event PropertyChangedEventHandler PropertyChanged;
        protected void OnPropertyChanged(string propertyName)
        {
            if (PropertyChanged != null)
                this.PropertyChanged(this, new System.ComponentModel.PropertyChangedEventArgs(propertyName));
        }

        #endregion        

        private void getData(string str2parse)
        {
            try
            {
                char[] separators = { ',', ' ' };
                string[] tokens = str2parse.Split(separators);
                uint dt = Convert.ToUInt32(tokens[0]);
                uint accX = Convert.ToUInt32(tokens[1]);
                uint accY = Convert.ToUInt32(tokens[2]);
                uint accZ = Convert.ToUInt32(tokens[3]);
                uint gyroX = Convert.ToUInt32(tokens[4]);
                uint gyroY = Convert.ToUInt32(tokens[5]);
                uint gyroZ = Convert.ToUInt32(tokens[6]);
                deltaT = dt / 1000;
            }
            catch (Exception ex)
            {
                ErrorMsg(ex.Message);
            }
        }

        public void lbCom_Selected(object sender, RoutedEventArgs e)
        {
            _comport = (string)lbCom.SelectedValue;
        }

        private void bConnect_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                _core = new application_core(_comport);

                int count = 5;
/*  
 *              Calibration
 *              
                bool res = false;
                do
                {
                    //                    res = _core.gyroCalibration();
                }
                while ((res == false) && (--count > 0));*/
                _tick.Start();
                isConnected = true;

                _core.getControllerValues();
                System.Threading.Thread.Sleep(1000);

                K1 = _core.K[0];
                K2 = _core.K[1];
                K3 = _core.K[2];
                K4 = _core.K[3];
            }
            catch (Exception ex)
            {
                _core.Dispose();
                ErrorMsg(ex.Message);
            }
        }

        private void ErrorMsg(string msg)
        {
            System.Windows.MessageBox.Show(msg, "Error", MessageBoxButton.OK, MessageBoxImage.Error);
        }

        private void lbCom_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            _comport = lbCom.SelectedItem as string;
        }

        private void guiUpdate()
        {
            while (true)
            {
             
            }
        }

        private void dudR_ValueChanged(object sender, RoutedPropertyChangedEventArgs<object> e)
        {
            var val = (double)e.NewValue;
            R = val;
        }

        private void dudQ_ValueChanged(object sender, RoutedPropertyChangedEventArgs<object> e)
        {
            var val = (double)e.NewValue;
            Q = val;
        }

        private void bReset_Click(object sender, RoutedEventArgs e)
        {
            kalman.angle = 0;
            kalman.q_bias = 0;
            double[] z = { 0, 0 };
            kalman.X = z;
            double[] Rv = { R, R };
            double[] Qv = { Q, Q };
            kalman.Init(Rv, Qv);
        }

        private void guiK1_TextChanged(object sender, TextChangedEventArgs e)
        {
            var _k1 = guiK1.Text; 
            try
            {
                K1 = Convert.ToDouble(_k1);
            }
            catch(Exception ex)
            {
                ErrorMsg("Error converting K1\n" + ex.Message);
            }
        }

        private void guiK2_TextChanged(object sender, TextChangedEventArgs e)
        {
            var _k2 = guiK2.Text;
            try
            {
                K2 = Convert.ToDouble(_k2);
            }
            catch(Exception ex)
            {
                ErrorMsg("Error converting K2\n" + ex.Message);
            }
        }

        private void guiK3_TextChanged(object sender, TextChangedEventArgs e)
        {
            var _k3 = guiK3.Text;
            try
            {
                K3 = Convert.ToDouble(_k3);
            }
            catch(Exception ex)
            {
                ErrorMsg("Error converting K3\n" + ex.Message);
            }
        }

        private void guiK4_TextChanged(object sender, TextChangedEventArgs e)
        {
            var _k4 = guiK4.Text;
            try
            {
                K4 = Convert.ToDouble(_k4);
            }
            catch(Exception ex)
            {
                ErrorMsg("Error converting K4\n" + ex.Message);
            }
        }

        private void cbContrActivation_Checked(object sender, RoutedEventArgs e)
        {
            _core.controllerSet(true);
        }

        private void cbContrActivation_Unchecked(object sender, RoutedEventArgs e)
        {
            _core.controllerSet(false);
        }

        private void bReadCoeff_Click(object sender, RoutedEventArgs e)
        {
            string msg = ReservedWord.controller_get;
            _core.getControllerValues();
            guiK1.Text = K1.ToString();
            guiK2.Text = K2.ToString();
            guiK3.Text = K3.ToString();
            guiK4.Text = K4.ToString();
        }

        private void bWriteCoeff_Click(object sender, RoutedEventArgs e)
        {
            _core.setControllerValues(K1, K2, K3, K4);
        }
    }
}
