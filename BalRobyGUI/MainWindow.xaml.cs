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
//using OxyPlot;
/*using Microsoft.Research.DynamicDataDisplay;
using Microsoft.Research.DynamicDataDisplay.DataSources;
using System.ComponentModel;*/
using InteractiveDataDisplay.WPF;

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
        private DateTime _time;
        public GraphWindow _graph;
               
        /*private double roll;
        private double pitch;
        private double yaw;*/
        private string rollStr;
        private string pitchStr;
        public string yawStr;
        // Graphic Data Structures
//       public PlotModel _plot { get; private set; }
//       public IList<DataPoint> Points { get; private set; }
        //        public SeriesCollection SeriesCollection { get; set; }
        public string[] Labels { get; set; }
        //       public Func<double, string> YFormatter { get; set; }
        //       private List<double> data1 = new List<double>();
        //       private List<double> data2 = new List<double>();
        //        private List<double> data3 = new List<double>();

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

//        public DataPointCollection MEMSPointCollection;
//        DispatcherTimer updateCollectionTimer;
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
//            MEMSPointCollection = new DataPointCollection();

/*            data = new double[300];
            for (int i = 0; i < data.Length; i++)
                data[i] = 3.1415 * i / (data.Length - 1);

            for (int i = 0; i < 2; i++)
            {
                var lg = new LineGraph();
                lines.Children.Add(lg);
                lg.Stroke = new SolidColorBrush(Color.FromArgb(255, 0, (byte)(i * 10), 0));
                lg.Description = String.Format("Data series {0}", i + 1);
                lg.StrokeThickness = 2;
                lg.Plot(data, data.Select(v => Math.Sin(v + i / 10.0)).ToArray());
            }*/
            double[] x1 = Enumerable.Range(0, 90).Select(i => i / 3.0).ToArray();
            double[] y1 = x1.Select(v => 7 * (Math.Abs(v) < 1e-10 ? 1 : Math.Sin(v) / v) + 1).ToArray();            
            //linegraph.Plot(x1, y1);
            _graph = new GraphWindow();
//            _graph.Plot(x1, y1);
            _graph.Show();
            
            /*            var ds = new EnumerableDataSource<MEMSPoint>(MEMSPointCollection);
                        ds.SetXMapping(x => time.ConvertToDouble(x._time));
                        ds.SetYMapping(y => y._mems);
                        plotter.AddLineGraph(ds, Colors.Green, 2, "g"); // to use this method you need "using Microsoft.Research.DynamicDataDisplay;" */

            MaxMEMS = 1024;
            MinMEMS = -1024;

            TimeSpan ts = new TimeSpan(0, 0, 0, 0, 250);
            _tick = new DispatcherTimer();
            _tick.Interval = ts;
            _tick.Tick += new EventHandler(TickHandler);
            //_tick.Start();            
            try
            {                
/*                _plot = new PlotModel { Title = "Data Plot" };
                //_plot.Series.Add(new FunctionSeries(Math.Cosh, 0, 10, 0.1, "cosh(x)"));
                this.Points = new List<DataPoint>
                              {
                                  new DataPoint(0, 4),
                                  new DataPoint(10, 13),
                                  new DataPoint(20, 15),
                                  new DataPoint(30, 16),
                                  new DataPoint(40, 12),
                                  new DataPoint(50, 12)
                              };*/
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
 //               MEMSPointCollection.Add(new MEMSPoint((int)(1024*Math.Sin(i * 0.1)), DateTime.Now));
                if (_core.responses.Count > 0)
                {
                    int count = _core.responses.Count;
                    //do
                    int i;
                    for (i = 0; i < count; i++)
                    {
                        MessageData msg = _core.responses.Dequeue();
                        _core.ParseMessage(msg);
                        rollStr = _core.roll.ToString();
                        pitchStr = _core.pitch.ToString();
                        yawStr = _core.yaw.ToString();
                        lbRoll.Content = rollStr;
                        lbPitch.Content = pitchStr;
                        lbYaw.Content = yawStr;
                        RotateTransform rtRoll = new RotateTransform(_core.roll * 180 / Math.PI, 25, 60);
                        rtRoll.CenterX = 25;
                        rtRoll.CenterY = 60;
                        RollRect.RenderTransform = rtRoll;
                        RotateTransform rtPitch = new RotateTransform(_core.pitch * 180 / Math.PI, 25, 60);
                        rtPitch.CenterX = 25;
                        rtPitch.CenterY = 60;
                        PitchRect.RenderTransform = rtPitch;
                        RotateTransform rtYaw = new RotateTransform(_core.yaw * 180 / Math.PI, 25, 60);
                        rtYaw.CenterX = 25;
                        rtYaw.CenterY = 60;
                        YawRect.RenderTransform = rtYaw;
                        DataPoint dp = new DataPoint();
                        dp._time = new DateTime();
                        dp._time = _time;
                        dp.d1 = _core.roll;
                        dp.d2 = _core.pitch;
                        dp.d3 = _core.yaw;
                        //                        _graph.AddMEMSPoint(dp);
                        //                        _graph.PlotMEMS();
                        _graph.newPoint(_core.acc[0], _core.acc[1], _core.acc[2]);
                        _graph.Plot(0);
                        _graph.Plot(1);
                        _graph.Plot(2);
                        /*SeriesCollection[0].Values.Add((double)_core.getAccX());
                        SeriesCollection[1].Values.Add((double)_core.getAccY());
                        SeriesCollection[2].Values.Add((double)_core.getAccZ());
                        if (SeriesCollection[0].Values.Count > 100)
                        {
                            SeriesCollection[0].Values.Remove(0);
                            SeriesCollection[1].Values.Remove(0);
                            SeriesCollection[2].Values.Remove(0);
                        }*/
                        //                       if (data1.Count > 100) data1.Remove(data1[0]);
                        //                        data1.Add((double)_core.getAccX());
                        //                        if (data2.Count > 100) data2.Remove(data2[0]);
                        //                        data2.Add((double)_core.getAccY());
                        //                        if (data3.Count > 100) data3.Remove(data3[0]);
                        //                        data3.Add((double)_core.getAccZ());
                        cvRoll.UpdateLayout();
                        lbRoll.Content = "ROLL = " + rollStr;
                    }
                    //while (count-- > 0);
/*                    double[] foo = new double[100];
                    for (i = 0; i < 100; i++) foo[i] = data1[i];
                    //                    SeriesCollection[1].Values.AddRange(foo);
                    //_plot.Series.Add(createDataSerie(foo, 100, "Acc X"));
                    _plot.Series.Add(new FunctionSeries(Math.Ceiling, 0, 10, 0.10, "1*x"));
                    for (i = 0; i < 100; i++) foo[i] = data2[i];
                    _plot.Series.Add(createDataSerie(foo, 100, "Acc Y"));
                    for (i = 0; i < 100; i++) foo[i] = data3[i];
                    _plot.Series.Add(createDataSerie(foo, 100, "Acc Z"));*/
                }
            }
            catch (Exception ex)
            {
                ErrorMsg(ex.Message);
            }
        }

/*        public event PropertyChangedEventHandler PropertyChanged;

        protected virtual void OnPropertyChanged(string propertyName)
        {
            if (PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }*/

        #region INotifyPropertyChanged members

        public event PropertyChangedEventHandler PropertyChanged;
        protected void OnPropertyChanged(string propertyName)
        {
            if (PropertyChanged != null)
                this.PropertyChanged(this, new System.ComponentModel.PropertyChangedEventArgs(propertyName));
        }

        #endregion
        /*private LineSeries createDataSerie(double[] data, int n = 100, string title = "")
        {
            var ls = new LineSeries { Title = title };
            double dt = 0.1;

            for (int i = 0; i < n; i++)
            {
                double x = i * dt;
                ls.Points.Add(new DataPoint(x, data[i]));
            }
            return ls;
        }*/

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
                /* if (quatQueue == null)
                 {
                     throw new NullReferenceException("Quaternion buffer queue not yet defined!");
                 }
                 _core = new application_core(_comport, quatQueue);*/
                _core = new application_core(_comport);
                //                guiThread.Start();
                int count = 5;
                bool res = false;
                do
                {
                    res = _core.gyroCalibration();
                }
                while ((res == false) && (--count > 0));
                _tick.Start();
                isConnected = true;
            }
            catch (Exception ex)
            {
                _core.Dispose();
                ErrorMsg(ex.Message);
            }
        }

        private void ErrorMsg(string msg)
        {
            MessageBox.Show(msg, "Error", MessageBoxButton.OK, MessageBoxImage.Error);
        }

        private void lbCom_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            _comport = lbCom.SelectedItem as string;
        }

        private void guiUpdate()
        {
            while (true)
            {
                //Quaternion q = quatQueue.Dequeue();
                //if (q != null)
                {
                    /*roll = q.roll;
                    pitch = q.pitch;
                    yaw = q.yaw;                    */
                }
            }
        }

    }

    public class VisibilityToCheckedConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            return ((Visibility)value) == Visibility.Visible;
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            return ((bool)value) ? Visibility.Visible : Visibility.Collapsed;
        }
    }
}
