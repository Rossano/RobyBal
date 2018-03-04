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
using System.Windows.Shapes;
using InteractiveDataDisplay.WPF;

namespace BalRobyGUI
{
    /// <summary>
    /// Logique d'interaction pour GraphWindow.xaml
    /// </summary>
    public partial class GraphWindow : Window
    {
        private const int POINTS = 100;

        public TimeSpan _time { get; private set; }
        public double _data { get; private set; }
        public Queue<DataPoint> dataQ;
        private System.Windows.Threading.DispatcherTimer tim;

        public GraphWindow()
        {
            InitializeComponent();
            this.DataContext = this;
            dataQ = new Queue<DataPoint>();

            tim = new System.Windows.Threading.DispatcherTimer();
            TimeSpan dt = new TimeSpan(0, 0, 0, 0, 100);
            tim.Interval = dt;
            //            tim.IsEnabled = true;
            tim.IsEnabled = false;
            tim.Tick += new EventHandler(timTick);
 //           tim.Start();
        }

        public void Plot(double[] x, double[] y)
        {
            linegraph.Plot(x, y);
        }

        public void Plot(int[] x, int[] y)
        {
            linegraph.Plot(x, y);
        }

        public void Plot(Queue<DataPoint> data)
        {
            //DataPoint[] dp = data.ToArray();
            double[] x = new double[POINTS];
            double[] y = new double[POINTS];
            int i = 0;
            foreach(DataPoint dp in data)
            {
                x[i] = dp.x;
                y[i++] = dp.y;
            }
            //InteractiveDataDisplay.WPF.DataRect plotRect = new InteractiveDataDisplay.WPF.DataRect(x.Min(), y.Min(), x.Max(), y.Max());
            InteractiveDataDisplay.WPF.DataRect plotRect = new InteractiveDataDisplay.WPF.DataRect(x[0], y.Min(), x.Max(), y.Max());
            linegraph.SetPlotRect(plotRect);
            Plot(x,y);
        }

        //public void PlotMEMS(Queue<DataPoint> data)
        public void PlotMEMS()
        {
            
            //foreach(Queue<DataPoint> qdp in data)
            //for (int i=0; i<3;i++)
            {
                int i = 0;
                //double[] x = new double[POINTS];
                double[] y1 = new double[POINTS];
                double[] y2 = new double[POINTS];
                double[] y3 = new double[POINTS];
                double[] time = new double[POINTS];
                var lg = new LineGraph();
                foreach(DataPoint dp in dataQ)
                {
                    time[i] = dp._time.ToOADate();
                    y1[i] = dp.d1;
                    y2[i] = dp.d2;
                    y3[i++] = dp.d3;
                }
                InteractiveDataDisplay.WPF.DataRect plotRect = new InteractiveDataDisplay.WPF.DataRect(time[0], y1.Min(), time.Max(), y1.Max());
                linegraph.SetPlotRect(plotRect);
                linegraph.Children.Add(lg);
                lg.Plot(time, y1);
                lg.Plot(time, y2);
                lg.Plot(time, y3);
            }
        }

        public void AddMEMSPoint(DataPoint mems)
        {
            DateTime tt;
            if (dataQ.Count > 0)
            {
                tt = mems._time;
            }
            else
            {
                tt = new DateTime();
            }
            DataPoint dp = new DataPoint();

            dp._time = tt;
            dp.d1 = mems.d1;
            dp.d2 = mems.d2;
            dp.d3 = mems.d3;

            dataQ.Enqueue(dp);
            if(dataQ.Count>POINTS)
            {
                dataQ.Dequeue();
            }            
        }

        private void timTick(object sender, EventArgs e)
        {
            double xx;
            if (dataQ.Count() > 0)
            {
                xx = dataQ.Last().x + 0.1;
            }
            else
            {
                xx = 0;
            }
            DataPoint dp = new DataPoint();
            dp.x = xx;
            dp.y = 7 * (Math.Abs(xx) < 1e-10 ? 1 : Math.Sin(xx) / xx + 0);
            dataQ.Enqueue(dp);
            if (dataQ.Count > POINTS)
            {
                dataQ.Dequeue();
                linegraph.UpdateLayout();
            }
            Plot(dataQ);
        }
    }

    public struct DataPoint
    {
        public double x { get; set; }
        public double y { get; set; }
        public DateTime _time { get; set; }
        public double d1 { get; set; }
        public double d2 { get; set; }
        public double d3 { get; set; }
    }
}
