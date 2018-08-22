﻿using System;
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
        List<double> accX { get; set; }
        List<double> accY { get; set; }
        List<double> accZ { get; set; }
        public List<double> time { get; set; }
        long t = 0;
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
            time = new List<double>();

            accX = new List<double>();
            accY = new List<double>();
            accZ = new List<double>();
            //           tim.Start();
            linegraphX.Stroke = new SolidColorBrush(Colors.Blue);
            linegraphX.StrokeThickness = 2;
            linegraphY.Stroke = new SolidColorBrush(Colors.Red);
            linegraphY.StrokeThickness = 2;
            linegraphZ.Stroke = new SolidColorBrush(Colors.Green);
            linegraphZ.StrokeThickness = 2;
            Rect r = new Rect(0, -2048, 100, 2048);
            DataRect dr = new DataRect(0, -2048, 10, 2048);
            linegraphX.SetPlotRect(dr); 
                
        }

        public void Plot(double[] x, double[] y)
        {
            //linegraph.Plot(x, y);
        }

        public void Plot(int[] x, int[] y)
        {
            //linegraph.Plot(x, y);
        }

        public void Plot(int i)
        {
            double[] _time = new double[] { };
            double[] acc_X = new double[] { };
            double[] acc_Y = new double[] { };
            double[] acc_Z = new double[] { };
            if (time.Count > POINTS)
            {
                time.RemoveAt(0);
                accX.RemoveAt(0);
                accY.RemoveAt(0);
                accZ.RemoveAt(0);
            }
            _time = time.ToArray();

            switch (i)
            {
                case 0:
                    acc_X = accX.ToArray();
                    linegraphX.Plot(_time, acc_X); break;
                case 1:
                    acc_Y = accY.ToArray();
                    linegraphY.Plot(_time, acc_Y); break;
                case 2:
                    acc_Z = accZ.ToArray();
                    linegraphZ.Plot(_time, acc_Z); break;
            }

        }

        public void PlotMEMS()
        {
            int i = 0;            
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
            lg.Plot(time, y1);
            lg.Plot(time, y2);
            lg.Plot(time, y3);            
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
            if (dataQ.Count > POINTS)
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
 //           dataQ.Enqueue(dp);
            if (dataQ.Count > POINTS)
            {
//                dataQ.Dequeue();
 //              linegraph.UpdateLayout();
            }
            //           Plot(dataQ);
            time.Add(0.0 * t++);
            accX.Add((int)(1000 * dp.y));
        }

  /*      public void newPoint(int x, int y, int z)
        {
            time.Add(0.1 * t++);
            accX.Add(x);
            accY.Add(y);
            accZ.Add(z);
        }*/

        public void newPoint(double x, double y, double z)
        {
            time.Add(0.1 * t++);
            accX.Add(x);
            accY.Add(y);
            accZ.Add(z);
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
