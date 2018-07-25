using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;

namespace tinyShell
{
    class Program
    {
        static SerialPort sp;

        static void Main(string[] args)
        {
            //SerialPort sp;
            string cmd;
            string _portname;
            Encoding ascii = Encoding.ASCII;
            Encoding utf8 = Encoding.UTF8;
            Encoding unicode = Encoding.Unicode;
            byte[] byteCmd;
            char[] charCmd;

            try
            {
                _portname = "COM3";                
                sp = new SerialPort(_portname, 115200, Parity.None, 8, StopBits.One);
                sp.Handshake = Handshake.None;
                sp.Encoding = Encoding.UTF8;
               // sp.NewLine = "\n";
                sp.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);
                sp.Open();
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error opening COM port");
            }

            Console.WriteLine("COM port open\ntype end to quit");
            cmd = string.Empty;
            while (!cmd.ToLower().Equals("end"))
            {
                cmd = Console.ReadLine() +"\r";// + "\n\r";                
                //cmd = "get_val";
                // charCmd = UTF8Encoding.UTF8(cmd);
                byteCmd = unicode.GetBytes(cmd);
                byte[] asciiBytes = Encoding.Convert(unicode, ascii, byteCmd);
                char[] asciiChars = new char[ascii.GetCharCount(asciiBytes, 0, asciiBytes.Length)];
                ascii.GetChars(asciiBytes, 0, asciiBytes.Length, asciiChars, 0);
                string asciiString = new string(asciiChars);
                //sp.Write(asciiString);
                //               sp.WriteLine(cmd);
                foreach(char ch in cmd)
                {
                    sp.Write(ch.ToString());
                    System.Threading.Thread.Sleep(11);
                }
                //System.Threading.Thread.Sleep(1000);
                Console.Write("Sent Command : ");
                for(int i=0; i<asciiBytes.Length; i++)
                {
                    Console.Write("{0,4:X2}", asciiBytes[i]);
                }
                Console.WriteLine();
                //cmd += "\n\r";
                //sp.Write(cmd);
            }
        }

        static private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                SerialPort sp = sender as SerialPort;
                string buffer = string.Empty;
                if (sp.ReadBufferSize > 0)
                {
                    buffer = sp.ReadExisting();
                }
                else if (sp.ReadBufferSize <= 0)
                {
                    return;
                }
                //System.Threading.Thread.Sleep(50);
                
                Console.Write(buffer);
            }
            catch (Exception ex)
            {
                Console.WriteLine("STM32 Exception: " + ex.Message);
                throw ex;
            }
        }
    }
}
