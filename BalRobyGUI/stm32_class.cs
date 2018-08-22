using System;
using System.Collections.Generic;
using System.IO.Ports;

namespace BalRobyGUI
{
    class stm32_class: IDisposable
    {
        public SerialPort _port { get; set; }
        private ulong ID = 0;
        private string _portname;
        private char _terminator = '\r';
        private string str = string.Empty;
        private string buffer = string.Empty;               
        public Queue<MessageData> responses;
        
        public stm32_class(string port, Queue<MessageData> msgBuffer)
        {
            try
            {
                _portname = port;
                responses = msgBuffer;                
                _port = new SerialPort(_portname, 115200, Parity.None, 8, StopBits.One);
                _port.Handshake = Handshake.None;
                _port.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);
                _port.Open();
            }
            catch(Exception ex)
            {
                throw ex;
            }
        }

        public void Dispose()
        {
            _port.Dispose();
            responses.Clear();
        }

        private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                SerialPort sp = sender as SerialPort;
 
                if (_port.ReadBufferSize>0)
                {
                    buffer += sp.ReadExisting();
                }
                else if (sp.ReadBufferSize <= 0)
                {
                    return;
                }

                if(buffer.Contains(ReserwedWords.end))
                {
                    MessageData msg;
                    char[] sep = { '\r', '\n', ' ' };
                    string[] s = buffer.Split(sep, StringSplitOptions.RemoveEmptyEntries);
                    
                    if (s.Length > 1)
                    {
                        buffer = s[s.Length - 1];
                    }
                    else
                    {
                        
                    }
                    
                    foreach (string str in s)
                    {
                        string foo = str + '#';
                        if ((str.ToLower().StartsWith(ReserwedWords.start) || str.ToLower().StartsWith(ReservedWord.mpu)) && str.EndsWith(ReserwedWords.end))
                        {
                            msg.reqId = ID++;                            
                            msg.msg = str;
                            
                            responses.Enqueue(msg);
                        }
                    }
                
                }
                
            }
            catch(Exception ex)
            {
                Console.WriteLine("STM32 Exception: " + ex.Message);
                throw ex;
            }
        }

        public void Open(string port)
        {
            if (!_port.IsOpen)
            {
                _port.Open();
            }
        }

        public void Close()
        {
            if (_port.IsOpen) _port.Close();
        }

        public void SendCommand(string cmd)
        {
            string request = cmd + ReserwedWords.Syntax.End;            
            foreach(char ch in request)
            {
                _port.Write(ch.ToString());
                System.Threading.Thread.Sleep(11);
            }
        }

    }
}
