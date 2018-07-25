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
        //public ProducerConsumerQueue<MessageData> responses;        
        public Queue<MessageData> responses;
        //public ProducerConsumerQueue<string> responses;

        //public stm32_class(string port, ProducerConsumerQueue<MessageData> msgBuffer)        
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
 //               string buffer = string.Empty;
                if (_port.ReadBufferSize>0)
                {
                    buffer += sp.ReadExisting();
                }
                else if (sp.ReadBufferSize <= 0)
                {
                    return;
                }
//                System.Threading.Thread.Sleep(50);
//                string buffer = sp.ReadExisting();
                //if (buffer.EndsWith(ReserwedWords.Syntax.End))
                if(buffer.Contains(ReserwedWords.end))
                {
                    MessageData msg;
                    char[] sep = { '\r', '\n', ' ' };
                    string[] s = buffer.Split(sep, StringSplitOptions.RemoveEmptyEntries);
                    //                    Console.Write(buffer);
                    //                   for (int i = 0; i <= s.Length - 1; i++) s[i] += '#';
                    if (s.Length > 1)
                    {
                        buffer = s[s.Length - 1];
                    }
                    else
                    {
                        
                    }
                    //buffer = string.Empty;
                    //                    buffer.Remove(0, buffer.IndexOf("\r\n"));
                    foreach (string str in s)
                    {
                        string foo = str + '#';
                        if ((str.ToLower().StartsWith(ReserwedWords.start) || str.ToLower().StartsWith(ReservedWord.mpu)) && str.EndsWith(ReserwedWords.end))
                        {
                            msg.reqId = ID++;
                            //msg.msg = buffer;
                            msg.msg = str;
                            //                            Console.WriteLine("STM32 Enqueue: " + str);
                            responses.Enqueue(msg);
                        }
                        /*if (!str.Contains(':'))
                        {
                            buffer = str + ReserwedWords.Syntax.End;
                        }
                        else
                        {
                            responses.Enqueue(msg);
                        }*/
                    }
                   // buffer = string.Empty;
                }
                
                /*
                //byte[] buf = new byte[sp.ReadBufferSize];
                char[] buf = new char[sp.ReadBufferSize];
                int bytesRead = sp.Read(buf, 0, buf.Length);
                //str += Encoding.ASCII.GetString(buf, 0, bytesRead);
                str += sp.ReadExisting();
                if (str.IndexOf((char)_terminator) > -1)
                {
                    string foo = str.Substring(0, str.IndexOf((char)_terminator));
                    str = str.Substring(str.IndexOf((char)_terminator));
                    MessageData msg = new MessageData();
                    msg.reqId = ID++;
                    msg.msg = foo;
                    responses.Enqueue(msg);
                }
                */
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
            //_port.Write(request);
            foreach(char ch in request)
            {
                _port.Write(ch.ToString());
                System.Threading.Thread.Sleep(11);
            }
        }

    }
}
