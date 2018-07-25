using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BalRobyGUI
{
    public struct MessageData
    {
        public ulong reqId;
        public string msg;
    }

    public struct ReserwedWords
    {
        public struct Syntax
        {
            public static string CR = "\r\n";
            public static string End = "\r";
            public static string Separator = ",";
            public static string Space = " ";
        }

        public static string version = "ver";
        public static string start = "start";
        public static string end = "#";
    }
}
