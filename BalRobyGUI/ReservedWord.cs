using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BalRobyGUI
{
    public struct ReservedWord
    {
        public static string request_data = "get_val";
        public static string request_feedback = "get_fb";
        public static string request_info = "info";
        public static string set_fb_coeff = "set_coeff";
        public static string get_ack = "get_ACK";
        public static string controller_toggle = "cont_toggle";
        public static string controller_set = "contr_set";
        public static string controller_get = "contr_get";
        public static string controller_state = "contr_state";
        public static string robot_move = "move";
        public static string robot_turn = "turn";
        public static string pid_toggle = "pid";
        public static string pid_set = "pid_set";
        public static string pid_get = "pid_get";
        public static string pid_error = "pid_error";
        public static string sep = " ";
        public static string mpu = "mpu";
	}
}
