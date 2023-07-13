using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;
using System.Threading;
using System.Diagnostics;
using System.Collections;


namespace SCN6
{
	class SCN6_Driver
	{
		public const int TMBS_NO_EXIST = 0;
		public const int TMBS_INITIAL = 1;
		public const int TMBS_INIT_ERROR = 2;
		public const int TMBS_OPENING = 3;
		public const int TMBS_RUNNING = 4;
		public const int SIO_DONE = 1;
		public const int SIO_ERROR = 0;

		public static COMPACK newPacket = new COMPACK();

		static public int hModule = 6;
		static public int[] axes_info;

		#region IMPORT TMBSCOM.DLL

		#region ############ COMUNICACIONES ############
		[DllImport("TMBSCOM", EntryPoint = "_init_tmbs_config@24")]
		private static extern int init_tmbs_config(
						string port,      // Port Name ( "COM1","COM2"etc. )
						int baud,          // Baudrate( 4,5,6,7,11H,12H,13H,14H )
						int nrt,           // Retry
						bool reset,        //
						bool automatic,    //
						ref int axes_info     //
						);

		[DllImport("TMBSCOM", EntryPoint = "_close_tmbs@0")]
		private static extern int close_tmbs();

		[DllImport("TMBSCOM", EntryPoint = "_get_current_baud@0")]
		private static extern int get_current_baud();

		[DllImport("TMBSCOM", EntryPoint = "_init_tmbs@0")]
		private static extern int init_tmbs();

		#endregion

		#region ############ ESTADO ############

		[DllImport("TMBSCOM", EntryPoint = "_get_tmbs_state@0")]
		private static extern int get_tmbs_state();

		[DllImport("TMBSCOM", EntryPoint = "_get_sio_error@0")]
		private static extern int get_sio_error();

		[DllImport("TMBSCOM", EntryPoint = "_check_pfin@4")]
		private static extern int check_pfin(int axis);

		[DllImport("TMBSCOM", EntryPoint = "_check_status@4")]
		private static extern int check_status(int axis);

		[DllImport("TMBSCOM", EntryPoint = "_check_alrm@4")]
		private static extern int check_alrm(int axis);

		[DllImport("TMBSCOM", EntryPoint = "_check_son@4")]
		private static extern int check_son(int axis);

		[DllImport("TMBSCOM", EntryPoint = "_reset_alarm@4")]
		private static extern int reset_alarm(int axis);

		[DllImport("TMBSCOM")]
		private static extern int read_param(int axis, ref COMPACK dst);

		[DllImport("TMBSCOM", EntryPoint = "_set_son@4")]
		private static extern int set_son(int axis);

		[DllImport("TMBSCOM", EntryPoint = "_set_soff@4")]
		private static extern int set_soff(int axis);

		#endregion

		#region ############ MOVIMIENTO ############

		[DllImport("TMBSCOM", CallingConvention = CallingConvention.Winapi, EntryPoint = "_move_point@8")]
		private static extern int move_point(int axis, int point);

		[DllImport("TMBSCOM", EntryPoint = "_move_org@8")]
		private static extern int move_org(int axis, int position);

		[DllImport("TMBSCOM")]
		private static extern int move_abs(int axis, int position);

		[DllImport("TMBSCOM")]
		private static extern int move_jog(int axis, int distance);

		[DllImport("TMBSCOM", EntryPoint = "_move_inc@8")]
		private static extern int move_inc(int axis, int distance);

		[DllImport("TMBSCOM", EntryPoint = "_move_rotate@16")]
		private static extern int move_rotate(int axis, int dir, int velocidad, int aceleracion);

		#endregion

		#region ############ PARAMETROS ############

		[DllImport("TMBSCOM", EntryPoint = "_write_velocity@12")]
		private static extern int write_velocity(int axis, int vcmd, int acmd);

		[DllImport("TMBSCOM", EntryPoint = "_select_svparm@12")]
		private static extern int select_svparm(int axis, int gain_sel, int svparm);

		[DllImport("TMBSCOM", EntryPoint = "_write_trqlim@12")]
		private static extern int write_trqlim(int axis, int atstop, int atmovement);

		[DllImport("TMBSCOM", EntryPoint = "_write_inpos@8")]
		private static extern int write_inpos(int axis, int width);

		[DllImport("TMBSCOM", EntryPoint = "_write_point@12")]
		private static extern int write_point(int axis, int point, ref COMPACK src);

		[DllImport("TMBSCOM", EntryPoint = "_read_point@12")]
		private extern static int read_point(int axis, int point, ref COMPACK dst);

		[DllImport("TMBSCOM")]
		private static extern int load_param(int axis);

		[DllImport("TMBSCOM")]
		private static extern int write_param(int axis, ref COMPACK src);

		[DllImport("TMBSCOM")]
		private static extern int read_svmem(int axis, int address, ref int dst);

		[DllImport("TMBSCOM")]
		private static extern int write_svmem(int axis, int address, ref int dst);

		public struct COMPACK
		{
			[MarshalAs(UnmanagedType.ByValArray, SizeConst = 31)]
			public long[] address;
			[MarshalAs(UnmanagedType.ByValArray, SizeConst = 31)]
			public long[] data;

			public void initialize()
			{
				address = new long[31];
				data = new long[31];

				for (int i = 0; i < address.Length; i++)
					address[i] = i;
			}
		}

		#endregion PARAMETROS

		#endregion IMPORT TMBSCOM.DLL

		#region &&&&&&&&&&&&&&&&& PUBLIC METHODS &&&&&&&&&&&&&&&&&

		#region ############ COMMUNICATION ############


		public static int Start_Communication(String port, int repeat)
		{

			axes_info = new int[17];

			int i = 0;
			int j = repeat;

			for (i = 0; i < 16; i++)
				axes_info[i] = -1;

			axes_info[0] = 0; // Axis#0 is used
			axes_info[1] = 0;
			axes_info[2] = 0;
			while (init_tmbs_config(port, Convert.ToInt16("14", 16), 3, false, false, ref axes_info[0]) != 1 && j != 0)
			{
				if (get_tmbs_state() == TMBS_INIT_ERROR)
				{
					Console.WriteLine("INIT FAILED");
					return get_tmbs_state();
				}
				j--;
			}
			j = repeat;
			while (get_tmbs_state() <= TMBS_OPENING && j != 0)
			{
				Thread.Sleep(500);
				j--;
			}
			Console.WriteLine("Termi-BUS is now running");
			return get_tmbs_state();
		}

		public static int Close_Communications()
		{
			int result = close_tmbs();

			return result;
		}

		public static int Get_CurrentBaud()
		{
			return get_current_baud();
		}

		public static int Init_Calibration(int axis)
		{
			int status = SIO_ERROR;
			if (get_tmbs_state() == TMBS_RUNNING)
			{
				Console.WriteLine("Tmbs is running, and the axis " + axis + " is good!");
				status = move_point(axis, -1000);
				Thread.Sleep(1000);
				move_abs(axis, -10000);
			}
			return status;
		}

		#endregion

		#region ############ STATES ############

		public static int Get_State()
		{
			return get_tmbs_state();
		}

		public static int Get_SIO_Error()
		{
			return get_sio_error();
		}

		public static int Check_Pfin(int axis)
		{
			return check_pfin(axis);
		}

		public static int Check_Status(int axis)
		{
			return check_status(axis);
		}

		public static int Check_Alarm(int axis)
		{
			return check_alrm(axis);
		}

		public static int Check_Son(int axis)
		{
			return check_son(axis);
		}

		public static int Reset_Alarm(int axis)
		{
			return reset_alarm(axis);
		}

		public static int Servo_On(int axis)
		{
			return set_son(axis);
		}

		public static int Servo_Off(int axis)
		{
			return set_soff(axis);
		}
		#endregion

		#region ############ MOTION ############
		public static int Move_Point(int axis, int point)
		{
			return move_point(axis, point);
		}

		public static int Move_Abs(int axis, int position)
		{
			return move_abs(axis, position);
		}

		public static int Move_Inc(int axis, int distance)
		{
			return move_inc(axis, distance);
		}

		public static int Move_ORG(int axis, int mode)
		{
			return move_org(axis, mode);
		}

		public static int Move_Rotate(int axis, int dir, int velocidad, int aceleracion)
		{
			return move_rotate(axis, dir, velocidad, aceleracion);
		}

		public static int Move_JOG(int axis, int distance)
		{
			return move_jog(axis, distance);
		}
		#endregion

		#region ############ PARAMETERS ############
		public static int Write_Velocity(int axis, int velocidad, int aceleracion)
		{
			return write_velocity(axis, velocidad, aceleracion);
		}

		public int Write_GainServo(int axis, int gain_sel, int svparm)
		{
			return select_svparm(axis, gain_sel, svparm);
		}

		public int Write_TrqLim(int axis, int atstop, int atmovement)
		{
			return write_trqlim(axis, atstop, atmovement);
		}

		public int Write_Tolerance(int axis, int width)
		{
			return write_inpos(axis, width);
		}

		public static int Read_Position(int axis)
		{
			int bank1 = Convert.ToInt16("29", 16);
			int bank2 = Convert.ToInt16("29", 16);
			int dataLoc = Convert.ToInt16("7400", 16);
			read_svmem(axis, dataLoc, ref bank1);
			int retVal = bank1;
			return retVal;
		}

		public int Read_Code_RZONE(int axis)
		{
			int bank1 = Convert.ToInt16("29", 16);
			int bank2 = Convert.ToInt16("12", 16);
			int dataLoc = Convert.ToInt16("5", 16);
			read_svmem(axis, dataLoc, ref bank1);
			int retVal = bank1;

			//write_svmem(axis, dataLoc, ref bank2);

			return retVal;
		}

		public int Read_Code_FZONE(int axis)
		{
			int bank1 = Convert.ToInt16("29", 16);
			int bank2 = Convert.ToInt16("12", 16);
			int dataLoc = Convert.ToInt16("4", 16);
			read_svmem(axis, dataLoc, ref bank1);
			int retVal = bank1;

			//write_svmem(axis, dataLoc, ref bank2);

			return retVal;
		}
		#endregion

		#region ############ OTHER METHODS ############

		#region ############ PARAM ############
		enum States { STAND_BY, ALARM, MOVING };
		static int currentAxis = 0;
		#endregion

		#region ############ METHODS ############

		/// <summary>
		/// Function used to communicate with the "ActuatorActions" class - Initialization
		/// </summary>
		/// <param name="comPort"></param>
		/// <returns></returns>
		public static bool Initialization_And_Check(string comPort)
		{
			if (Start_Communication(comPort, 100) == TMBS_RUNNING)
			{
				if (InitAxis())
				{
					return true;
				}
				else
				{
					return false;
				}
			}
			else
			{
				//Debug.WriteLine("");
				return false;
			}
		}

		/// <summary>
		/// Function used to communicate with the "ActuatorActions" class - Homming
		/// </summary>
		/// <returns></returns>
		public static bool Homming_AllAxis()
		{
			Thread.Sleep(10);
			if (Move_ORG(0, 8) == SIO_ERROR)
				return false;
			Thread.Sleep(10);
			if (Move_ORG(1, 8) == SIO_ERROR)
				return false;
			Thread.Sleep(10);
			if (Move_ORG(2, 8) == SIO_ERROR)
				return false;
			Thread.Sleep(500);
			return true;
		}

		/// <summary>
		/// 
		/// </summary>
		/// <param name="pulse0"></param>
		/// <param name="pulse1"></param>
		/// <param name="pulse2"></param>
		public static void Update_AllPositions(int pulse0, int pulse1, int pulse2)
		{
			Thread.Sleep(10);
			ActionAxis(0, pulse0);
			Thread.Sleep(10);
			ActionAxis(1, pulse1);
			Thread.Sleep(10);
			ActionAxis(2, pulse2);
		}

		/// <summary>
		/// Launch an action on the given axis in parameters (Check if it is ready to receive the order, and them submit the order)
		/// </summary>
		/// <param name="axis"></param>
		/// <param name="newPulse"></param>
		public static bool ActionAxis(int axis, int newPulse)
		{
			if (Check_Axis(axis))
			{
				Move_Abs(axis, newPulse);
				return true;
			}
			return false;
		}

		/// <summary>
		/// Checks if the axis is ready too receive an order
		/// </summary>
		/// <param name="axis"></param>
		/// <returns></returns>
		public static bool Check_Axis(int axis)
		{
			if (Check_And_Fix_Alarm(axis))
			{
				if (Check_Son(axis) == SIO_DONE)
				{
					return true;
				}
			}
			return false;
		}

		/// <summary>
		///  Checks and fix the alarm mode if needed on a specific axis 
		/// </summary>
		/// <param name="axis"> The axis on which we want to fix the alarm. </param>
		/// <returns> If the axis is good to use </returns>
		public static bool Check_And_Fix_Alarm(int axis)
		{
			bool result = true;
			if (Check_Alarm(axis) == SIO_DONE) // if there is an alarm on axis 0
			{
				result = false;
				set_soff(axis);
				Console.WriteLine("Alarm on axis 0!");
				Thread.Sleep(50);
				if (Reset_Alarm(axis) == SIO_DONE)
				{
					if (Check_Alarm(axis) != SIO_DONE)
					{
						result = true;
						Console.WriteLine("Alarm Clear");
					}
				}
				set_son(axis);
				Thread.Sleep(500);
				Move_ORG(axis, 8);
				Thread.Sleep(500);
			}
			return result;
		}

		/// <summary>
		/// Initialize the three axis of the actuator.
		/// </summary>
		/// <returns> If the whole initialization has been successful or not </returns>
		public static bool InitAxis()
		{
			bool result = true;
			for (int i = 0; i < 3; i++)
			{
				set_soff(i); set_son(i);        //makes sure each axis is ON and working
				if (!Check_And_Fix_Alarm(i))
				{
					Console.WriteLine("THE AXIS #" + i + " IS NOT WORKING");
					result = false;
				}
			}
			return result;
		}

		public static bool ActuatorLoop(int pulse0, int pulse1, int pulse2)
		{
			Thread.Sleep(8);
			ActionAxis(0, pulse0);
			Thread.Sleep(8);
			ActionAxis(1, pulse1);
			Thread.Sleep(8);
			ActionAxis(2, pulse2);

			return true;
		}

		#endregion

		#endregion

		#endregion

		#region &&&&&&&&&&&&&&&&& PRIVATE METHODS &&&&&&&&&&&&&&&&&
		private long getLongValue(string positionStr)
		{
			bool isNegative;
			string hexRep;
			long longRep;
			long position = Convert.ToInt64(positionStr, 10);

			if (position < 0)
				isNegative = true;
			else
				isNegative = false;

			if (isNegative)
			{
				//Convert to hex to view in word format
				hexRep = Convert.ToString(position, 16);
				//Get least significant double word
				hexRep = hexRep.Substring(hexRep.Length - 8, 8);
				//swap double words
				hexRep = hexRep.PadRight(16, '0');
				//convert to number
				longRep = Convert.ToInt64(hexRep, 16);
				return longRep;
			}
			else
			{
				//converts integer to a hex value
				hexRep = Convert.ToString(position, 16);
				//pad the value to ensure correct offset
				//since the positive numbers have leading 0's we do not need to fill the full 64 bits
				hexRep = hexRep + "00000000";
				longRep = Convert.ToInt64(hexRep, 16);
				return longRep;
			}
		}
		private int getIntValue(long position)
		{
			string binaryRep;
			string hexRep;
			int intRep;
			bool isNegative;
			string onesComp;
			long onesCompInt;

			binaryRep = Convert.ToString(position, 2);

			//Check to see that the length is 64 bits, and check the sign bit(msb) to see if its negative
			//In the event that the number is positive, the msb=0 and will normally be left out when
			//converted to an integer
			if (binaryRep.Length == 64 && binaryRep.Substring(0, 1) == "1")
				isNegative = true;
			else
			{
				//Check to see if the position is 0
				if (binaryRep.Length == 1)
					return 0;
				isNegative = false;
			}

			if (isNegative)
			{
				//To get actual decimal value, we must convert from the one's compliment representation
				//To ones compliment switch 0's to 1's and 1's to zeros, then add 1
				onesComp = binaryRep.Replace("0", "2");
				onesComp = onesComp.Replace("1", "0");
				onesComp = onesComp.Replace("2", "1");
				onesCompInt = Convert.ToInt64(onesComp, 2);
				onesCompInt = onesCompInt + 1;
				//This is now the unsigned value
				hexRep = Convert.ToString(onesCompInt, 16);
				//There is an extra word (8 bits) that we must remove
				hexRep = hexRep.Remove(hexRep.Length - 8, 8);
				intRep = Convert.ToInt16(hexRep, 16);
				//Add the sign back on
				intRep = (intRep * -1);
				return intRep;
			}
			else
			{
				//Convert the binary representation to HEX
				hexRep = Convert.ToString(Convert.ToInt64(binaryRep, 2), 16);
				//Remove the extra word (8 Bits)
				hexRep = hexRep.Remove(hexRep.Length - 8, 8);
				//Convert back to a decimal
				intRep = Convert.ToInt32(hexRep, 16);
				return intRep;
			}
		}
		#endregion
	}
}