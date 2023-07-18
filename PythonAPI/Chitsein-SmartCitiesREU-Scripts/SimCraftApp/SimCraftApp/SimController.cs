using System.Collections.Generic;
using System.Threading;
using System;
using System.IO;
using System.Numerics;
using System.Timers;
using SCN6;
using System.Text;
using System.IO.Pipes;


namespace TRC.Services
{
	/// <summary>
	/// Controller of the IRL actuator which translate the movement of the virtual car
	/// in the real world
	/// </summary>
	public class SimController
	{
		#region Public Enum
		public enum ShakingPosition
		{
			Front, Rear, Both
		}
		#endregion

		#region Internal Structures
		internal struct ShakingOrder
		{
			public ShakingPosition pos;
			public bool shakeLeft;
			public bool shakeRight;
		}
		#endregion

		#region Private Fields
		// private GameObject driverCar;
		private float shakeAmplifierFactor = 1f;
		private float minimalVelocity = 0;
		private float maximalVelocity;
		private int pulseThreshold;
		private int minShakePulseOffset;
		private int maxShakePulseOffset;
		private int shakeOffset;
		private int numberOfPulses;
		private int pulseSleep;

		private Queue<ShakingOrder> shakingOrders;
		private EventWaitHandle semaphore;
		// private Rigidbody carRigidbody;
		private Vector3 angles;
		private Vector3 angless;
		private Thread actuatorThread;
		private Mutex mutex;
		private float memcurvel;
		private int[] previousPulses;
		private float currentAngularVelocity;
		private float currentVelocity;

		private float coefSens = 0.7f;
        private int coefVelo = 30;
        private int coefVeloBis = 100;

		private int tempCount = 0;

		private string MOTION_DATA_PIPE_FILE_PATH;
		#endregion

		#region Public Static Methods
		/// <summary>
		/// Add a new shaking order to the queue
		/// </summary>
		/// <param name="pos">Where the shaking must take place</param>
		/// <param name="shakeLeft">If we have to shake on the left side of the driver</param>
		/// <param name="shakeRight">If we have to shake on the right side of the driver</param>
		public void RequestShake(ShakingPosition pos, bool shakeLeft = true, bool shakeRight = true)
		{
			this.mutex.WaitOne();
			this.shakingOrders.Enqueue(new ShakingOrder()
			{
				pos = pos,
				shakeLeft = shakeLeft,
				shakeRight = shakeRight
			});
			this.mutex.ReleaseMutex();
		}
		#endregion

		#region Private Methods
		/// <summary>
		/// Actions to perform when the script is started
		/// </summary>
		public static void Main(String[] args)
		{
			if (args.Length > 0)
			{
				for (int i = 0; i < args.Length; i++)
				{
					Console.WriteLine(args[i]);
				}
			}
			
			
			SimController instance = new SimController();
			instance.previousPulses = new int[3];

			if (false)
			{
				instance.SetMotionFilePath(System.IO.Directory.GetCurrentDirectory() + "\\MOTION_DATA_PIPE.csv");
			}
			else
			{
				instance.SetMotionFilePath(System.IO.Directory.GetCurrentDirectory() + "\\MOTION_DATA_PIPE.csv");
			}

			instance.SetTimer();

			// We getting the required elements to open the actuator thread
			instance.shakingOrders = new Queue<ShakingOrder>();
			instance.semaphore = new EventWaitHandle(false, EventResetMode.ManualReset);
			// carRigidbody = driverCar.GetComponent<Rigidbody>();
			instance.angles = new Vector3(0f, 0f, 0f);
			instance.mutex = new Mutex();

			// We try to connect to the actuator through a serial connection
			int status = 0, count = 0, tries = 0;
			string[] ports = System.IO.Ports.SerialPort.GetPortNames();
			do
			{
				// For each port, we'll try 5 times to start a serial communictation
				status = SCN6_Driver.Start_Communication(ports[count], 3);
				if (status == SCN6_Driver.TMBS_NO_EXIST)
				{
					++count;
				}
				else if (status == SCN6_Driver.TMBS_INIT_ERROR)
				{
					if (++tries == 5)
					{
						++count;
						tries = 0;
					}
				}
			} while (status != SCN6_Driver.TMBS_RUNNING && count < ports.Length);

			// If we are not connected to the actuator, we stop the application
			if (status != SCN6_Driver.TMBS_RUNNING)
				System.Environment.Exit(1);

			// We are checking that each axis is good, and we set the initial velocity
			for (int i = 0; i < 3; ++i)
			{
				Console.WriteLine("Hello");
				SCN6_Driver.Check_And_Fix_Alarm(0);
				SCN6_Driver.Check_And_Fix_Alarm(1);
				SCN6_Driver.Check_And_Fix_Alarm(2);
				SCN6_Driver.Servo_On(i);
				SCN6_Driver.Move_ORG(i, 0x8);
				CheckActuatorStatus();
				SCN6_Driver.Write_Velocity(i, 1500, i == 2 ? 20 : 10);
				CheckActuatorStatus();
				SCN6_Driver.Move_Abs(i, 10000);
				CheckActuatorStatus();
				instance.previousPulses[i] = 10000;
				//VelocityButton.SitSpeed + 
			}
			SCN6_Driver.Write_Velocity(1, 2000, instance.coefVelo);
			SCN6_Driver.Write_Velocity(0, 2000, instance.coefVeloBis);
			SCN6_Driver.Write_Velocity(2, 2000, instance.coefVeloBis);
			// Finally, we create and launch the thread
			instance.actuatorThread = new Thread(new ThreadStart(instance.UpdateDevice));
			instance.actuatorThread.Start();

			
		}

		private static System.Timers.Timer aTimer;
		// Unity FixedUpdate runs at 50 times per second (every 20 milliseconds)
		private static int milliseconds = 20;
		public void SetTimer()
		{
			aTimer = new System.Timers.Timer(milliseconds);
			aTimer.Elapsed += OnTimedEvent;
			aTimer.AutoReset = true;
        	aTimer.Enabled = true;
		}

		private void OnTimedEvent(object sender, EventArgs e)
		{
			ReadMotionCSV();
			FixedUpdate();
		}

		/// <summary>
		/// Actions to perform each frame rendered
		/// </summary>
		private void FixedUpdate()
		{
			// We lock the mutex, then we set the important informations for the actuator thread
			mutex.WaitOne();
			// We retrieve the current angles of the driver car
			// ***angless = driverCar.transform.eulerAngles;*** -> SetAngles()
			if (angless.X < 180)
				angles = new Vector3(angless.X * this.coefSens, angless.Y, angless.Z);
			if (angless.X > 300)
				angles = new Vector3(360 - ((360 - angless.X) * (this.coefSens)), angless.Y, angless.Z); //+SensitivityButton.SitSensitivity

			// We adjust them to the actuator
			// UnityEngine.Console.WriteLine((angles).ToString());
			//UnityEngine.Console.WriteLine((CoefSensitivity.coefSens).ToString());
			angles.X = angles.X >= 270f ? Clamp(angles.X, 342.5f, 360f) : Clamp(angles.X, 0f, 17.5f);
			angles.Y = angles.Y >= 270f ? Clamp(angles.Y, 342.5f, 360f) : Clamp(angles.Y, 0f, 17.5f);
			angles.Z = angles.Z >= 270f ? Clamp(angles.Z, 342.5f, 360f) : Clamp(angles.Z, 0f, 17.5f);

			// Getting the y axis angular velocity and the velocity of the car
			// ***currentAngularVelocity = carRigidbody.angularVelocity.Y;*** -> SetAngularVelocityY() in radians!
			// ****currentVelocity = carRigidbody.velocity.magnitude;*** -> SetVelocity()
			mutex.ReleaseMutex();

			// Finally, we set the semaphore
			semaphore.Set();
		}

		/// <summary>
		/// Actions to perform when the application is terminated
		/// </summary>
		private void OnApplicationQuit()
		{
			// Setting the actuator to initial position
			if (SCN6_Driver.Get_State() == SCN6_Driver.TMBS_RUNNING) {
			    for (int i = 0 ; i < 3 ; ++i)
			        SCN6_Driver.Move_Abs(i, 10000);
			    CheckActuatorStatus();
			}

			// Closing serial communication and stopping actuator thread
			SCN6_Driver.Close_Communications();
			actuatorThread.Abort();
		}

		/// <summary>
		/// Routine to check if the actuator is allright
		/// </summary>
		private static void CheckActuatorStatus()
		{
			// Same as "while (true)"
			for (; ; )
			{
				if (SCN6_Driver.Check_Status(0) == SCN6_Driver.SIO_DONE && SCN6_Driver.Check_Status(1) == SCN6_Driver.SIO_DONE && SCN6_Driver.Check_Status(2) == SCN6_Driver.SIO_DONE)
				{
					if (SCN6_Driver.Check_Pfin(0) == SCN6_Driver.SIO_DONE && SCN6_Driver.Check_Pfin(1) == SCN6_Driver.SIO_DONE && SCN6_Driver.Check_Pfin(2) == SCN6_Driver.SIO_DONE)
						break;
				}

				for (int i = 0; i < 3; ++i)
					SCN6_Driver.Check_And_Fix_Alarm(i);

				Thread.Sleep(10);
			}
		}

		/// <summary>
		/// The function run by the actuator thread
		/// </summary>
		private void UpdateDevice()
		{
			// Same as "while (true)"
			for (; ; )
			{
				// We lock the semaphore before refreshing the state of the actuator
				semaphore.WaitOne();
				semaphore.Reset();

				// Locking mutex to avoid changes on working variables
				mutex.WaitOne();

				// We are checking that angular velocity is not out of the accepted range
				if (currentAngularVelocity > 6f)
					currentAngularVelocity = 6f;
				else if (currentAngularVelocity < -6f)
					currentAngularVelocity = -6f;

				// We get the pulses to apply to the actuator
				int[] pulses = new int[3] {
					(int) (AdaptAngle(angles.X) / .00175f),
					(int) (10000 - currentAngularVelocity * 10000 / 6),
					20000 - (int) (AdaptAngle(angles.Z) / .00175f)
				};
				// We release the mutex
				mutex.ReleaseMutex();

				//////////
				//if (shakingOrders.Count > 0) {
				// Console.WriteLine("\n velocite ; " + currentVelocity);
				//Console.WriteLine("\n minvelocite ; " + minimalVelocity);
				//Console.WriteLine("\n shakingOrders.Count ;  " + shakingOrders.Count);
				//}
				// If a shaking is requested and that the car is at the correct velocity
				if (shakingOrders.Count > 0 && currentVelocity >= minimalVelocity)
				{
					//Console.WriteLine("passed");
					//Console.WriteLine("\n velocite ; " + currentVelocity);
					//Console.WriteLine("\n velociteMIN ; " + minimalVelocity);
					mutex.WaitOne();
					ShakingOrder order = shakingOrders.Dequeue();
					//Console.WriteLine("release");
					mutex.ReleaseMutex();

					// Changing the velocity of the X axis
					SCN6_Driver.Write_Velocity(1, 2000, this.coefVelo);
					SCN6_Driver.Write_Velocity(0, 2000, this.coefVeloBis);
					SCN6_Driver.Write_Velocity(2, 2000, this.coefVeloBis); //VelocityButton.SitSpeed+


					// We compute the offset of the shaking
					// It depends of the speed of the car and and amplifier factor which set in the Unity editor
					// first formula to simulate vibration speed difference
					//int offset = (int) (shakeAmplifierFactor * ((currentVelocity - minimalVelocity) / (maximalVelocity - minimalVelocity))
					//    * (maxShakePulseOffset - minShakePulseOffset) + minShakePulseOffset);

					// formula to have more vibration differnce between  high speed and low speed shakeAmplifierFactor *
					int offset = (int)((currentVelocity) * (currentVelocity));
					if (offset > 200)
					{
						offset = 200;
					}
					Console.WriteLine("\n offset ; " + offset);
					Console.WriteLine("\n velocite ; " + currentVelocity);
					// Fosr now, the behavior for shaking only on a particular side is disabled because it was not realistic
					// You can improve this later though

					// Simulating the shaking of the front wheels
					if (order.pos == ShakingPosition.Front || order.pos == ShakingPosition.Both)
					{
						for (int i = 0; i < numberOfPulses; ++i)
						{
							Console.WriteLine("\n pulse :" + pulses[0] + "et move" + (pulses[0] - offset));
							SCN6_Driver.Move_Abs(1, pulses[0] - offset);

							for (; ; )
							{
								if (SCN6_Driver.Check_Status(1) == SCN6_Driver.SIO_DONE)
								{
									if (SCN6_Driver.Check_Pfin(1) == SCN6_Driver.SIO_DONE)
										break;
								}

								Thread.Sleep(pulseSleep);
							}

							offset = -offset;
						}
					}

					// Simulating the shaking on the rear wheels
					/*if (order.pos == ShakingPosition.Rear || order.pos == ShakingPosition.Both) {
                        for (int i = 0 ; i < numberOfPulses ; ++i) {
                            SCN6_Driver.Move_Abs(1, pulses[0] - shakeOffset - offset);
                            Console.WriteLine("\n pulse2 :" + pulses[0] + "et move2" + (pulses[0]- shakeOffset - offset));
                            for ( ; ; ) {
                                if (SCN6_Driver.Check_Status(1) == SCN6_Driver.SIO_DONE) {
                                    if (SCN6_Driver.Check_Pfin(1) == SCN6_Driver.SIO_DONE)
                                        break;
                                }

                                Thread.Sleep(pulseSleep);
                            }

                            offset = -offset;
                        }
                    }*/
				}
				else
				{
					// Setting the velocity for the X axis
					SCN6_Driver.Write_Velocity(1, 2000, this.coefVelo);
					SCN6_Driver.Write_Velocity(0, 2000, this.coefVeloBis);
					SCN6_Driver.Write_Velocity(2, 2000, this.coefVeloBis);//VelocityButton.SitSpeed+
				}

				// For each axis...
				// We start on axis 1, because :
				//  - Axis #0 => Z axis
				//  - Axis #1 => X axis
				//  - Axis #2 => Y axis
				if (currentVelocity > memcurvel)
				{
					Console.WriteLine("\n currentvelocity" + currentVelocity);
					Console.WriteLine("\n memcurvel ; " + memcurvel);
					Console.WriteLine("\n calcul ; " + (int)((currentVelocity - memcurvel) * 5000));
					Console.WriteLine("\n pulse avant" + pulses[0]);
					pulses[0] = pulses[0] + (int)((currentVelocity - memcurvel) * 5000);
					Console.WriteLine("\n pulse apres" + pulses[0]);
					memcurvel = currentVelocity;
				}

				int axis = 1;
				for (int i = 0; i < 3; i = i + 2)
				{
					// We apply the pulse if it is different enough from the previous pulse
					if (Math.Abs(pulses[i] - previousPulses[i]) >= pulseThreshold)
					{
						SCN6_Driver.Move_Abs(axis, pulses[i]);

						previousPulses[i] = pulses[i];
						axis = (axis + 2) % 3;
					}

					SCN6_Driver.Check_And_Fix_Alarm(i);
				}
			}
		}

		/// <summary>
		/// Adapt an angle value to correct value for the actuator
		/// </summary>
		/// <param name="value">The value of the angle to adapt in degree</param>
		/// <returns>The adapted value of the angle in degree</returns>
		private float AdaptAngle(float value)
		{
			return (value >= 0f && value <= 17.5f) ? 17.5f - value : 360f - value + 17.5f;
		}
		#endregion

		// Getters and setters

		// Coefficients
		public float GetCoefSens()
		{
			return this.coefSens;
		}

		public int GetCoefVelo()
		{
			return this.coefVelo;
		}

		public int GetCoefVeloBis()
		{
			return this.coefVeloBis;
		}

		public void SetCoefSens(float coef)
		{
			this.coefSens = coef;
		}

		public void SetCoefVelo(int coef)
		{
			this.coefVelo = coef;
		}

		public void SetCoefVeloBis(int coef)
		{
			this.coefVeloBis = coef;
		}

		// Input Values (angles, speed, etc.)

		
		private void ReadMotionCSV()
		{
			String path = MOTION_DATA_PIPE_FILE_PATH;

			if (tempCount >= 100)
				Console.WriteLine("Path at: " + path);

			/* Will contain:
			 * eulerZAxis, eulerXAxis, eulerYAxis, angularVelocityY, velocityMagnitude,
			in that order */

			// Pipeline?
			/* using (NamedPipeClientStream pipeClient =
				new NamedPipeClientStream(".", "Simcraft_Data_Pipeline", PipeDirection.InOut))
				{
					// Connect to the pipe or wait until the pipe is available.
					Console.Write("Attempting to connect to pipe...");
					pipeClient.Connect();

					Console.WriteLine("Connected to pipe.");
					Console.WriteLine("There are currently {0} pipe server instances open.",
						pipeClient.NumberOfServerInstances);
					StreamWriter swrite = new StreamWriter(pipeClient);
					StreamReader sread = new StreamReader(pipeClient);
					while (true)
					{
						try
						{
							swrite.AutoFlush = true;
							swrite.WriteLine("Hello from c#");
							//  Console.WriteLine("Received: " + temp);
							string temp = sread.ReadLine();
							Console.WriteLine("Received from python : {0}", temp);
							// Console.ReadLine();
						}
						catch (EndOfStreamException)
						{
							break;                    // When client disconnects
						}
					}

				}*/


			StreamReader sr;
			List<string> values = new List<string>();

			if (File.Exists(path))
			{
				// Open the file such that it does not lock the file and can be written by Python script
				sr = new StreamReader(File.Open(path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite));
				String data;
				char[] separators = { ',' };
				int line = 1;

				// Read all values in the CSV file and add them to a list of strings
				if (tempCount >= 100)
					Console.WriteLine();
				while (!sr.EndOfStream)
				{
					data = sr.ReadLine();
					string[] v = data.Split(separators);
					for (int index = 0; index < v.Length; index++)
					{
						// Add those values to the list ('vaues')
						if (line == 2)
						{
							values.Add(v[index]);
						}
						if (tempCount >= 100)
							Console.Write(" " + v[index]);
					}
					if (tempCount >= 100)
						Console.WriteLine();
					line++;
				}
				sr.Close();
			}
			else
			{
				Console.WriteLine("Failed to find path at the set location");
				Environment.Exit(1);
			}
			if (tempCount >= 100)
				Console.WriteLine();

			/* Input the values for eulerZAxis, eulerXAxis, eulerYAxis, angularVelocityY, and velocity -> ***AXES ARE DEFINED BY CARLA DOCS***
			 * ***CSV FILE FORMAT BELOW*** (Subject to change, always check CSV File under MOTION_DATA_PIPE_FILE_PATH to corroborate)
			 * Line 1: "eulerZAxis", "eulerXAxis", "eulerYAxis", "angularVelocityY", "velocity"
			 * Line 2: eulerZAxis value, eulerXAxis value, eulerYAxis value, angularVelocityY value, velocity value
			 * 
			 * Below, we convert the string values from line2 to 
			*/
			if (values.Count < 5)
			{
				Console.WriteLine("Not all values received");
			}
			else
			{
				float pitch = float.Parse(values[0]);
				float yaw = float.Parse(values[1]);
				float roll = float.Parse(values[2]);
				float angularVelocityY = float.Parse(values[3]);
				// Angular velocity needs to be converted to radians!!
				angularVelocityY = angularVelocityY * 2 * (float) Math.PI / 360;
				float velocity = float.Parse(values[4]);
				// Convert said angles from Carla/Unreal Engine coordinate system to Unity coordinate system -> Unity is [Z forward, X right, Y up], Unreal is [X forward, Y right, Z up]

				// Also must be from range 0 to 360!
				pitch *= -1;
				yaw *= -1;
				roll *= -1;
				if (pitch < 0)
					pitch += 360;
				if (yaw < 0)
					yaw += 360;
				if (roll < 0)
					roll += 360;


				SetAngles(pitch, yaw, roll); // Sets angle such that it is represented like Unity EulerAngle (forward, right, up)
				SetAngularVelocityY(angularVelocityY);
				SetVelocity(velocity * 5);
				if (tempCount >= 100)
					Console.WriteLine("Set values from CSV file in SimController variables");
			}
			if (tempCount >= 100)
				tempCount = 0;
			else
				tempCount++;

		}

		public void SetAngles(float eulerXAxis, float eulerYAxis, float eulerZAxis)
		{
			angless = new Vector3(eulerXAxis, eulerYAxis, eulerZAxis);
		}

		public void SetAngularVelocityY(float angularVelocityY)
		{
			currentAngularVelocity = angularVelocityY;
		}

		public void SetVelocity(float velocity)
		{
			currentVelocity = velocity;
		}

		private void SetMotionFilePath(string path)
		{
			MOTION_DATA_PIPE_FILE_PATH = path;
		}

		// Misc. Methods

		public float Clamp(float value, float min, float max)
		{
			if (min > max)
			{
				return min;
			}
			else if (value < min)
			{
				return min;
			}
			else if (value > max)
			{
				return max;
			}
			else
			{
				return value;
			}
		}
	}
}
