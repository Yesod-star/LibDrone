using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;
using LibDrone;

namespace SimpleExample
{

    class Program
    {
        static Drone DroneAct = new Drone();
        static int port;
        static int frequency;
        static string connection;
        static string mode1;
        static int alt;
        static int coord1;
        static int coord2;

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {

            Console.WriteLine("Type the connection model(UDP, TCP or SERIAL):");
            connection = Console.ReadLine();
            Console.WriteLine("Type the number of the connection door:");
            port = Int32.Parse(Console.ReadLine());
            Console.WriteLine("Type the frequency of the connection:");
            frequency = Int32.Parse(Console.ReadLine());
            Console.WriteLine("Enter to connect");
            Console.ReadLine();
            DroneAct.but_connect_without_message(port,frequency,connection);
            Console.WriteLine("Enter to arm");
            Console.ReadLine();
            DroneAct.but_armdisarm();
            Console.WriteLine("Enter to enter flight mode:");
            Console.ReadLine();
            DroneAct.but_changeMode("GUIDED");
            Console.WriteLine("Type height for flight:");
            alt = Int32.Parse(Console.ReadLine());
            DroneAct.but_takeoff(80);
            Console.WriteLine("Type for model of flight(GUIDED, STABILIZED, LOITER, LAND, ALTHOLD or AUTO)");
            mode1 = Console.ReadLine();
            DroneAct.but_changeMode(mode1);
            Console.WriteLine("Enter to fly");
            Console.ReadLine();
            Console.WriteLine("Enter coord1");
            coord1 = Int32.Parse(Console.ReadLine());
            Console.WriteLine("Enter coord2");
            coord2 = Int32.Parse(Console.ReadLine());
            DroneAct.but_waypoint(coord1,coord2,alt);
            Console.WriteLine("Enter to land");
            Console.ReadLine();
            DroneAct.but_changeMode("LAND");
            DroneAct.but_land();
        }

    }
}
