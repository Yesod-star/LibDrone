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

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {

            Console.WriteLine("Digite o modelo de Conexão(UDP, TCP ou SERIAL):");
            connection = Console.ReadLine();
            Console.WriteLine("Digite o numero da porta da Conexão:");
            port = Int32.Parse(Console.ReadLine());
            Console.WriteLine("Digite a frequência da Conexão:");
            frequency = Int32.Parse(Console.ReadLine());
            Console.WriteLine("Aperte para se Conectar");
            Console.ReadLine();
            DroneAct.but_connect_without_message(port,frequency,connection);
            Console.WriteLine("Aperte para se Armar");
            Console.ReadLine();
            DroneAct.but_armdisarm();
            Console.WriteLine("Digite a altura do voo:");
            alt = Int32.Parse(Console.ReadLine());
            DroneAct.but_changeMode("GUIDED");
            DroneAct.but_takeoff(alt);
            Console.WriteLine("Digite o modo do Drone(GUIDED, STABILIZED, LOITER, LAND, ALTHOLD ou AUTO)");
            mode1 = Console.ReadLine();
            DroneAct.but_changeMode(mode1);
            Console.WriteLine("Aperte para voar");
            Console.ReadLine();
            DroneAct.but_waypoint();
            Console.WriteLine("Aperte para pousar");
            Console.ReadLine();
            DroneAct.but_changeMode("LAND");
            DroneAct.but_land();
        }

    }
}
