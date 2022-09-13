/*
         private void but_armdisarm_Click(object sender, EventArgs e)
        {
            //Variavel para armazenar o comando
            MAVLink.mavlink_command_long_t req = new MAVLink.mavlink_command_long_t();


            //Geracao do comando
            req.target_system = 1;
            req.target_component = 1;

            req.command = (ushort)MAVLink.MAV_CMD.COMPONENT_ARM_DISARM;

            req.param1 = armed ? 0 : 1;
            armed = !armed;




            //Armazenamento do comando
            byte[] packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, req);


            //Envio do comando
            UdpSerialConnect1.Write(packet, 0, packet.Length);


            //Ver se esta sendo aceito o comando
            try
            {
                var ack = readsomedata<MAVLink.mavlink_command_ack_t>(sysid, compid);
                if (ack.result == (byte)MAVLink.MAV_RESULT.ACCEPTED) 
                {

                }
            }
            catch 
            { 
            }


        }
 
 */