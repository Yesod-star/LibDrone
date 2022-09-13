//private void but_mission_Click(object sender, EventArgs e)
//{
//    //Variavel para armazenar o comando2
//    MAVLink.mavlink_command_long_t req2 = new MAVLink.mavlink_command_long_t();

//    //Geracao do 2 comando
//    req2.target_system = 1;
//    req2.target_component = 1;

//    req2.command = (ushort)MAVLink.MAV_CMD.DO_SET_MODE;
//    req2.param1 = 3;
//    req2.param2 = 0;
//    req2.param3 = 0;

//    //Armazenamento do comando2
//    byte[] packet2 = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, req2);

//    //Envio do comando2
//    UdpSerialConnect1.Write(packet2, 0, packet2.Length);

//    //Ver se esta sendo aceito o comando
//    try
//    {
//        var ack = readsomedata<MAVLink.mavlink_command_ack_t>(sysid, compid);
//        if (ack.result == (byte)MAVLink.MAV_RESULT.ACCEPTED)
//        {

//        }
//    }
//    catch
//    {
//    }
//}

