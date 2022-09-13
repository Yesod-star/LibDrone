/*
 * 
 *         MAVLink.MavlinkParse mavlink = new MAVLink.MavlinkParse();
        bool armed = false;
        // locking to prevent multiple reads on serial port
        object readlock = new object();
        // our target sysid
        byte sysid;
        // our target compid
        byte compid;
        //Connect via UDP
        UdpSerial UdpSerialConnect1 = new UdpSerial();
        //Connect via TCP
        TcpSerial TcpSerialConnect1 = new TcpSerial();
 
 private void but_connect_Click(object sender, EventArgs e)
        {
            UdpSerialConnect1.Port = "14551";
            UdpSerialConnect1.client = new UdpClient("127.0.0.1", 14551);

            //if the port is open close it
            if (UdpSerialConnect1.IsOpen)
            {
                UdpSerialConnect1.Close();
                return;
            }


            // set the comport options
            UdpSerialConnect1.PortName = "UDP14551";
            UdpSerialConnect1.BaudRate = int.Parse(cmb_baudrate.Text);
            // open the comport
            UdpSerialConnect1.Open();
            // set timeout to 2 seconds
            UdpSerialConnect1.ReadTimeout = 2000;

            BackgroundWorker bgw = new BackgroundWorker();

            bgw.DoWork += bgw_DoWork;

            bgw.RunWorkerAsync();
        }

        void bgw_DoWork(object sender, DoWorkEventArgs e)
        {
            while (UdpSerialConnect1.IsOpen)
            {
                try
                {
                    MAVLink.MAVLinkMessage packet;
                    lock (readlock)
                    {
                        // read any valid packet from the port
                        packet = mavlink.ReadPacket(UdpSerialConnect1.BaseStream);
                        
                        // check its valid
                        if (packet == null || packet.data == null)
                            continue;
                    }

                    // check to see if its a hb packet from the comport
                    if (packet.data.GetType() == typeof(MAVLink.mavlink_heartbeat_t))
                    {
                        var hb = (MAVLink.mavlink_heartbeat_t)packet.data;

                        // save the sysid and compid of the seen MAV
                        sysid = packet.sysid;
                        compid = packet.compid;

                        // request streams at 2 hz
                        var buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM,
                            new MAVLink.mavlink_request_data_stream_t()
                            {
                                req_message_rate = 2,
                                req_stream_id = (byte)MAVLink.MAV_DATA_STREAM.ALL,
                                start_stop = 1,
                                target_component = compid,
                                target_system = sysid
                            });

                        UdpSerialConnect1.Write(buffer, 0, buffer.Length);

                        buffer = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, hb);

                        UdpSerialConnect1.Write(buffer, 0, buffer.Length);
                    }

                    // from here we should check the the message is addressed to us
                    if (sysid != packet.sysid || compid != packet.compid)
                        continue;

                    Console.WriteLine(packet.msgtypename);
                    
                    if (packet.msgid == (byte)MAVLink.MAVLINK_MSG_ID.ATTITUDE)
                    //or
                    //if (packet.data.GetType() == typeof(MAVLink.mavlink_attitude_t))
                    {
                        var att = (MAVLink.mavlink_attitude_t)packet.data;

                        Console.WriteLine(att.pitch*57.2958 + " " + att.roll*57.2958);
                    }
                }
                catch
                {
                }

                System.Threading.Thread.Sleep(1);
            }
        }
 
 */