/*
         T readsomedata<T>(byte sysid,byte compid,int timeout = 2000)
        {
            DateTime deadline = DateTime.Now.AddMilliseconds(timeout);

            lock (readlock)
            {
                // read the current buffered bytes
                while (DateTime.Now < deadline)
                {
                    var packet = mavlink.ReadPacket(UdpSerialConnect1.BaseStream);

                    // check its not null, and its addressed to us
                    if (packet == null || sysid != packet.sysid || compid != packet.compid)
                        continue;

                    Console.WriteLine(packet);

                    if (packet.data.GetType() == typeof (T))
                    {
                        return (T) packet.data;
                    }
                }
            }

            throw new Exception("No packet match found");
        }
 */