//using System;
//using System.Collections.Concurrent;
//using System.Collections.Generic;
//using System.Linq;
//using System.Runtime.InteropServices;
//using System.Text;
//using System.Threading;
//using System.Threading.Tasks;

//    public static class MavlinkUtil {

//        static readonly byte[][] gcbuffer;
//        static readonly GCHandle[] handle;
//        static readonly SemaphoreSlim semaphore;
//        static readonly ConcurrentStack<int> freebuffers;
//        public static MAVLink.message_info GetMessageInfo(this MAVLink.message_info[] source, uint msgid) {
//            foreach (var item in source) {
//                if (item.msgid == msgid)
//                    return item;
//            }

//            Console.WriteLine("Unknown Packet " + msgid);
//            return new MAVLink.message_info();
//        }

//        public static T ByteArrayToStructureGC<T>(byte[] bytearray, Type type, int startoffset) where T : struct {
//            GCHandle gch = GCHandle.Alloc(bytearray, GCHandleType.Pinned);
//            try {
//                return (T)Marshal.PtrToStructure(new IntPtr(gch.AddrOfPinnedObject().ToInt64() + startoffset), typeof(T));
//            }
//            finally {
//                gch.Free();
//            }
//        }

//        public static object ByteArrayToStructureGC(byte[] bytearray, Type typeinfoType, byte startoffset, byte payloadlength) {
//            semaphore.Wait();
//            int bufferindex = 0;
//            if (freebuffers.TryPop(out bufferindex)) {
//                try {
//                    // copy it
//                    var len = Marshal.SizeOf(typeinfoType);
//                    if (len - payloadlength > 0)
//                        Array.Clear(gcbuffer[bufferindex], payloadlength, len - payloadlength);
//                    Buffer.BlockCopy(bytearray, startoffset, gcbuffer[bufferindex], 0, payloadlength);
//                    // structure it
//                    return Marshal.PtrToStructure(handle[bufferindex].AddrOfPinnedObject(), typeinfoType);
//                }
//                finally {
//                    freebuffers.Push(bufferindex);
//                    semaphore.Release();
//                }
//            }

//            semaphore.Release();

//            throw new InvalidOperationException("Failed to get free buffer");
//        }

//        public static byte[] trim_payload(ref byte[] payload) {
//            var length = payload.Length;
//            while (length > 1 && payload[length - 1] == 0) {
//                length--;
//            }
//            if (length != payload.Length)
//                Array.Resize(ref payload, length);
//            return payload;
//        }



//    public static byte[] StructureToByteArray(object obj) {
//            try {
//                // fix's byte arrays that are too short or too long
//                obj.GetType().GetFields()
//                    .Where(a => a.FieldType.IsArray && a.FieldType.UnderlyingSystemType == typeof(byte[]))
//                    .Where(a => {
//                        var attributes = a.GetCustomAttributes(typeof(MarshalAsAttribute), false);
//                        if (attributes.Length > 0) {
//                            MarshalAsAttribute marshal = (MarshalAsAttribute)attributes[0];
//                            int sizeConst = marshal.SizeConst;
//                            var data = ((byte[])a.GetValue(obj));
//                            if (data == null) {
//                                data = new byte[sizeConst];
//                            }
//                            else if (data.Length != sizeConst) {
//                                Array.Resize(ref data, sizeConst);
//                                a.SetValue(obj, data);
//                            }
//                        }

//                        return false;
//                    }).ToList();
//            }
//            catch { }

//            int len = Marshal.SizeOf(obj);
//            byte[] arr = new byte[len];
//            IntPtr ptr = Marshal.AllocHGlobal(len);
//            Marshal.StructureToPtr(obj, ptr, true);
//            Marshal.Copy(ptr, arr, 0, len);
//            Marshal.FreeHGlobal(ptr);
//            return arr;
//        }
//}
