using CyUSB;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Linq;

namespace fx2sharp
{

    public enum FirmwareConfigDirection
    {
        None,
        In,
        Out
    }

    public enum FirmwareConfigIFClock
    {
        None,
        Internal30,
        Internal48,
        External
    }

    public enum FirmwareConfigFifoMode
    {
        None,
        Sync,
        Async
    }

    public enum FirmwareConfigBuffering
    {
        None,
        Double,
        Triple,
        Quadruple
    }

    public enum FirmwareConfigCPUClock
    {
        None,
        CPU12,
        CPU24,
        CPU48
    }


    public class bufandlen {
        public byte[] buf;
        public int len;
    }

    public class FirmwareConfig
    {
        public FirmwareConfigDirection Direction { get; set; }
        public FirmwareConfigIFClock IFClock { get; set; }

        public bool IFClockOut { get; set; }
        public bool IFClockInvert { get; set; }

        public FirmwareConfigCPUClock CPUClock { get; set; }

        public bool CPUClockOut { get; set; }
        public bool CPUClockInvert { get; set; }


        public FirmwareConfigFifoMode FifoMode { get; set; }
        public FirmwareConfigBuffering FifoBufferMode { get; set; }

        public bool Fifo8bit { get; set; }

        public FirmwareConfig()
        {
            Direction = FirmwareConfigDirection.In;
        }

        public byte[] ToBytes()
        {
            byte[] b = new byte[5];

            const int FC_DIR = 0;
            const int FC_IFCONFIG = 1;
            const int FC_EPCFG = 2;
            const int FC_EPFIFOCFG = 3;
            const int FC_CPUCS = 4;

            switch (Direction)
            {
                case FirmwareConfigDirection.In:
                    b[FC_DIR] = 0x12;
                    b[FC_EPCFG] = 0xE0;
                    b[FC_EPFIFOCFG] = 0x0C;
                    break;
                case FirmwareConfigDirection.Out:
                    b[FC_DIR] = 0x12;
                    b[FC_EPCFG] = 0xA0;
                    b[FC_EPFIFOCFG] = 0x10;
                    break;
                default:
                    throw new ArgumentException("Missing direction", nameof(Direction));
            }

            switch (CPUClock)
            {
                case FirmwareConfigCPUClock.CPU12:
                    b[FC_CPUCS] = 0x00;
                    break;
                case FirmwareConfigCPUClock.CPU24:
                    b[FC_CPUCS] = 0x08;
                    break;
                case FirmwareConfigCPUClock.CPU48:
                    b[FC_CPUCS] = 0x10;
                    break;
            }

            if (CPUClockInvert)
                b[FC_CPUCS] |= 0x04;

            if (CPUClockOut)
                b[FC_CPUCS] |= 0x02;

            switch (IFClock)
            {
                case FirmwareConfigIFClock.External:
                    b[FC_IFCONFIG] = 0x00;
                    break;
                case FirmwareConfigIFClock.Internal30:
                    b[FC_IFCONFIG] = 0x80;
                    break;
                case FirmwareConfigIFClock.Internal48:
                    b[FC_IFCONFIG] = 0xC0;
                    break;
                default:
                    throw new ArgumentException("Missing IF Clock", nameof(IFClock));
            }

            switch (FifoMode)
            {
                case FirmwareConfigFifoMode.Async:
                    b[FC_IFCONFIG] |= 0x08;
                    break;
                case FirmwareConfigFifoMode.Sync:
                    b[FC_IFCONFIG] |= 0x00;
                    break;
                default:
                    throw new ArgumentException("Missing FifoMode", nameof(FifoMode));
            }

            if (IFClockInvert)
                b[FC_IFCONFIG] |= 0x10;

            if (IFClockOut)
                b[FC_IFCONFIG] |= 0x20;

            b[FC_IFCONFIG] |= 0x03;    // bits 1,0 = 11 for slave FIFO mode

            switch (FifoBufferMode)
            {
                case FirmwareConfigBuffering.Double:
                    b[FC_EPCFG] |= 0x02;
                    break;
                case FirmwareConfigBuffering.Triple:
                    b[FC_EPCFG] |= 0x03;
                    break;
                case FirmwareConfigBuffering.Quadruple:
                    b[FC_EPCFG] |= 0x00;
                    break;
                default:
                    throw new ArgumentException("Missing FifoBufferMode", nameof(FifoBufferMode));
            }

            if (!Fifo8bit)
            {
                b[FC_EPFIFOCFG] |= 0x01;
            }

            return b;
        }
    };



    class Program
    {
        const int E_OK = 0;
        const int E_ENDPOINT = 95;
        const int E_CFG_FAIL = 96;
        const int E_GO_FAIL = 97;
        const int E_RESET_FAIL = 98;
        const int E_FIRMWARE = 99;
        const int E_NODEV = 100;
        const int E_UNEX = 101;

        const byte FX2_RESET_RESET = 1;
        const byte FX2_RESET_GO = 0;
        const ushort FX2_RESET_A = 0xe600;

        const ushort FW_CFG_A = 0x1003;

        const byte EP_ALT_IF = 0x01;

        const byte EP_BULK_IN = 0x86;
        const byte EP_BULK_OUT = 0x02;


        static bool Cancelled = false;


        static int Main(string[] args)
        {

            Console.CancelKeyPress += delegate (object sender, ConsoleCancelEventArgs e)
            {
                e.Cancel = true;
                Cancelled = true;
            };

            try
            {

                using (var usbDevices = new USBDeviceList(CyConst.DEVICES_CYUSB))
                {
                    using (CyFX2Device fx2 = Connect(usbDevices))
                    {
                        if (fx2 != null)
                        {
                            Console.WriteLine($"Found: {fx2.ToString()} ");


                            Console.WriteLine("RESET");
                            //reset device
                            if (!WriteReset(fx2, true))
                            {
                                Console.Error.WriteLine("RESET failed");
                                return E_RESET_FAIL;
                            }
                            Thread.Sleep(500);


                            Console.WriteLine("UPLOAD FIRMWARE");
                            string firmname = Path.Combine(
                                    Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().Location),
                                    "fx2pipe_hoglet.hex"
                                    );
                            //upload firmware
                            if (!fx2.LoadRAM(firmname))
                            {
                                Console.Error.WriteLine("Failed to program Firmware");
                                return E_FIRMWARE;
                            }


                            Console.WriteLine("RESTART");
                            //reset device
                            if (!WriteReset(fx2, false))
                            {
                                Console.Error.WriteLine("RESTART failed");
                                return E_RESET_FAIL;
                            }
                            Thread.Sleep(500);

                        }
                        if (fx2 != null)
                        {
                            Console.WriteLine($"Found: {fx2.ToString()} ");

                            Console.WriteLine("RESET");
                            //reset device
                            if (!WriteReset(fx2, true))
                            {
                                Console.Error.WriteLine("RESET failed");
                                return E_RESET_FAIL;
                            }
                            Thread.Sleep(500);


                            //configure firmware
                            FirmwareConfig cfg = new FirmwareConfig();
                            cfg.CPUClock = FirmwareConfigCPUClock.CPU48;
                            cfg.CPUClockInvert = false;
                            cfg.CPUClockOut = true;
                            cfg.Direction = FirmwareConfigDirection.In;
                            cfg.Fifo8bit = false;
                            cfg.FifoBufferMode = FirmwareConfigBuffering.Double;
                            cfg.FifoMode = FirmwareConfigFifoMode.Async;
                            cfg.IFClock = FirmwareConfigIFClock.Internal48;
                            cfg.IFClockInvert = false;
                            cfg.IFClockOut = false;


                            Console.WriteLine("UPLOAD CONFIG");
                            //byte[] cfgbytes = cfg.ToBytes();
                            //byte[] cfgbytes = new byte[] { 0x21, 0xcb, 0xa0, 0x11, 0x12 }; //hog
                            //byte[] cfgbytes = new byte[] { 0x12, 0xcb, 0xae, 0x0d, 0x12 }; //elm
                            byte[] cfgbytes = new byte[] { 0x12, 0xcb, 0xe0, 0x0d, 0x12 }; //hog2
                            Console.WriteLine($"CFG: {cfgbytes[0]:X2}:{cfgbytes[1]:X2}:{cfgbytes[2]:X2}:{cfgbytes[3]:X2}:{cfgbytes[4]:X2}");
                            if (!WriteRAM(fx2, FW_CFG_A, cfgbytes))
                            {
                                Console.Error.WriteLine("Write config failed");
                                return E_CFG_FAIL;
                            }

                            WriteRAM(fx2, 0x1020, new byte[] { 1, 2, 3, 4, 5, 6 });

                            DumpMem(fx2, 0x1000, 0x10);
                            DumpMem(fx2, 0x1010, 0x10);
                            DumpMem(fx2, 0x1020, 0x10);
                            DumpMem(fx2, 0x1030, 0x10);

                            DumpMem(fx2, 0x1000, 0x40);

                            DumpMem(fx2, 0xE600, 0x40);

                            DumpMem(fx2, 0x0000, 0x080);


                            Console.WriteLine("RESTART DEV");
                            //restart device
                            if (!WriteReset(fx2, false))
                            {
                                Console.Error.WriteLine("START failed");
                                return E_GO_FAIL;
                            }

                            Console.WriteLine("Wait for restart...");
                            Thread.Sleep(1000);
                        }
                        else
                        {
                            Console.Error.WriteLine("Cannot find fx2 device");
                            return E_NODEV;
                        }

                        fx2.AltIntfc = 1;
                        CyBulkEndPoint e = fx2.EndPointOf(EP_BULK_IN) as CyBulkEndPoint;

                        //CyBulkEndPoint e = fx2.EndPointOf(EP_BULK_IN) as CyBulkEndPoint;

                        if (e == null)
                        {
                            Console.Error.WriteLine("Cannot get endpoint");
                            return E_ENDPOINT;
                        }


                        const int BUFFERSMAX = 1000;
                        const int BUFFERSIZE = 512;
                        const int DISCARD = 10;
                        const int MAX = 10000;
                        const int BUFFERSMIN = 16;

                        //                        ConcurrentBag<byte[]> bufferPool = new ConcurrentBag<byte[]>();
                        Queue<bufandlen> bufferPool = new Queue<bufandlen>();
                        for (int i = 0; i < BUFFERSMIN; i++)
                        {
                            bufferPool.Enqueue(new bufandlen { buf = new byte[BUFFERSIZE], len = 0 });
                        }

                        BlockingCollection<bufandlen> bc = new BlockingCollection<bufandlen>(BUFFERSMAX);

                        long count = 0;
                        int bcmax = 0;
                        int szmin = BUFFERSIZE;
                        int extcount = 0;

                        //start a pair of Tasks, one to read from USB, the other to write
                        var prodThread = new Thread(() =>
                        {
                            Thread.CurrentThread.Priority = ThreadPriority.Highest;
                            int ctr = 0;
                            while (!Cancelled && ctr < MAX)
                            {

                                bufandlen buf = null;

                                /*if (!bufferPool.TryTake(out buf))
                                {
                                    buf = new byte[BUFFERSIZE];
                                }*/
                                lock (bufferPool)
                                {
                                    buf = bufferPool.FirstOrDefault();
                                }
                                if (buf == null)
                                {
                                    buf = new bufandlen { buf = new byte[BUFFERSIZE] };
                                    extcount++;
                                }

                                buf.len = BUFFERSIZE;
                                int len2 = BUFFERSIZE;

                                if (!e.XferData(ref buf.buf, ref buf.len))
                                {
                                    Console.Write("TO");                                    
                                }
                                else
                                {
                                    if (buf.len != 0 && ctr > DISCARD)
                                    {
                                        bc.Add(buf);
                                        len2 = buf.len;
                                    }
                                    ctr++;
                                }
                                int bcc = bc.Count;
                                if (bcc > bcmax)
                                    bcmax = bcc;
                                if (len2 != 0 && len2 < szmin)
                                    szmin = len2;
                            }
                            bc.CompleteAdding();
                        });

                        var consThread = new Thread(() =>
                        {
                            Console.WriteLine("STREAMING...");
                            using (var f = new FileStream("d:\\temp\\test.bin", FileMode.Create, FileAccess.Write))
                            {

                                foreach (var tp in bc.GetConsumingEnumerable())
                                {

                                    f.Write(tp.buf, 0, tp.len);
                                    count += tp.len;
                                    lock(bufferPool)
                                        bufferPool.Enqueue(tp);
                                }

                            }
                        });

                        consThread.Start();
                        Thread.Sleep(10);
                        prodThread.Start();

                        prodThread.Join();
                        consThread.Join();

                        Console.WriteLine($"Wrote {count} bytes [{bcmax} {szmin} {extcount}]");

                    }
                }


                Console.WriteLine("OK!");
                Console.ReadLine();
                return E_OK;
            }
            catch (Exception ex)
            {
                Console.Error.WriteLine("Unexpected error:" + ex.ToString());
                return E_UNEX;
            }
        }

        static byte[] ReadRAM(CyFX2Device fx2, ushort addr, int len)
        {

            fx2.ControlEndPt.Target = CyConst.TGT_DEVICE;
            fx2.ControlEndPt.ReqType = CyConst.REQ_VENDOR + 0x80;
            fx2.ControlEndPt.ReqCode = 0xA0;
            fx2.ControlEndPt.Value = addr;
            fx2.ControlEndPt.Index = 0;

            byte[] ret = new byte[len];
            if (!fx2.ControlEndPt.Read(ref ret, ref len))
                throw new Exception("ReadRAM failed");
            return ret;

        }


        static bool WriteRAM(CyFX2Device fx2, ushort addr, byte[] data)
        {

            fx2.ControlEndPt.Target = CyConst.TGT_DEVICE;
            fx2.ControlEndPt.ReqType = CyConst.REQ_VENDOR;
            fx2.ControlEndPt.ReqCode = 0xA0;
            fx2.ControlEndPt.Value = addr;
            fx2.ControlEndPt.Index = 0;

            int l = data.Length;
            if (!fx2.ControlEndPt.Write(ref data, ref l))
                throw new Exception("WriteRAM failed");


            return l == data.Length;
        }

        static bool WriteReset(CyFX2Device fx2, bool reset)
        {
            return WriteRAM(fx2, FX2_RESET_A, new byte[] { (reset) ? (byte)0x01 : (byte)0x00 });

        }

        static CyFX2Device Connect(USBDeviceList usbDevices)
        {
            CyFX2Device fx2 = null;

            foreach (USBDevice u in usbDevices)
            {
                if (u is CyFX2Device)
                {
                    if (fx2 == null)
                        fx2 = u as CyFX2Device;
                    else
                        throw new Exception("More than one!");
                }
            }
            return fx2;
        }

        static void DumpMem(CyFX2Device fx2, ushort addr, int len)
        {
            //read memory and dump 
            byte[] b = ReadRAM(fx2, addr, len);
            for (int i = 0; i < b.Length; i++)
            {
                if (i % 16 == 0)
                {
                    Console.WriteLine();
                    Console.Write($"{(addr + i):X4} : ");
                }
                Console.Write($" {b[i]:X2}");
            }
            Console.WriteLine();
        }

    }
}
