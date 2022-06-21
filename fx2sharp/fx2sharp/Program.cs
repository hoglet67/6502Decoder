using CyUSB;
using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Threading;

namespace fx2sharp
{

    public class CmdLineArgumentException : ArgumentException
    {
        public CmdLineArgumentException(string message, string argument)
            : base(message, argument)
        {
        }
    }

    public class FX2SharpException : Exception
    {
        public int ReturnCode { get; }

        public FX2SharpException(string message, int returnCode)
            : base(message)
        {
            ReturnCode = returnCode;
        }

    }

    public enum FirmwareConfigDirection
    {
        In,
        Out
    }

    public enum FirmwareConfigIFClock
    {
        Internal30,
        Internal48,
        External
    }

    public enum FirmwareConfigFifoMode
    {
        Sync,
        Async
    }

    public enum FirmwareConfigBuffering
    {
        Double,
        Triple,
        Quadruple
    }

    public enum FirmwareConfigCPUClock
    {
        CPU12,
        CPU24,
        CPU48
    }

    public class bufandlen
    {
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
        const int E_ARGUMENT = 1;
        const int E_USER_BREAK = 2;
        const int E_ENDPOINT = 95;
        const int E_CFG_FAIL = 96;
        const int E_GO_FAIL = 97;
        const int E_RESET_FAIL = 98;
        const int E_FIRMWARE = 99;
        const int E_NODEV = 100;
        const int E_UNEX = 101;
        const int E_BUFFERS_EXHAUSTED = 102;
        const int E_TIMEOUT = 103;

        const byte FX2_RESET_RESET = 1;
        const byte FX2_RESET_GO = 0;
        const ushort FX2_RESET_A = 0xe600;

        const ushort FW_CFG_A = 0x1003;

        const byte EP_ALT_IF = 0x01;

        const byte EP_BULK_IN = 0x86;
        const byte EP_BULK_OUT = 0x02;

        static CancellationTokenSource ct_source = new CancellationTokenSource();
        static CancellationToken ct_token = ct_source.Token;

        static void Usage(TextWriter sw)
        {
            sw.Write("USAGE: fx2pipe [option|assignment...] <input/output filename>\n"
        + "Long options:\n"
        + "  --help       print this\n"
        + "  --version    print version information\n"
        + "  --debug      print debugging info\n"
        + "Short options: (Can be compined into single option)\n"
        + "  -i           run in IN direction, i.e. read data from USB EP6 (default)\n"
        + "  -o           run in OUT direction, i.e. write data to USB EP2\n"
        + "  -0           no stdio; send NULs / throw away read data (for timing)\n"
        + "  -8,-w        use 8bit / 16bit (default) wide fifo bus on FX2 side\n"
        + "  -2,-3,-4     use double, triple or quad (default) buffered FX2 fifo\n"
        + "  -s,-a        run in sync (default) / async slave fifo mode\n"
        + "  -O,-I        shortcut for -o0, -i0, respectively\n"
        + "Assignments: (Leading '-' can be left away)\n"
        + "  -d=NN        use n-th (unconfigured) FX2 device (start with NN=0, default)\n"
        + "  -d=VID:PID[:N] use N-th device with specified VID and PID\n"
        + "  -n=NNN       stop after NNN bytes; suffix k,M,G for mult. with 2^10,20,30\n"
        + "  -bs=NNN      set IO block size to NNN, max 16384 (default 16384)\n"
        + "  -ps=NN       set pipeline size (number of URBs; default 16)\n"
        + "  -bufs=NN     number of spare buffers (default 64)\n"
        //        + "  -sched=P[,N] set scheduling policy P (\"fifo\" or \"rr\") and prio N\n"
        + "  -fw=PATH     use specified firmware IHX file instead of built-in one\n"
        + "               omit path to not download any firmware (just reset device)\n"
        + "  -ifclk=[x|30[o]|48[o]][i] specify interface clock:\n"
        + "               x -> external; 30,48 -> internal clock 30/48MHz, suffix 'o'\n"
        + "               to enable output to IFCLK pin, 'i' to invert IFCLK\n"
        + "  -cko=[12|24|48][o|z][i] specify 8051 frequency in MHz (default: 48) and\n"
        + "               CLKOUT pin: output 'o' (default), tristate 'z', invert 'i'\n"
        + "  -discard=NN  Number of bytes to discard when reading from fifos (default 256k)\n"
        + "               ; suffix k,M,G for mult. with 2^10,20,30\n"
        + "\n"
        + "input/output filename can be specified as -- for stdin/stdout"
        + "\n"
        + "fx2sharp - pipe data in or out of an Cypress FX2 device (CY7C6801x[A])\n"
        + "\n"
        + "Copyright(c) 2020 by Dominic Beesley & David Banks; License: GPL\n"
        + "\n"
        + "based on fx2pipe: Copyright (c) 2006--2011 by Wolfgang Wieser; License: GPL\n");
        }

        static void Version(TextWriter sw)
        {

            string githash = GetGitHash();

            githash = (githash == null) ? "" : " --" + githash;

            sw.WriteLine($"Version: 0.01{githash}");


        }

        /// <summary> Gets the git hash value from the assembly
        /// or null if it cannot be found. </summary>
        public static string GetGitHash()
        {
            var asm = typeof(Program).Assembly;
            var attrs = asm.GetCustomAttributes<AssemblyMetadataAttribute>();
            return attrs.FirstOrDefault(a => a.Key == "GitHash")?.Value;
        }

        static long readKMG(string parm)
        {
            long mult = 1;
            if (parm.ToLower().EndsWith("k"))
            {
                mult = 1024;
                parm = parm.Substring(0, parm.Length - 1);
            }
            else if (parm.ToLower().EndsWith("m"))
            {
                mult = 1024 * 1024;
                parm = parm.Substring(0, parm.Length - 1);
            }
            if (parm.ToLower().EndsWith("g"))
            {
                mult = 1024 * 1024 * 1024;
                parm = parm.Substring(0, parm.Length - 1);
            }

            long v;
            if (!long.TryParse(parm, out v))
                throw new CmdLineArgumentException($"Cannot parse \"{parm}\" as a number", "-n");
            else
                return v * mult;

        }

        static unsafe int Main(string[] args)
        {
            try
            {
                bool noIO = false;
                string devSpec = "";
                long captureLimit = -1;
                int blocksize = 16384;
                int uiblen = 16;
                string firmwarepath = Path.Combine(
                    Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().Location),
                    "fx2pipe_hoglet.hex"
                );
                Stream fs = null;
                bool debug = false;
                int bufs = 64;
                long discard = 256*1024;

                //configure firmware
                FirmwareConfig cfg_fw = new FirmwareConfig();
                cfg_fw.CPUClock = FirmwareConfigCPUClock.CPU48;
                cfg_fw.CPUClockInvert = false;
                cfg_fw.CPUClockOut = true;
                cfg_fw.Direction = FirmwareConfigDirection.In;
                cfg_fw.Fifo8bit = false;
                cfg_fw.FifoBufferMode = FirmwareConfigBuffering.Quadruple;
                cfg_fw.FifoMode = FirmwareConfigFifoMode.Async;
                cfg_fw.IFClock = FirmwareConfigIFClock.Internal48;
                cfg_fw.IFClockInvert = false;
                cfg_fw.IFClockOut = false;

                // Decode command line parameters
                int i = 0;
                while (i < args.Length && args[i].StartsWith("-"))
                {
                    string arg = args[i];
                    string sw = arg;
                    string parm = "";
                    int ie = sw.IndexOf("=");
                    if (ie >= 0)
                    {
                        parm = sw.Substring(ie + 1).Trim();
                        sw = sw.Substring(0, ie).Trim();
                    }

                    switch (sw)
                    {
                        case "--help":
                            Usage(Console.Out);
                            return E_OK;
                        case "--version":
                            Version(Console.Out);
                            return E_OK;
                        case "--debug":
                            debug = true;
                            break;
                        case "-i":
                            cfg_fw.Direction = FirmwareConfigDirection.In;
                            break;
                        case "-I":
                            cfg_fw.Direction = FirmwareConfigDirection.In;
                            noIO = true;
                            break;
                        case "-o":
                            cfg_fw.Direction = FirmwareConfigDirection.Out;
                            break;
                        case "-O":
                            cfg_fw.Direction = FirmwareConfigDirection.Out;
                            noIO = true;
                            break;
                        case "-0":
                            noIO = true;
                            break;
                        case "-8":
                            cfg_fw.Fifo8bit = true;
                            break;
                        case "-w":
                            cfg_fw.Fifo8bit = false;
                            break;
                        case "-2":
                            cfg_fw.FifoBufferMode = FirmwareConfigBuffering.Double;
                            break;
                        case "-3":
                            cfg_fw.FifoBufferMode = FirmwareConfigBuffering.Triple;
                            break;
                        case "-4":
                            cfg_fw.FifoBufferMode = FirmwareConfigBuffering.Quadruple;
                            break;
                        case "-s":
                            cfg_fw.FifoMode = FirmwareConfigFifoMode.Sync;
                            break;
                        case "-a":
                            cfg_fw.FifoMode = FirmwareConfigFifoMode.Async;
                            break;
                        case "-d":
                            devSpec = parm;
                            break;
                        case "-n":
                            if (parm == "")
                                captureLimit = -1;
                            else
                                captureLimit = readKMG(parm);
                            break;
                        case "-discard":
                            if (parm == "")
                                discard = -1;
                            else
                                discard = readKMG(parm);
                            break;
                        case "-bs":
                            if (!int.TryParse(parm, out blocksize))
                                throw new CmdLineArgumentException($"Cannot parse \"{parm}\" as a number", "-bs");
                            if (blocksize < 128 || blocksize > 16384)
                                throw new CmdLineArgumentException($"Blocksize must be 128 <= bs <=16384", "-bs");
                            break;
                        case "-ps":
                            if (!int.TryParse(parm, out uiblen))
                                throw new CmdLineArgumentException($"Cannot parse \"{parm}\" as a number", "-ps");
                            if (uiblen < 4 | uiblen > 64)
                                throw new CmdLineArgumentException($"Pipeline must be 4<= ps <=64", "-ps");
                            break;
                        case "-fw":
                            if (!File.Exists(parm))
                                throw new CmdLineArgumentException($"Firmware path \"{parm}\" is not a file", "-fw");
                            firmwarepath = parm;
                            break;
                        case "-ifclk":
                            if (parm == "")
                                throw new CmdLineArgumentException("Bad -ifclk param", "-ifclk");
                            if (parm.ToLower().EndsWith("i"))
                            {
                                cfg_fw.IFClockInvert = true;
                                parm = parm.Substring(0, parm.Length - 1);
                            }
                            else
                                cfg_fw.IFClockInvert = false;

                            switch (parm)
                            {
                                case "x":
                                    cfg_fw.IFClock = FirmwareConfigIFClock.External;
                                    cfg_fw.IFClockOut = false;
                                    break;
                                case "30":
                                    cfg_fw.IFClock = FirmwareConfigIFClock.Internal30;
                                    cfg_fw.IFClockOut = false;
                                    break;
                                case "30o":
                                    cfg_fw.IFClock = FirmwareConfigIFClock.Internal30;
                                    cfg_fw.IFClockOut = true;
                                    break;
                                case "48":
                                    cfg_fw.IFClock = FirmwareConfigIFClock.Internal48;
                                    cfg_fw.IFClockOut = false;
                                    break;
                                case "48o":
                                    cfg_fw.IFClock = FirmwareConfigIFClock.Internal48;
                                    cfg_fw.IFClockOut = true;
                                    break;
                                default:
                                    throw new CmdLineArgumentException($"Unrecoginised option {parm}", "-ifclk");
                            }
                            break;

                        case "-cko":
                            cfg_fw.CPUClockInvert = false;
                            cfg_fw.CPUClockOut = false;

                            if (parm.EndsWith("i"))
                            {
                                cfg_fw.CPUClockInvert = true;
                                parm = parm.Substring(0, parm.Length - 1);
                            }

                            if (parm.EndsWith("o"))
                            {
                                cfg_fw.CPUClockOut = true;
                                parm = parm.Substring(0, parm.Length - 1);
                            }
                            else if (parm.EndsWith("z"))
                            {
                                cfg_fw.CPUClockOut = false;
                                parm = parm.Substring(0, parm.Length - 1);
                            }

                            switch (parm)
                            {
                                case "12":
                                    cfg_fw.CPUClock = FirmwareConfigCPUClock.CPU12;
                                    break;
                                case "24":
                                    cfg_fw.CPUClock = FirmwareConfigCPUClock.CPU24;
                                    break;
                                case "48":
                                    cfg_fw.CPUClock = FirmwareConfigCPUClock.CPU12;
                                    break;
                                default:
                                    throw new CmdLineArgumentException($"Unrecognized option {parm}", "-cko");
                            }

                            break;

                        case "-bufs":
                            if (!int.TryParse(parm, out bufs))
                                throw new CmdLineArgumentException($"Bad bufs {parm}", "-bufs");
                            break;

                        default:
                            throw new CmdLineArgumentException($"Unrecognized option \"{arg}\"", sw);
                    }

                    i++;
                }

                if (!noIO)
                {
                    if (i != args.Length - 1)
                    {
                        Console.Error.WriteLine("Incorrect number of arguments");
                        return E_ARGUMENT;
                    }
                    string filename = args[i];
                    if (filename == "--")
                    {
                        fs = (cfg_fw.Direction == FirmwareConfigDirection.In) ? Console.OpenStandardInput() : Console.OpenStandardOutput();
                    }
                    else
                    {
                        try
                        {
                            fs = new FileStream(filename, (cfg_fw.Direction == FirmwareConfigDirection.In) ? FileMode.Create : FileMode.Open);
                        }
                        catch (Exception ex)
                        {
                            Console.Error.WriteLine($"Error opening file \"{filename}\": {ex.Message}");
                            return E_ARGUMENT;
                        }
                    }

                }
                else
                {
                    if (i != args.Length)
                    {
                        Console.Error.WriteLine("Incorrect number of arguments");
                        return E_ARGUMENT;
                    }
                }



                try
                {
                    DoIt(
                        cfg_fw
                        , devSpec
                        , captureLimit
                        , blocksize
                        , uiblen
                        , firmwarepath
                        , fs
                        , debug
                        , bufs
                        , discard
                        );
                    return E_OK;
                }
                finally
                {
                    if (fs != null)
                    {
                        fs.Close();
                        fs.Dispose();
                        fs = null;
                    }
                }

            }
            catch (CmdLineArgumentException a_ex)
            {
                Console.Error.WriteLine($"Bad command line argument \"{a_ex.ParamName}\": {a_ex.Message}");
                return E_ARGUMENT;
            }
            catch (FX2SharpException fx2_ex)
            {
                Console.Error.WriteLine(fx2_ex.Message);
                return fx2_ex.ReturnCode;
            }
            catch (Exception ex)
            {
                Console.Error.WriteLine("Unexpected error:" + ex.ToString());
                return E_UNEX;
            }
        }

        static unsafe void DoIt(
            FirmwareConfig cfg_fw,
            string devSpec,
            long captureLimit,
            int blocksize,
            int uiblen,
            string firmwarepath,
            Stream fs,
            bool debug,
            int bufs,
            long discard
        )
        {


            Console.CancelKeyPress += delegate (object sender, ConsoleCancelEventArgs e)
            {
                e.Cancel = true;
                ct_source.Cancel();                
            };

            using (var usbDevices = new USBDeviceList(CyConst.DEVICES_CYUSB))
            {
                using (CyFX2Device fx2 = Connect(usbDevices, devSpec))
                {
                    if (debug)
                        Console.Error.WriteLine($"Found: {fx2.ToString()} ");

                    if (debug)
                        Console.Error.WriteLine("RESET");
                    //reset device
                    if (!WriteReset(fx2, true))
                        throw new FX2SharpException("RESET failed", E_RESET_FAIL);
                    Thread.Sleep(100);


                    if (debug)
                        Console.Error.WriteLine("UPLOAD FIRMWARE");
                    //upload firmware
                    if (!fx2.LoadRAM(firmwarepath))
                        throw new FX2SharpException("Failed to program Firmware", E_FIRMWARE);

                    if (debug)
                        Console.Error.WriteLine("RESTART");

                    //reset device
                    if (!WriteReset(fx2, false))
                        throw new FX2SharpException("RESTART failed", E_RESET_FAIL);

                    Thread.Sleep(100);

                    if (debug)
                        Console.Error.WriteLine("RESET");

                    //reset device
                    if (!WriteReset(fx2, true))
                        throw new FX2SharpException("RESET failed", E_RESET_FAIL);

                    Thread.Sleep(100);


                    if (debug)
                        Console.Error.WriteLine("UPLOAD CONFIG");

                    byte[] cfgbytes = cfg_fw.ToBytes();
                    //byte[] cfgbytes = new byte[] { 0x21, 0xcb, 0xa0, 0x11, 0x12 }; //hog
                    //byte[] cfgbytes = new byte[] { 0x12, 0xcb, 0xae, 0x0d, 0x12 }; //elm
                    //byte[] cfgbytes = new byte[] { 0x12, 0xcb, 0xe0, 0x0d, 0x12 }; //hog2

                    if (debug)
                        Console.Error.WriteLine($"CFG: {cfgbytes[0]:X2}:{cfgbytes[1]:X2}:{cfgbytes[2]:X2}:{cfgbytes[3]:X2}:{cfgbytes[4]:X2}");

                    if (!WriteRAM(fx2, FW_CFG_A, cfgbytes))
                        throw new FX2SharpException("Write config failed", E_CFG_FAIL);

                    if (debug)
                    {
                        WriteRAM(fx2, 0x1020, new byte[] { 1, 2, 3, 4, 5, 6 });

                        DumpMem(Console.Error, fx2, 0x1000, 0x10);
                        DumpMem(Console.Error, fx2, 0x1010, 0x10);
                        DumpMem(Console.Error, fx2, 0x1020, 0x10);
                        DumpMem(Console.Error, fx2, 0x1030, 0x10);

                        DumpMem(Console.Error, fx2, 0x1000, 0x40);

                        DumpMem(Console.Error, fx2, 0xE600, 0x40);

                        DumpMem(Console.Error, fx2, 0x0000, 0x080);
                    }


                    if (debug)
                        Console.Error.WriteLine("RESTART DEV");
                    //restart device
                    if (!WriteReset(fx2, false))
                        throw new FX2SharpException("START failed", E_GO_FAIL);

                    if (debug)
                        Console.Error.WriteLine("Wait for restart...");

                    Thread.Sleep(100);

                    fx2.AltIntfc = 1;
                    CyBulkEndPoint e = fx2.EndPointOf(EP_BULK_IN) as CyBulkEndPoint;

                    if (e == null)
                        throw new FX2SharpException("Cannot get endpoint", E_ENDPOINT);



                    //bufferpool holds a collection of "free" buffers, to be used by 
                    //the producer thread
                    BlockingCollection<bufandlen> bufferPool = new BlockingCollection<bufandlen>();
                    for (int i = 0; i < bufs; i++)
                    {
                        bufferPool.Add(new bufandlen { buf = new byte[blocksize], len = 0 });
                    }

                    BlockingCollection<bufandlen> bc = new BlockingCollection<bufandlen>(bufs);

                    //start a pair of Thread, one producer, one consumer

                    if (debug)
                        Console.Error.WriteLine("Streaming...");

                    Thread prodThread = null, consThread = null;
                    ThreadParams prodParams = new ThreadParams(
                        uiblen,
                        blocksize,
                        e,
                        (captureLimit == -1)?-1:(captureLimit + (long)blocksize - 1) / (long)blocksize,
                        bufferPool,
                        bc,
                        (discard + blocksize - 1) / blocksize,
                        fs
                        );
                    ThreadParams consParams = new ThreadParams(
                        uiblen,
                        blocksize,
                        e,
                        (captureLimit == -1) ? -1 : (captureLimit + (long)blocksize - 1) / (long)blocksize,
                        bufferPool,
                        bc,
                        (discard + blocksize - 1) / blocksize,
                        fs
                        );

                    Stopwatch sw = new Stopwatch();
                    sw.Start();

                    if (cfg_fw.Direction == FirmwareConfigDirection.In)
                    {
                        if (fs == null)
                            consThread = new Thread(() => NullConsumer(consParams));
                        else
                            consThread = new Thread(() => StreamConsumer(consParams));

                        prodThread = new Thread(() => UsbProducer(prodParams));

                        consThread.Start();

                        Thread.Sleep(10);

                        prodThread.Start();

                        prodThread.Join();
                        consThread.Join();

                    }
                    else
                    {
/*
                        consThread = new Thread(() => UsbConsumer(consParams));

                        if (fs == null)
                            prodThread = new Thread(() => NullProducer(prodParams));
                        else
                            prodThread = new Thread(() => StreamProducer(prodParams));
                        prodThread.Start();

                        Thread.Sleep(10);

                        consThread.Start();
*/                
                    }

                    sw.Stop();

                    // this is bogus as it doesn't take into account discard (need a way of catering for that without slowing thread)
                    if (debug)
                        Console.Error.WriteLine($"Transfered {prodParams.Transfered}in/{consParams.Transfered}out in {sw.ElapsedMilliseconds}ms => {(double)prodParams.Transfered/(double)(1024*sw.ElapsedMilliseconds)}MB/s ");

                }
            }


            if (debug)
                Console.Error.WriteLine("OK!");
        }

        public class ThreadParams
        {
            public int Uiblen { get; }
            public int Blocksize { get; }
            public CyBulkEndPoint Endpoint { get; }

            public long CaptureLimitBlocks { get; }

            public BlockingCollection<bufandlen> BufferPool { get; }
            public BlockingCollection<bufandlen> FilledBuffers { get; }

            public long Discard { get; }

            public Stream IOStream { get; }

            public long Transfered { get; set; }

            public ThreadParams(
                int uibLen,
                int blockSize,
                CyBulkEndPoint endpoint,
                long captureLimitBlocks,
                BlockingCollection<bufandlen> bufferPool,
                BlockingCollection<bufandlen> filledBuffers,
                long discard,
                Stream ioStream
                )
            {
                this.Uiblen = uibLen;
                this.Blocksize = blockSize;
                this.Endpoint = endpoint;
                this.CaptureLimitBlocks = captureLimitBlocks;
                this.BufferPool = bufferPool;
                this.FilledBuffers = filledBuffers;
                this.Discard = discard;
                this.IOStream = ioStream;
                this.Transfered = 0;
            }
        }

        static void NullConsumer(ThreadParams p)
        {
            foreach (var ret in p.FilledBuffers)
            {
                p.Transfered += ret.len;

                p.BufferPool.Add(ret);
            }
        }

        static void StreamConsumer(ThreadParams p)
        {
            foreach (var ret in p.FilledBuffers.GetConsumingEnumerable())
            {
                p.IOStream.Write(ret.buf, 0, ret.len);

                p.Transfered += ret.len;

                p.BufferPool.Add(ret);
            }
        }

        static unsafe void UsbProducer(ThreadParams p)
        {
            int i;

            byte[][] cmdBufs = new byte[p.Uiblen][];
            byte[][] xferBufs = new byte[p.Uiblen][];
            byte[][] ovLaps = new byte[p.Uiblen][];
            int[] lens = new int[p.Uiblen];

            try
            {
                Thread.CurrentThread.Priority = ThreadPriority.Highest;

                // Setup the queue buffers
                for (i = 0; i < p.Uiblen; i++)

                {
                    cmdBufs[i] = new byte[CyConst.SINGLE_XFER_LEN + ((p.Endpoint.XferMode == XMODE.BUFFERED) ? p.Blocksize : 0)];
                    xferBufs[i] = new byte[p.Blocksize];
                    ovLaps[i] = new byte[CyConst.OverlapSignalAllocSize];
                    fixed (byte* tmp0 = ovLaps[i])
                    {
                        OVERLAPPED* ovLapStatus = (OVERLAPPED*)tmp0;
                        ovLapStatus->hEvent = PInvoke.CreateEvent(0, 0, 0, 0);
                    }

                }

                // Pre-load the queue with requests

                for (i = 0; i < p.Uiblen; i++)
                {
                    lens[i] = p.Blocksize;
                    p.Endpoint.BeginDataXfer(ref cmdBufs[i], ref xferBufs[i], ref lens[i], ref ovLaps[i]);
                }


                i = 0;
                long ctr = 0;
                int lenmin = p.Blocksize;
                bool first = true;
                while (!ct_token.IsCancellationRequested && (p.CaptureLimitBlocks == -1 || ctr < p.CaptureLimitBlocks))
                {
                    bool skip = p.Discard != -1 && ctr < p.Discard;

                
                    fixed (byte* tmp0 = ovLaps[i])
                    {
                        OVERLAPPED* ovLapStatus = (OVERLAPPED*)tmp0;
                        if (!p.Endpoint.WaitForXfer(ovLapStatus->hEvent, (uint)(first?50000:500)))
                        {
                            p.Endpoint.Abort();
                            PInvoke.WaitForSingleObject(ovLapStatus->hEvent, CyConst.INFINITE);
                            skip = true;
                        }
                        first = false;
                    }

                    if (p.Endpoint.FinishDataXfer(ref cmdBufs[i], ref xferBufs[i], ref lens[i], ref ovLaps[i]))
                    {
                        if (lens[i] != 0)
                        {
                            if (!skip)
                            {
                                bufandlen ret = null;
                                if (!skip)
                                {
                                    if (!p.BufferPool.TryTake(out ret))
                                    {
                                        throw new FX2SharpException("Out of buffers", E_BUFFERS_EXHAUSTED);
                                    }
                                }


                                ret.len = lens[i];
                                Array.Copy(xferBufs[i], ret.buf, lens[i]);

                                p.FilledBuffers.Add(ret);
                                p.Transfered += lens[i];
                            }
                            ctr++;
                        }
                        else
                        {
                            throw new FX2SharpException("USB Timeout", E_TIMEOUT);
                        }

                    }
                    else
                        throw new FX2SharpException($"USB FAIL! {p.Endpoint.LastError}", E_ENDPOINT);

                    lens[i] = p.Blocksize;

                    p.Endpoint.BeginDataXfer(ref cmdBufs[i], ref xferBufs[i], ref lens[i], ref ovLaps[i]);
                    
                    i = (i + 1) % p.Uiblen;                                                                          
                }

            }
            finally
            {
                p.FilledBuffers.CompleteAdding();

                for (i = 0; i < p.Uiblen; i++)
                {
                    fixed (byte* tmp0 = ovLaps[i])
                    {
                        OVERLAPPED* ovLapStatus = (OVERLAPPED*)tmp0;
                        if (ovLapStatus->hEvent != null)
                        {
                            PInvoke.CloseHandle(ovLapStatus->hEvent);
                        }
                    }

                }
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

        static CyFX2Device Connect(USBDeviceList usbDevices, string devSpec)
        {
            CyFX2Device fx2 = null;

            int matchctr = -1;
            int vid = -1;
            int pid = -1;
            int n = -1;

            if (devSpec != "")
            {
                try
                {
                    string[] comp = devSpec.Split(new char[] { ':' });
                    if (comp.Length == 1)
                    {
                        if (!int.TryParse(comp[0], out n))
                            throw new Exception();
                    }
                    else if (comp.Length == 2)
                    {
                        if (!int.TryParse(comp[0], System.Globalization.NumberStyles.HexNumber, CultureInfo.InvariantCulture, out vid) 
                            || !int.TryParse(comp[1], System.Globalization.NumberStyles.HexNumber, CultureInfo.InvariantCulture, out pid))
                            throw new Exception();
                    }
                    else if (comp.Length == 3)
                    {
                        if (!int.TryParse(comp[0], System.Globalization.NumberStyles.HexNumber, CultureInfo.InvariantCulture, out vid) 
                            || !int.TryParse(comp[1], System.Globalization.NumberStyles.HexNumber, CultureInfo.InvariantCulture, out pid) 
                            || !int.TryParse(comp[2], System.Globalization.NumberStyles.HexNumber, CultureInfo.InvariantCulture, out n))
                            throw new Exception();
                    }
                    else
                        throw new Exception();
                }
                catch (Exception)
                {
                    throw new CmdLineArgumentException($"Bad device spec \"{devSpec}\"", "-d=");
                }
            }

            foreach (USBDevice u in usbDevices)
            {
                if (u is CyFX2Device)
                {
                    if (fx2 == null)
                    {
                        if (vid == -1 || (u.VendorID == vid && u.ProductID == pid))
                        {
                            matchctr++;
                            if (n == -1 || matchctr == n)
                                fx2 = u as CyFX2Device;
                        }
                    }
                    else
                        throw new Exception("More than one!");
                }
            }

            if (fx2 == null)
                throw new FX2SharpException("Device not found", E_NODEV);

            return fx2;
        }

        static void DumpMem(TextWriter w, CyFX2Device fx2, ushort addr, int len)
        {
            //read memory and dump 
            byte[] b = ReadRAM(fx2, addr, len);
            for (int i = 0; i < b.Length; i++)
            {
                if (i % 16 == 0)
                {
                    w.WriteLine();
                    w.Write($"{(addr + i):X4} : ");
                }
                w.Write($" {b[i]:X2}");
            }
            w.WriteLine();
        }

    }
}
