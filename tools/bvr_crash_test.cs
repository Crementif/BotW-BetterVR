using System;
using System.Runtime.InteropServices;
class CrashTest {
    static void Main() {
        Console.WriteLine("bvr_crash_test: triggering access violation for WER LocalDumps verification");
        Marshal.WriteInt32(IntPtr.Zero, 42);
    }
}
