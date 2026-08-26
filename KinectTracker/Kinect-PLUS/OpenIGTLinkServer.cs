using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Numerics;
using System.Text;
using System.Threading;
using MathNet.Numerics.LinearAlgebra;

namespace KinectTracker
{
    //Servidor OpenIGTLink. La app actúa de fuente de tracking (como un PlusServer):
    //Slicer se conecta como cliente y reciben las poses como mensajes TRANSFORM.

    //Start() -> SendTransform(deviceName, R, t) -> Stop()

    public class OpenIGTLinkServer
    {
        private TcpListener listener;
        private readonly List<TcpClient> clients = new List<TcpClient>();
        private readonly object clientsLock = new object();
        private Thread acceptThread;
        private bool running = false;

        //Tabla CRC-64 precalculada (ECMA-182)
        private static readonly ulong[] crcTable = BuildCrcTable();

        public OpenIGTLinkServer(int port = 18944)
        {
            listener = new TcpListener(IPAddress.Any, port);
        }

        public void Start()
        {
            running = true;
            listener.Start();
            acceptThread = new Thread(AcceptLoop) { IsBackground = true };
            acceptThread.Start();
            Console.WriteLine("[IGTL] Servidor escuchando en puerto 18944. Esperando a Slicer/PLUS...");
        }

        public void Stop()
        {
            running = false;
            lock (clientsLock)
            {
                foreach (var c in clients) { try { c.Close(); } catch { } }
                clients.Clear();
            }
            try { listener.Stop(); } catch { }
        }

        //Acepta conexiones entrantes en segundo plano
        private void AcceptLoop()
        {
            while (running)
            {
                try
                {
                    TcpClient client = listener.AcceptTcpClient();
                    lock (clientsLock) { clients.Add(client); }
                    Console.WriteLine("[IGTL] Cliente conectado.");
                }
                catch { if (!running) break; }
            }
        }

        //Envía una pose (R 3x3, t en mm) como mensaje TRANSFORM a todos los clientes
        public void SendTransform(string deviceName, Matrix<double> R, Vector3 t)
        {
            byte[] packet = BuildTransformMessage(deviceName, R, t);

            lock (clientsLock)
            {
                //Recorremos al revés para poder quitar clientes muertos
                for (int i = clients.Count - 1; i >= 0; i--)
                {
                    try
                    {
                        clients[i].GetStream().Write(packet, 0, packet.Length);
                    }
                    catch
                    {
                        try { clients[i].Close(); } catch { }
                        clients.RemoveAt(i);
                        Console.WriteLine("[IGTL] Cliente desconectado.");
                    }
                }
            }
        }

        //CONSTRUCCIÓN DEL MENSAJE

        private static byte[] BuildTransformMessage(string deviceName, Matrix<double> R, Vector3 t)
        {
            //Cuerpo: 12 floats big-endian, column-major (rotación) + traslación
            float[] m = new float[12];
            m[0] = (float)R[0, 0]; m[1] = (float)R[1, 0]; m[2] = (float)R[2, 0];
            m[3] = (float)R[0, 1]; m[4] = (float)R[1, 1]; m[5] = (float)R[2, 1];
            m[6] = (float)R[0, 2]; m[7] = (float)R[1, 2]; m[8] = (float)R[2, 2];
            m[9] = t.X; m[10] = t.Y; m[11] = t.Z;

            byte[] body = new byte[48];
            for (int i = 0; i < 12; i++)
            {
                byte[] f = BitConverter.GetBytes(m[i]);
                if (BitConverter.IsLittleEndian) Array.Reverse(f); //a big-endian
                Array.Copy(f, 0, body, i * 4, 4);
            }

            //CRC-64 del cuerpo
            ulong crc = Crc64(body, body.Length, 0UL);

            //Header (58 bytes)
            byte[] header = new byte[58];
            int offset = 0;

            //Version (uint16) = 1
            WriteBE(header, ref offset, (ushort)1);

            //Tipo "TRANSFORM" (12 bytes, padding 0)
            WriteString(header, ref offset, "TRANSFORM", 12);

            //Nombre del dispositivo (20 bytes, padding 0)
            WriteString(header, ref offset, deviceName, 20);

            //Timestamp (uint64): segundos.fracción en formato fixed-point IGTL
            double secs = (DateTime.UtcNow - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalSeconds;
            uint sec = (uint)Math.Floor(secs);
            uint frac = (uint)((secs - sec) * 4294967296.0); //* 2^32
            ulong ts = ((ulong)sec << 32) | frac;
            WriteBE(header, ref offset, ts);

            //Body size (uint64) = 48
            WriteBE(header, ref offset, (ulong)body.Length);

            //CRC (uint64)
            WriteBE(header, ref offset, crc);

            //oncatenar header + body
            byte[] packet = new byte[header.Length + body.Length];
            Array.Copy(header, 0, packet, 0, header.Length);
            Array.Copy(body, 0, packet, header.Length, body.Length);
            return packet;
        }

        //Helpers de escritura big-endian

        private static void WriteBE(byte[] buf, ref int offset, ushort value)
        {
            buf[offset++] = (byte)(value >> 8);
            buf[offset++] = (byte)(value & 0xFF);
        }

        private static void WriteBE(byte[] buf, ref int offset, ulong value)
        {
            for (int i = 7; i >= 0; i--)
                buf[offset++] = (byte)((value >> (i * 8)) & 0xFF);
        }

        private static void WriteString(byte[] buf, ref int offset, string s, int length)
        {
            byte[] bytes = Encoding.ASCII.GetBytes(s);
            for (int i = 0; i < length; i++)
                buf[offset + i] = (i < bytes.Length) ? bytes[i] : (byte)0;
            offset += length;
        }

        //CRC-64 ECMA-182 (poly 0x42F0E1EBA9EA3693, sin reflexión)

        private static ulong[] BuildCrcTable()
        {
            ulong[] table = new ulong[256];
            for (int n = 0; n < 256; n++)
            {
                ulong c = ((ulong)n) << 56;
                for (int k = 0; k < 8; k++)
                {
                    if ((c & 0x8000000000000000UL) != 0)
                        c = (c << 1) ^ 0x42F0E1EBA9EA3693UL;
                    else
                        c = c << 1;
                }
                table[n] = c;
            }
            return table;
        }

        private static ulong Crc64(byte[] data, int len, ulong crc)
        {
            for (int i = 0; i < len; i++)
                crc = crcTable[((crc >> 56) ^ data[i]) & 0xFF] ^ (crc << 8);
            return crc;
        }
    }
}