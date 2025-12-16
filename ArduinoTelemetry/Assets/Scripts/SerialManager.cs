using UnityEngine;
using System;
using System.IO.Ports;
using System.Threading;
using System.Collections.Generic;
using System.Text;

public class SerialManager : MonoBehaviour
{
    [Header("Serial")]
    public string portName = "COM3";
    public int baudRate = 115200;

    [Header("RX ESP32 -> Unity (volante,acelerador,marcha)")]
    public int wheelValue = 0;
    public int throttleValue = 0;
    public int gearValue = 0;

    [Header("TX Unity -> ESP32 (revolutions,revolutionsMax)")]
    public int revolutions = 0;
    public int revolutionsMax = 12000;
    public float sendInterval = 0.05f; // 20 Hz

    [Header("Debug")]
    public bool logRx = false;
    public bool logTx = false;

    private SerialPort serialPort;
    private Thread ioThread;
    private volatile bool running = false;

    private readonly object rxLock = new object();
    private readonly Queue<string> rxQueue = new Queue<string>(128);

    void Start()
    {
        try
        {
            serialPort = new SerialPort(portName, baudRate);
            serialPort.ReadTimeout = 1;   // no bloquea casi nada
            serialPort.WriteTimeout = 5;
            serialPort.NewLine = "\n";
            serialPort.Open();

            running = true;
            ioThread = new Thread(IOLoop);
            ioThread.IsBackground = true;
            ioThread.Start();
        }
        catch (Exception e)
        {
            //Debug.LogError("No se pudo abrir el puerto serie: " + e.Message);
        }
    }

    void Update()
    {
        PumpRx();
    }

    void OnApplicationQuit() => CloseEverything();
    void OnDestroy() => CloseEverything();

    void CloseEverything()
    {
        running = false;

        try
        {
            if (ioThread != null && ioThread.IsAlive)
                ioThread.Join(200);
        }
        catch { }

        try
        {
            if (serialPort != null && serialPort.IsOpen)
                serialPort.Close();
        }
        catch { }
    }

    // ======================
    // Hilo único de I/O: LEE + ESCRIBE
    // ======================
    void IOLoop()
    {
        var sb = new StringBuilder(256);
        int nextSendMs = Environment.TickCount; // enviar nada más arrancar
        int intervalMs = Mathf.Max(1, Mathf.RoundToInt(sendInterval * 1000f));

        while (running && serialPort != null && serialPort.IsOpen)
        {
            // ---- LECTURA no bloqueante (por trozos) ----
            try
            {
                int n = serialPort.BytesToRead;
                if (n > 0)
                {
                    string chunk = serialPort.ReadExisting();
                    sb.Append(chunk);

                    // Procesa líneas completas
                    while (true)
                    {
                        int nl = sb.ToString().IndexOf('\n');
                        if (nl < 0) break;

                        string line = sb.ToString(0, nl);
                        sb.Remove(0, nl + 1);

                        line = line.Trim();
                        if (line.Length == 0) continue;

                        lock (rxLock)
                        {
                            if (rxQueue.Count > 200) rxQueue.Clear();
                            rxQueue.Enqueue(line);
                        }
                    }
                }
            }
            catch { }

            // ---- ESCRITURA periódica (rev,max) ----
            int now = Environment.TickCount;
            if (unchecked(now - nextSendMs) >= 0)
            {
                nextSendMs = now + intervalMs;

                // Lee las variables (int es atómico)
                int rev = revolutions;
                int mx = revolutionsMax;

                if (mx < 1) mx = 1;
                if (rev < 0) rev = 0;
                if (rev > mx) rev = mx;

                string msg = rev + "," + mx + "\n"; // IMPORTANTÍSIMO el \n
                try
                {
                    serialPort.Write(msg);
                    // OJO: logTx puede ralentizar muchísimo el editor si está a 20Hz
                    if (logTx) Debug.Log("Tx: '" + msg.Trim() + "'");
                }
                catch { }
            }

            Thread.Sleep(1); // baja CPU sin perder “tiempo real”
        }
    }

    // ======================
    // Consume RX en main thread
    // ======================
    void PumpRx()
    {
        while (true)
        {
            string line;

            lock (rxLock)
            {
                if (rxQueue.Count == 0) break;
                line = rxQueue.Dequeue();
            }

            if (logRx) Debug.Log("Rx: '" + line + "'");

            // ESP32 -> Unity: w,t,g
            string[] parts = line.Split(',');
            if (parts.Length != 3) continue;

            if (int.TryParse(parts[0], out int w) &&
                int.TryParse(parts[1], out int t) &&
                int.TryParse(parts[2], out int g))
            {
                wheelValue = w;
                throttleValue = t;
                gearValue = g;
            }
        }
    }
}
