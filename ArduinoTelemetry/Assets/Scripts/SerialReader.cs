using UnityEngine;
using System.IO.Ports;
using System.Threading;

public class SerialReader : MonoBehaviour
{
    public string portName = "COM3"; // cámbialo por el puerto real
    public int baudRate = 115200;

    private SerialPort serialPort;
    private Thread readThread;
    private bool running = false;
    private string lastLine = "";
    
    public int wheelValue = 0;
    public int throttleValue = 0;
    public int gearValue = 0;

    void Start()
    {
        serialPort = new SerialPort(portName, baudRate);
        serialPort.ReadTimeout = 50;

        try
        {
            serialPort.Open();
            running = true;
            readThread = new Thread(ReadSerial);
            readThread.Start();
        }
        catch (System.Exception e)
        {
            Debug.LogError("No se pudo abrir el puerto serie: " + e.Message);
        }
    }

    void ReadSerial()
    {
        while (running && serialPort != null && serialPort.IsOpen)
        {
            try
            {
                string line = serialPort.ReadLine();
                lock (this)
                {
                    lastLine = line;
                }
            }
            catch { }
        }
    }

    void Update()
    {
        string lineCopy = "";
        lock (this)
        {
            lineCopy = lastLine;
        }

        if (!string.IsNullOrEmpty(lineCopy))
        {
            // Para ver qué llega exactamente
            Debug.Log("Rx: '" + lineCopy + "'");

            string[] parts = lineCopy.Trim().Split(',');

            if (parts.Length == 3 &&
                int.TryParse(parts[0], out int w) &&
                int.TryParse(parts[1], out int t) &&
                int.TryParse(parts[2], out int g))
            {
                wheelValue = w;
                throttleValue = t;
                gearValue = g;
            }
        }
    }

    void OnApplicationQuit()
    {
        running = false;
        if (readThread != null && readThread.IsAlive)
            readThread.Join();

        if (serialPort != null && serialPort.IsOpen)
            serialPort.Close();
    }
}
