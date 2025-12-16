using UnityEngine;
using TMPro;

public class CarHUD : MonoBehaviour
{
    public CarController car;
    public TMP_Text speedText;
    public TMP_Text gearText;
    public TMP_Text rpmText;

    Rigidbody rb;

    void Awake()
    {
        car = gameObject.GetComponent<CarController>();
        rb = gameObject.GetComponent<Rigidbody>();
    }

    void Update()
    {
        if (!car || !rb) return;

        float kmh = rb.linearVelocity.magnitude * 3.6f;

        if (speedText) speedText.text = $"{kmh:0} km/h";
        if (gearText)  gearText.text  = FormatGear(car.CurrentGear);
        if (rpmText)   rpmText.text = $"{car.EngineRpm}/{car.revolutionsMax}";

    }

    static string FormatGear(int g)
    {
        if (g < 0) return "R";
        if (g == 0) return "N";
        return g.ToString();
    }
}