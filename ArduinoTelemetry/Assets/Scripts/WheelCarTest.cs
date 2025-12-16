using UnityEngine;

public class WheelCarTest : MonoBehaviour
{
    [Header("WheelColliders")]
    public WheelCollider wcFL;
    public WheelCollider wcFR;
    public WheelCollider wcRL;
    public WheelCollider wcRR;

    [Header("Wheel Meshes (visual)")]
    public Transform meshFL;
    public Transform meshFR;
    public Transform meshRL;
    public Transform meshRR;

    [Header("Driving")]
    public bool rearWheelDrive = true;
    public float maxSteerAngle = 22f;
    public float motorTorque = 1800f;   // ajustaremos
    public float brakeTorque = 3000f;   // ajustaremos

    [Header("Stability")]
    public Vector3 centerOfMassOffset = new Vector3(0f, -0.35f, 0f);

    Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass += centerOfMassOffset;
    }

    void FixedUpdate()
    {
        float steer = Input.GetAxis("Horizontal"); // A/D
        float accel = Mathf.Clamp01(Input.GetAxis("Vertical")); // W (solo adelante)
        float brake = Mathf.Clamp01(-Input.GetAxis("Vertical")); // S

        // Dirección (delanteras)
        float steerAngle = steer * maxSteerAngle;
        wcFL.steerAngle = steerAngle;
        wcFR.steerAngle = steerAngle;

        // Motor (traseras por defecto)
        float torque = accel * motorTorque;

        if (rearWheelDrive)
        {
            wcRL.motorTorque = torque;
            wcRR.motorTorque = torque;
            wcFL.motorTorque = 0f;
            wcFR.motorTorque = 0f;
        }
        else
        {
            wcFL.motorTorque = torque;
            wcFR.motorTorque = torque;
            wcRL.motorTorque = 0f;
            wcRR.motorTorque = 0f;
        }

        // Frenos (a las 4)
        float bt = brake * brakeTorque;
        wcFL.brakeTorque = bt;
        wcFR.brakeTorque = bt;
        wcRL.brakeTorque = bt;
        wcRR.brakeTorque = bt;

        // Actualizar meshes
        UpdateWheelPose(wcFL, meshFL);
        UpdateWheelPose(wcFR, meshFR);
        UpdateWheelPose(wcRL, meshRL);
        UpdateWheelPose(wcRR, meshRR);
        
        if (wcRL.GetGroundHit(out WheelHit hitRL))
            Debug.Log($"RL slip: {hitRL.forwardSlip:F2}");

    }

    void UpdateWheelPose(WheelCollider wc, Transform wheelMesh)
    {
        if (!wc || !wheelMesh) return;

        wc.GetWorldPose(out Vector3 pos, out Quaternion rot);
        wheelMesh.position = pos;
        wheelMesh.rotation = rot;
    }
}
