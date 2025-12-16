using UnityEngine;

public class CarController : MonoBehaviour
{
    // =========================
    // Public read-only (HUD)
    // =========================
    public int CurrentGear => _currentGear;                 // -1..8 (0 = N)
    public int EngineRpm => Mathf.RoundToInt(_engineRpm);    // RPM motor (idle + N free-rev)
    public float SpeedKmh
    {
        get
        {
            if (!_rb) return 0f;
            Vector3 v = _rb.linearVelocity;
            v.y = 0f; // HUD más estable
            return v.magnitude * 3.6f;
        }
    }

    // =========================
    // References
    // =========================
    [Header("Refs")]
    public SerialManager serial;

    [Header("WheelColliders")]
    public WheelCollider wcFL;
    public WheelCollider wcFR;
    public WheelCollider wcRL;
    public WheelCollider wcRR;

    [Header("Wheel Visual Roots (WheelRoot)")]
    public Transform visFL;
    public Transform visFR;
    public Transform visRL;
    public Transform visRR;

    // =========================
    // Steering
    // =========================
    [Header("Steering")]
    public float maxSteerAngle = 22f;
    public bool invertSteer = false;

    // =========================
    // Input mapping (Serial)
    // =========================
    [Header("Input mapping (Serial raw)")]
    public int brakeMaxRaw = 95;   // 0..95 freno (0 = a fondo)
    public int accelMinRaw = 105;  // 105..255 gas
    public int accelMaxRaw = 255;
    public int wheelMinRaw = 0;    // normalmente 0..255
    public int wheelMaxRaw = 255;

    // =========================
    // Throttle response
    // =========================
    [Header("Throttle response")]
    [Tooltip("Activa suavizado del acelerador (hace la aceleración más lenta/progresiva).")]
    public bool throttleSmoothingEnabled = true;

    [Tooltip("Qué rápido SUBE el gas (más bajo = acelera más lento).")]
    public float throttleRiseResponse = 4f;

    [Tooltip("Qué rápido BAJA el gas (más alto = suelta gas más rápido).")]
    public float throttleFallResponse = 12f;

    // =========================
    // Drivetrain / gears
    // =========================
    [Header("Drivetrain (arcade)")]
    public bool rearWheelDrive = true;

    [Tooltip("Más alto = marchas más cortas")]
    public float finalDrive = 9.5f;

    [Range(0.5f, 1f)]
    public float drivetrainEfficiency = 0.92f;

    public float reverseRatio = 3.20f;

    [Tooltip("Ratios 1ª..8ª")]
    public float[] forwardRatios = new float[] { 3.10f, 2.06f, 1.55f, 1.24f, 1.03f, 0.88f, 0.77f, 0.69f };

    public bool lockReverseWhileMoving = true;

    // =========================
    // Engine
    // =========================
    [Header("Engine")]
    public int revolutionsMax = 12000;
    public int idleRpm = 4500;

    [Tooltip("Par máximo de motor (Nm).")]
    public float maxEngineTorque = 520f;

    [Tooltip("Respuesta de RPM en marcha (acoplado)")]
    public float coupledRpmResponse = 14f;

    [Tooltip("Respuesta de RPM en neutro (free-rev)")]
    public float neutralRpmResponse = 22f;

    [Header("Torque curve (simple)")]
    [Range(0f, 1f)] public float torqueBase = 0.35f;
    [Range(0.7f, 0.98f)] public float torqueFallStart = 0.88f;
    [Range(0.8f, 1.0f)] public float torqueFallEnd = 1.00f;

    [Header("Limiter (real)")]
    public bool limiterEnabled = true;
    public float limiterBandRpm = 300f; // soft-cut antes del corte

    [Header("Limiter hold (prevents endless speed in same gear)")]
    public bool limiterHoldEnabled = true;
    public float limiterHoldToleranceKmh = 1.0f;
    public float limiterHoldBrakePerMs = 1200f;
    public float limiterHoldMaxBrakeTorque = 8000f;

    // =========================
    // Launch Control (NEW)
    // =========================
    [Header("Launch Control (NEW)")]
    [Tooltip("Hace que 1ª tenga sentido: limita wheelspin a baja velocidad recortando torque.")]
    public bool launchControlEnabled = true;

    [Tooltip("Hasta qué velocidad actúa el Launch Control.")]
    public float launchMaxSpeedKmh = 120f;

    [Tooltip("Slip a partir del cual empieza a recortar torque.")]
    public float launchSlipStart = 0.12f;

    [Tooltip("Slip donde ya recorta al máximo.")]
    public float launchSlipFull = 0.35f;

    [Tooltip("Escala mínima de torque cuando el slip es muy alto.")]
    public float launchMinTorqueScale = 0.25f;

    [Tooltip("Qué rápido responde el recorte (más alto = más agresivo).")]
    public float launchResponse = 25f;

    // =========================
    // Brakes + ABS
    // =========================
    [Header("Brakes")]
    public float maxBrakeTorque = 3500f;

    [Header("ABS (optional)")]
    public bool absEnabled = true;
    public float absSlipStart = 0.20f;
    public float absSlipFull = 0.55f;
    public float absMinBrakeScale = 0.20f;
    public float absReleaseRate = 25f;
    public float absApplyRate = 10f;

    // =========================
    // Traction Control (full)
    // =========================
    [Header("Traction Control (optional)")]
    public bool tractionControlEnabled = false;
    public float tcSlipStart = 0.20f;
    public float tcSlipFull = 0.60f;
    public float tcMinTorqueScale = 0.25f;

    // =========================
    // Stability
    // =========================
    [Header("Stability")]
    public Vector3 centerOfMassOffset = new Vector3(0f, -0.35f, 0f);

    // =========================
    // Internal
    // =========================
    Rigidbody _rb;
    int _currentGear = 0;   // -1..8 (0=N)
    float _engineRpm = 0f;

    float _absScaleFL = 1f, _absScaleFR = 1f, _absScaleRL = 1f, _absScaleRR = 1f;

    // Throttle smoothed
    float _accelSmoothed01 = 0f;

    // Launch torque scale smoothed
    float _launchTorqueScale = 1f;

    void Awake()
    {
        _rb = GetComponent<Rigidbody>();
        if (!serial) serial = GetComponent<SerialManager>();

        if (_rb) _rb.centerOfMass += centerOfMassOffset;

        _engineRpm = Mathf.Clamp(idleRpm, 0, revolutionsMax);
    }

    void FixedUpdate()
    {
        if (!_rb) return;

        // 1) Leer input raw
        int rawWheel = serial ? serial.wheelValue : 128;
        int rawThrottle = serial ? serial.throttleValue : 128;
        int rawGearValue = serial ? serial.gearValue : 1; // 0..9
        int desiredGear = rawGearValue - 1;               // -1..8

        // 2) Mapear inputs
        float steerInput = MapSteer(rawWheel);
        if (invertSteer) steerInput = -steerInput;

        float accelInputRaw = MapAccel(rawThrottle);   // 0..1
        float brakeInput = MapBrake(rawThrottle);      // 0..1

        // 2.1) Suavizado de gas
        float accelInput = accelInputRaw;
        if (throttleSmoothingEnabled)
        {
            _accelSmoothed01 = MoveExpAsym(_accelSmoothed01, accelInputRaw, throttleRiseResponse, throttleFallResponse, Time.fixedDeltaTime);
            accelInput = _accelSmoothed01;
        }
        else
        {
            _accelSmoothed01 = accelInputRaw;
        }

        // 3) Dirección
        float steerAngle = steerInput * maxSteerAngle;
        if (wcFL) wcFL.steerAngle = steerAngle;
        if (wcFR) wcFR.steerAngle = steerAngle;

        // 4) speed y wheelRPM (desde velocidad, arcade estable)
        Vector3 v = _rb.linearVelocity;
        float speedMs = new Vector3(v.x, 0f, v.z).magnitude;
        float speedKmh = speedMs * 3.6f;

        float wheelRadius = GetReferenceWheelRadius();
        float wheelRpm = (speedMs / (2f * Mathf.PI * wheelRadius)) * 60f;

        // 5) Marcha (shift denied al bajar si haría overrev)
        _currentGear = ResolveGearWithOverrevLock(_currentGear, desiredGear, wheelRpm, speedMs);

        // 6) RPM prospectivas por ruedas y RPM motor
        float ratio = GetRatioForGear(_currentGear);

        float prospectiveRpm = 0f;
        if (_currentGear != 0)
            prospectiveRpm = Mathf.Abs(wheelRpm * ratio * finalDrive);

        if (_currentGear == 0)
        {
            // Neutro: free-rev con idle (usa accelInput suavizado)
            float freeTarget = Mathf.Lerp(idleRpm, revolutionsMax, accelInput);
            _engineRpm = MoveExp(_engineRpm, freeTarget, neutralRpmResponse, Time.fixedDeltaTime);
        }
        else
        {
            // En marcha: acoplado (clamp a idle..max)
            float coupled = Mathf.Clamp(prospectiveRpm, idleRpm, revolutionsMax);
            _engineRpm = MoveExp(_engineRpm, coupled, coupledRpmResponse, Time.fixedDeltaTime);
        }

        // 7) Par motor base
        float engineTorque = maxEngineTorque * SimpleTorqueCurve(_engineRpm, revolutionsMax) * accelInput;

        // 8) Limitador: corta par
        if (limiterEnabled && accelInput > 0f)
        {
            float limiterRpm = (_currentGear == 0) ? _engineRpm : prospectiveRpm;
            engineTorque *= LimiterScale(limiterRpm);
        }

        // 9) A ruedas (en N no empuja)
        float totalWheelTorque = 0f;
        if (_currentGear != 0)
        {
            float sign = (_currentGear < 0) ? -1f : 1f;
            totalWheelTorque = engineTorque * Mathf.Abs(ratio) * finalDrive * drivetrainEfficiency * sign;
        }

        // 10) Launch Control (NEW): recorta torque a baja velocidad si hay slip
        //     Esto hace que 1ª sea mejor que 8ª al salir (porque en 1ª sin esto desperdicias en wheelspin)
        if (launchControlEnabled && _currentGear != 0 && accelInput > 0f && speedKmh <= launchMaxSpeedKmh)
        {
            float slip = GetDrivenForwardSlip();
            float targetScale = LaunchControlScale(slip);

            _launchTorqueScale = MoveExp(_launchTorqueScale, targetScale, launchResponse, Time.fixedDeltaTime);
            totalWheelTorque *= _launchTorqueScale;
        }
        else
        {
            _launchTorqueScale = MoveExp(_launchTorqueScale, 1f, launchResponse, Time.fixedDeltaTime);
        }

        // 11) TC completo (si lo activas aparte)
        if (tractionControlEnabled && _currentGear != 0)
            totalWheelTorque *= TractionControlScale(GetDrivenForwardSlip());

        ApplyMotorTorque(totalWheelTorque);

        // 12) Limiter hold: evita que en la misma marcha siga subiendo la velocidad
        float extraBrakeFL = 0f, extraBrakeFR = 0f, extraBrakeRL = 0f, extraBrakeRR = 0f;
        if (limiterHoldEnabled && _currentGear != 0 && accelInput > 0f)
        {
            float absGearDrive = Mathf.Abs(ratio * finalDrive);
            if (absGearDrive > 0.0001f)
            {
                float targetWheelRpm = revolutionsMax / absGearDrive;
                float targetSpeedMs = (targetWheelRpm / 60f) * (2f * Mathf.PI * wheelRadius);

                float tolMs = Mathf.Max(0f, limiterHoldToleranceKmh) / 3.6f;
                float overspeed = speedMs - (targetSpeedMs + tolMs);

                if (overspeed > 0f)
                {
                    float extra = Mathf.Clamp(overspeed * limiterHoldBrakePerMs, 0f, limiterHoldMaxBrakeTorque);

                    if (rearWheelDrive)
                    {
                        extraBrakeRL = extra * 0.5f;
                        extraBrakeRR = extra * 0.5f;
                    }
                    else
                    {
                        extraBrakeFL = extra * 0.5f;
                        extraBrakeFR = extra * 0.5f;
                    }
                }
            }
        }

        // 13) Frenos + ABS (con extra por limiter hold)
        ApplyBrakesWithAbs(brakeInput, extraBrakeFL, extraBrakeFR, extraBrakeRL, extraBrakeRR);

        // 14) Visual wheels
        UpdateWheelPose(wcFL, visFL);
        UpdateWheelPose(wcFR, visFR);
        UpdateWheelPose(wcRL, visRL);
        UpdateWheelPose(wcRR, visRR);

        // 15) Telemetría a Serial
        if (serial)
        {
            serial.revolutionsMax = revolutionsMax;
            serial.revolutions = Mathf.RoundToInt(_engineRpm);
        }
    }

    // =========================
    // Input mapping
    // =========================
    float MapSteer(int raw)
    {
        float t = Mathf.InverseLerp(wheelMinRaw, wheelMaxRaw, raw);
        return Mathf.Clamp(t * 2f - 1f, -1f, 1f);
    }

    float MapAccel(int raw)
    {
        if (raw < accelMinRaw) return 0f;
        return Mathf.Clamp01((raw - accelMinRaw) / (float)(accelMaxRaw - accelMinRaw));
    }

    float MapBrake(int raw)
    {
        if (raw > brakeMaxRaw) return 0f;
        return Mathf.Clamp01(1f - (raw / (float)brakeMaxRaw));
    }

    // =========================
    // Gear logic
    // =========================
    int ResolveGearWithOverrevLock(int current, int desired, float wheelRpm, float speedMs)
    {
        desired = Mathf.Clamp(desired, -1, 8);
        current = Mathf.Clamp(current, -1, 8);

        if (desired == 0) return 0;

        if (lockReverseWhileMoving && desired < 0 && speedMs > 2.0f)
            return current;

        if (desired >= current)
            return desired;

        float prospectiveRpm = Mathf.Abs(wheelRpm * GetRatioForGear(desired) * finalDrive);
        if (prospectiveRpm > revolutionsMax)
            return current;

        return desired;
    }

    float GetRatioForGear(int gear)
    {
        if (gear == 0) return 0f;
        if (gear < 0) return reverseRatio;

        int idx = Mathf.Clamp(gear - 1, 0, forwardRatios.Length - 1);
        return forwardRatios[idx];
    }

    // =========================
    // Engine helpers
    // =========================
    float SimpleTorqueCurve(float rpm, float rpmMax)
    {
        float x = Mathf.Clamp01(rpm / Mathf.Max(1f, rpmMax));
        float rise = Mathf.SmoothStep(0f, 1f, x);
        float fall = 1f - Mathf.SmoothStep(torqueFallStart, torqueFallEnd, x);
        return Mathf.Clamp01((torqueBase + (1f - torqueBase) * rise) * fall);
    }

    float LimiterScale(float rpmValue)
    {
        if (rpmValue >= revolutionsMax) return 0f;

        float band = Mathf.Max(1f, limiterBandRpm);
        float start = revolutionsMax - band;

        if (rpmValue <= start) return 1f;

        return Mathf.Clamp01((revolutionsMax - rpmValue) / band);
    }

    float LaunchControlScale(float slip)
    {
        float s = Mathf.Abs(slip);

        if (s <= launchSlipStart) return 1f;
        if (s >= launchSlipFull) return launchMinTorqueScale;

        float t = Mathf.InverseLerp(launchSlipStart, launchSlipFull, s);
        return Mathf.Lerp(1f, launchMinTorqueScale, t);
    }

    static float MoveExp(float current, float target, float response, float dt)
    {
        return Mathf.Lerp(current, target, 1f - Mathf.Exp(-Mathf.Max(0f, response) * dt));
    }

    static float MoveExpAsym(float current, float target, float riseResponse, float fallResponse, float dt)
    {
        float resp = (target > current) ? riseResponse : fallResponse;
        return Mathf.Lerp(current, target, 1f - Mathf.Exp(-Mathf.Max(0f, resp) * dt));
    }

    // =========================
    // Slip
    // =========================
    float GetDrivenForwardSlip()
    {
        float slipSum = 0f;
        int count = 0;

        if (rearWheelDrive)
        {
            if (wcRL && wcRL.GetGroundHit(out WheelHit hitRL)) { slipSum += Mathf.Abs(hitRL.forwardSlip); count++; }
            if (wcRR && wcRR.GetGroundHit(out WheelHit hitRR)) { slipSum += Mathf.Abs(hitRR.forwardSlip); count++; }
        }
        else
        {
            if (wcFL && wcFL.GetGroundHit(out WheelHit hitFL)) { slipSum += Mathf.Abs(hitFL.forwardSlip); count++; }
            if (wcFR && wcFR.GetGroundHit(out WheelHit hitFR)) { slipSum += Mathf.Abs(hitFR.forwardSlip); count++; }
        }

        return (count > 0) ? (slipSum / count) : 0f;
    }

    float TractionControlScale(float slip)
    {
        if (slip <= tcSlipStart) return 1f;
        if (slip >= tcSlipFull) return tcMinTorqueScale;

        float t = Mathf.InverseLerp(tcSlipStart, tcSlipFull, slip);
        return Mathf.Lerp(1f, tcMinTorqueScale, t);
    }

    // =========================
    // Torque + brakes
    // =========================
    void ApplyMotorTorque(float totalWheelTorque)
    {
        if (rearWheelDrive)
        {
            if (wcRL) wcRL.motorTorque = totalWheelTorque * 0.5f;
            if (wcRR) wcRR.motorTorque = totalWheelTorque * 0.5f;
            if (wcFL) wcFL.motorTorque = 0f;
            if (wcFR) wcFR.motorTorque = 0f;
        }
        else
        {
            if (wcFL) wcFL.motorTorque = totalWheelTorque * 0.5f;
            if (wcFR) wcFR.motorTorque = totalWheelTorque * 0.5f;
            if (wcRL) wcRL.motorTorque = 0f;
            if (wcRR) wcRR.motorTorque = 0f;
        }
    }

    void ApplyBrakesWithAbs(float brakeInput, float extraFL, float extraFR, float extraRL, float extraRR)
    {
        float baseBt = brakeInput * maxBrakeTorque;

        if (absEnabled && brakeInput > 0f)
        {
            _absScaleFL = UpdateAbsScale(wcFL, _absScaleFL);
            _absScaleFR = UpdateAbsScale(wcFR, _absScaleFR);
            _absScaleRL = UpdateAbsScale(wcRL, _absScaleRL);
            _absScaleRR = UpdateAbsScale(wcRR, _absScaleRR);
        }
        else
        {
            _absScaleFL = Mathf.MoveTowards(_absScaleFL, 1f, absApplyRate * Time.fixedDeltaTime);
            _absScaleFR = Mathf.MoveTowards(_absScaleFR, 1f, absApplyRate * Time.fixedDeltaTime);
            _absScaleRL = Mathf.MoveTowards(_absScaleRL, 1f, absApplyRate * Time.fixedDeltaTime);
            _absScaleRR = Mathf.MoveTowards(_absScaleRR, 1f, absApplyRate * Time.fixedDeltaTime);
        }

        if (wcFL) wcFL.brakeTorque = (baseBt + extraFL) * _absScaleFL;
        if (wcFR) wcFR.brakeTorque = (baseBt + extraFR) * _absScaleFR;
        if (wcRL) wcRL.brakeTorque = (baseBt + extraRL) * _absScaleRL;
        if (wcRR) wcRR.brakeTorque = (baseBt + extraRR) * _absScaleRR;
    }

    float UpdateAbsScale(WheelCollider wc, float currentScale)
    {
        float targetScale = 1f;

        if (wc != null && wc.GetGroundHit(out WheelHit hit))
        {
            float slip = Mathf.Abs(hit.forwardSlip);
            if (slip > absSlipStart)
            {
                float t = Mathf.InverseLerp(absSlipStart, absSlipFull, slip);
                targetScale = Mathf.Lerp(1f, absMinBrakeScale, t);
            }
        }

        float rate = (targetScale < currentScale) ? absReleaseRate : absApplyRate;
        return Mathf.MoveTowards(currentScale, targetScale, rate * Time.fixedDeltaTime);
    }

    // =========================
    // Wheel visuals
    // =========================
    void UpdateWheelPose(WheelCollider wc, Transform visRoot)
    {
        if (!wc || !visRoot) return;
        wc.GetWorldPose(out Vector3 pos, out Quaternion rot);
        visRoot.position = pos;
        visRoot.rotation = rot;
    }

    // =========================
    // Utilities
    // =========================
    float GetReferenceWheelRadius()
    {
        if (rearWheelDrive && wcRL) return Mathf.Max(0.01f, wcRL.radius);
        if (!rearWheelDrive && wcFL) return Mathf.Max(0.01f, wcFL.radius);

        if (wcRR) return Mathf.Max(0.01f, wcRR.radius);
        if (wcFR) return Mathf.Max(0.01f, wcFR.radius);
        return 0.35f;
    }
}
