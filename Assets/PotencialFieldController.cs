using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class PotentialFieldController : MonoBehaviour
{
    [Header("Objetivo")]
    public Transform target;
    public float goalTolerance = 0.4f; // Umbral epsilon (||q - q_final|| < eps)

    [Header("Parámetros Campo Atractivo (U_att)")]
    public float zeta = 2.0f;          // Ganancia de atracción (ζ)
    public float dThreshold = 3.0f;    // Umbral parabólico-cónico (d)

    [Header("Parámetros Campo Repulsivo (U_rep)")]
    public float eta = 1.5f;           // Ganancia repulsiva (η)
    public float rho0 = 2.5f;          // Distancia de influencia máxima (ρ0)
    public float maxRepulsiveForce = 15.0f; // Saturación para estabilidad física

    [Header("Sensores Raycast (LiDAR 2D)")]
    public int numRays = 24;           // Número de rayos en 360 grados
    public float rayHeightOffset = 0.25f;
    public LayerMask obstacleLayer;

    [Header("Control Cinemático No Holonómico")]
    public float maxLinearSpeed = 3.0f;   // v_max (m/s)
    public float maxAngularSpeed = 2.5f;  // ω_max (rad/s)
    public float angularGain = 3.0f;      // Ganancia proporcional de orientación

    [Header("Escape de Mínimos Locales (RPP)")]
    public float stuckCheckInterval = 1.0f;
    public float minMovementThreshold = 0.15f;
    public float escapeDuration = 1.5f;

    private Rigidbody rb;
    private bool goalReached = false;

    // Variables de detección de mínimos locales
    private Vector3 lastCheckedPos;
    private float stuckTimer = 0f;
    private float escapeTimer = 0f;
    private Vector3 escapeDirection = Vector3.zero;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        lastCheckedPos = new Vector3(transform.position.x, 0f, transform.position.z);
    }

    void FixedUpdate()
    {
        if (target == null || goalReached) return;

        // Proyección al plano horizontal XZ
        Vector3 q = new Vector3(transform.position.x, 0f, transform.position.z);
        Vector3 qFinal = new Vector3(target.position.x, 0f, target.position.z);

        // Criterio de parada: ||q - q_final|| < epsilon
        float distToGoal = Vector3.Distance(q, qFinal);
        if (distToGoal < goalTolerance)
        {
            goalReached = true;
            Debug.Log("¡Objetivo alcanzado mediante Campos Potenciales!");
            return;
        }

        // Comprobar si estamos atrapados en un mínimo local
        CheckLocalMinimum(q);

        Vector3 totalForce;

        if (escapeTimer > 0f)
        {
            // Modo escape (caminata aleatoria)
            escapeTimer -= Time.fixedDeltaTime;
            totalForce = escapeDirection;
            Debug.DrawRay(transform.position + Vector3.up * rayHeightOffset, totalForce * 2f, Color.yellow);
        }
        else
        {
            // Fuerza Atractiva
            Vector3 fAtt = ComputeAttractiveForce(q, qFinal, distToGoal);

            // Fuerza Repulsiva (anillo de Raycasts)
            Vector3 fRep = ComputeRepulsiveForce(q);

            // Fuerza Neta (Descenso de gradiente)
            totalForce = fAtt + fRep;

            // Visualización de vectores en la ventana Scene
            Vector3 origin = transform.position + Vector3.up * rayHeightOffset;
            Debug.DrawRay(origin, fAtt.normalized * 1.5f, Color.green);
            Debug.DrawRay(origin, fRep.normalized * 1.5f, Color.red);
            Debug.DrawRay(origin, totalForce.normalized * 2.0f, Color.blue);
        }

        // Control cinemático no holonómico (v, w)
        ApplyNonHolonomicControl(totalForce, distToGoal);
    }

    private Vector3 ComputeAttractiveForce(Vector3 q, Vector3 qFinal, float rhoF)
    {
        Vector3 diff = q - qFinal; // (q - q_final)

        if (rhoF <= dThreshold)
        {
            // Región cercana (parabólica): F_att = -ζ (q - q_final)
            return -zeta * diff;
        }
        else
        {
            // Región lejana (cónica): F_att = -d * ζ * (q - q_final) / ρ_f
            return -(dThreshold * zeta / rhoF) * diff;
        }
    }

    private Vector3 ComputeRepulsiveForce(Vector3 q)
    {
        Vector3 totalRep = Vector3.zero;
        Vector3 rayOrigin = transform.position + Vector3.up * rayHeightOffset;
        float angleStep = 360f / numRays;

        for (int i = 0; i < numRays; i++)
        {
            float angle = i * angleStep;
            Vector3 dir = Quaternion.Euler(0f, angle, 0f) * transform.forward;

            if (Physics.Raycast(rayOrigin, dir, out RaycastHit hit, rho0, obstacleLayer))
            {
                // Distancia mínima al obstáculo ρ(q), evitando división por cero
                float rho = Mathf.Max(hit.distance, 0.05f);

                // Gradiente de distancia ∇ρ(q) = (q - b) / ||q - b|| en el plano XZ
                Vector3 b = new Vector3(hit.point.x, 0f, hit.point.z);
                Vector3 gradRho = (q - b).normalized;

                // F_rep = η * (1/ρ - 1/ρ0) * (1/ρ^2) * ∇ρ(q)
                float magnitude = eta * ((1f / rho) - (1f / rho0)) * (1f / (rho * rho));
                totalRep += magnitude * gradRho;

                Debug.DrawLine(rayOrigin, hit.point, new Color(1f, 0.4f, 0f, 0.5f));
            }
        }

        // Saturación para evitar impulsos infinitos al rozar colliders
        if (totalRep.magnitude > maxRepulsiveForce)
        {
            totalRep = totalRep.normalized * maxRepulsiveForce;
        }

        return totalRep;
    }

    private void CheckLocalMinimum(Vector3 currentPos)
    {
        if (escapeTimer > 0f) return;

        stuckTimer += Time.fixedDeltaTime;
        if (stuckTimer >= stuckCheckInterval)
        {
            float movedDist = Vector3.Distance(currentPos, lastCheckedPos);
            if (movedDist < minMovementThreshold)
            {
                // Mínimo local detectado: generar dirección de escape aleatoria
                float randomAngle = Random.Range(-180f, 180f);
                escapeDirection = Quaternion.Euler(0f, randomAngle, 0f) * transform.forward;
                escapeTimer = escapeDuration;
                Debug.LogWarning("Mínimo local detectado: ejecutando caminata aleatoria (RPP).");
            }

            lastCheckedPos = currentPos;
            stuckTimer = 0f;
        }
    }

    private void ApplyNonHolonomicControl(Vector3 force, float distToGoal)
    {
        if (force.sqrMagnitude < 1e-5f) return;

        Vector3 desiredDir = force.normalized;

        // Error angular entre la orientación actual del chasis y el gradiente descendente
        float angleErrorDeg = Vector3.SignedAngle(transform.forward, desiredDir, Vector3.up);
        float angleErrorRad = angleErrorDeg * Mathf.Deg2Rad;

        // Velocidad angular ω proporcional al error angular
        float omega = Mathf.Clamp(angularGain * angleErrorRad, -maxAngularSpeed, maxAngularSpeed);

        // Velocidad lineal v modulada por cos(error) para girar antes de avanzar
        float alignmentFactor = Mathf.Max(0f, Mathf.Cos(angleErrorRad));
        float speedScale = Mathf.Clamp01(distToGoal / dThreshold);
        float v = maxLinearSpeed * alignmentFactor * Mathf.Max(0.25f, speedScale);

        // Integración del movimiento en el Rigidbody
        Quaternion deltaRot = Quaternion.Euler(0f, omega * Mathf.Rad2Deg * Time.fixedDeltaTime, 0f);
        rb.MoveRotation(rb.rotation * deltaRot);
        rb.MovePosition(rb.position + transform.forward * (v * Time.fixedDeltaTime));
    }
}