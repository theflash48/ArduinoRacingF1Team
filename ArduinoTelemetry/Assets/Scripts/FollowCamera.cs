using UnityEngine;

public class FollowCamera : MonoBehaviour
{
    public Transform cameraTarget;

    void LateUpdate()
    {
        if (!cameraTarget) return;

        transform.position = cameraTarget.position;

        float yaw = cameraTarget.eulerAngles.y;
        float pitch = transform.eulerAngles.x;   // conserva X de la cámara
        float roll = 0f;

        transform.rotation = Quaternion.Euler(pitch, yaw, roll);
    }
}