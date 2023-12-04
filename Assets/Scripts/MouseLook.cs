using UnityEngine;

public class MouseLook : MonoBehaviour
{
    public float sensitivity = 2.0f; // Mouse sensitivity

    private bool isDragging = false;
    private Vector3 lastMousePosition;

    void Update()
    {
        HandleInput();
    }

    void HandleInput()
    {
        // Check for mouse button down to start dragging
        if (Input.GetMouseButtonDown(0))
        {
            isDragging = true;
            lastMousePosition = Input.mousePosition;
        }

        // Check for mouse button up to stop dragging
        if (Input.GetMouseButtonUp(0))
        {
            isDragging = false;
        }

        // Rotate the camera while dragging
        if (isDragging)
        {
            Vector3 deltaMouse = Input.mousePosition - lastMousePosition;
            transform.Rotate(Vector3.up * deltaMouse.x * sensitivity, Space.World);
            transform.Rotate(Vector3.left * deltaMouse.y * sensitivity, Space.Self);
        }

        // Update the last mouse position
        lastMousePosition = Input.mousePosition;
    }
}
