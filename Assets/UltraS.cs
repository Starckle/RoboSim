using UnityEngine;

public class UltraS : MonoBehaviour {
    public float distance = float.MaxValue;
    void FixedUpdate() {
        //Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;
        
        if (Physics.Raycast(transform.position, transform.forward, out hit, 100)) {
            Debug.DrawLine(transform.position, hit.point);
            distance = (hit.point - transform.position).magnitude;
        } else {
            distance = float.MaxValue;
        }
    }
}