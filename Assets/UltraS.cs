using UnityEngine;

public class UltraS : MonoBehaviour {
    public float distance = float.MaxValue;
    void FixedUpdate() {
        //Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;
        
        if (Physics.SphereCast(transform.position, .4f, transform.forward, out hit, 100)) { //+ transform.forward*.5f
            Debug.DrawLine(transform.position, hit.point);
            distance = (hit.point - transform.position).magnitude;
        } else {
            distance = float.MaxValue;
        }
    }
}