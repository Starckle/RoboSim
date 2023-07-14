using UnityEngine;

public class UltraS : MonoBehaviour {
    public float distance = float.MaxValue;
    void FixedUpdate() {
        //Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;
        var lm = LayerMask.GetMask(new string[] { "Default", "Body" });
        if (Physics.SphereCast(transform.position, .4f, transform.forward, out hit, 100, lm)) { //+ transform.forward*.5f
            distance = (hit.point - transform.position).magnitude;
            var color = Color.white;
            if (distance < 1.5f) {
                color = Color.red;
            }
            Debug.DrawLine(transform.position, hit.point, color);
        } else {
            distance = float.MaxValue;
        }
    }
}