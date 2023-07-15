using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VrPcSwitcher : MonoBehaviour
{
    public GameObject[] EnableOnPC;
    public GameObject[] EnableOnVR;
    public GameObject[] DisableOnPC;
    public GameObject[] DisableOnVR;
    // Start is called before the first frame update
    void Start()
    {
#if UNITY_ANDROID
        foreach(var go in EnableOnVR) {
            go.SetActive(true);
        }
        foreach (var go in DisableOnVR) {
            go.SetActive(false);
        }
#else
        foreach(var go in EnableOnPC) {
            go.SetActive(true);
        }
        foreach (var go in DisableOnPC) {
            go.SetActive(false);
        }
#endif
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
