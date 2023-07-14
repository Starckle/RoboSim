using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObstacleViewer : MonoBehaviour
{
    public Transform quad_prefab;
    Transform[,] obstacles = new Transform[RC.GRID_WIDTH, RC.GRID_HEIGHT];
    // Start is called before the first frame update
    void Start()
    {
        for(var x=0; x<RC.GRID_WIDTH; x++) {
            for(var y=0; y<RC.GRID_HEIGHT; y++) {
                obstacles[x,y] = Instantiate(quad_prefab, RC.IndexToPosition(new Vector2Int(x, y))+Vector3.up, Quaternion.Euler(90, 0, 0), transform);
            }
        }
    }
    public void setObstacles(float[,] obs_vals) {
        for (var x = 0; x < RC.GRID_WIDTH; x++) {
            for (var y = 0; y < RC.GRID_HEIGHT; y++) {
                obstacles[x, y].localScale = Mathf.Clamp(obs_vals[x,y]/(10*RC.CLEAR_OF_OBSTACLES)-0.1f, 0, 1)*Vector3.one;
            }
        }
    }
    // Update is called once per frame
    void Update()
    {
        
    }
}
