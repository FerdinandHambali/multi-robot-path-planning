
using UnityEngine;
using System.Collections.Generic;

public class RobotPath : MonoBehaviour
{
    public List<Node> path { get; set; } =
        new List<Node>();

    [HideInInspector]
    public bool routeFinalized = false;

    [Header("Path Appearance")]
    public Color pathColor = Color.blue;
    public float pathThickness = 0.35f;

    [Header("Visualisation Only")]
    public float gizmoHeight = 1.3f;
    public float gizmoSideOffset = 0f;

    private bool debugLogged = false;

    void OnDrawGizmos()
    {
        if (path == null || path.Count == 0)
            return;

        // Confirm that the path is reaching the
        // drawing function.
        if (Application.isPlaying && !debugLogged)
        {
            Debug.Log(
                gameObject.name +
                " Gizmo drawing " +
                path.Count + " nodes."
            );

            debugLogged = true;


            #if UNITY_EDITOR
            // Draw a clearly visible line above all scene geometry.
            Vector3[] points = new Vector3[path.Count];

            Vector3 visualOffset =
                Vector3.up * gizmoHeight +
                Vector3.right * gizmoSideOffset;

            for (int i = 0; i < path.Count; i++)
            {
                points[i] = path[i].worldPosition + visualOffset;
            }

            var previousZTest = UnityEditor.Handles.zTest;

            UnityEditor.Handles.zTest =
                UnityEngine.Rendering.CompareFunction.Always;

            UnityEditor.Handles.color =
                new Color(
                    pathColor.r,
                    pathColor.g,
                    pathColor.b,
                    1f
                );

            UnityEditor.Handles.DrawAAPolyLine(14f, points);

            UnityEditor.Handles.zTest = previousZTest;
#endif

        }

        Gizmos.color = pathColor;

        // Visual offset does not change robot movement.
        Vector3 offset =
            Vector3.up * gizmoHeight +
            Vector3.right * gizmoSideOffset;

        for (int i = 0; i < path.Count - 1; i++)
        {
            Vector3 start =
                path[i].worldPosition + offset;

            Vector3 end =
                path[i + 1].worldPosition + offset;

            Vector3 centre = (start + end) / 2f;

            Vector3 size = new Vector3(
                Mathf.Abs(end.x - start.x)
                    + pathThickness,
                pathThickness,
                Mathf.Abs(end.z - start.z)
                    + pathThickness
            );

            Gizmos.DrawCube(centre, size);
        }

        foreach (Node node in path)
        {
            Vector3 position =
                node.worldPosition + offset;

            Gizmos.DrawSphere(
                position,
                pathThickness * 0.55f
            );
        }
    }
}
