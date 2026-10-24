[System.Serializable]
public class SpaceTimeVertexConstraint
{
    public int gridX;
    public int gridZ;

    public int time;


    public SpaceTimeVertexConstraint(
        int gridX,
        int gridZ,
        int time)
    {
        this.gridX = gridX;
        this.gridZ = gridZ;

        this.time = time;
    }
}