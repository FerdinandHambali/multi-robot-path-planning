[System.Serializable]
public class SpaceTimeEdgeConstraint
{
    public int fromX;
    public int fromZ;

    public int toX;
    public int toZ;

    // Represents:
    // time -> time + 1
    public int time;


    public SpaceTimeEdgeConstraint(
        int fromX,
        int fromZ,
        int toX,
        int toZ,
        int time)
    {
        this.fromX = fromX;
        this.fromZ = fromZ;

        this.toX = toX;
        this.toZ = toZ;

        this.time = time;
    }
}