public class RelativePose
{
    public double r;
    public double theta;
    public double objectYaw;
    public double heading;
    public double distance;

    public RelativePose(double h, double d, double y)
    {
        this.heading = Math.toDegrees(h);
        this.distance = d;
        this.objectYaw = Math.toDegrees(y);
    }
    public String toString(){
        return new String("Heading: " + heading + "\n Distance: " + distance + "\n ObjectYaw: " + objectYaw);
    }
}
