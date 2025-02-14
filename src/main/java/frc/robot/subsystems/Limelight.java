package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private NetworkTable table;
    private String name;
    private boolean isRed;

    private double ta = 0;
    private double tx = 0;
    private double ty = 0;
    private double tv = 0;

    public Pose2d botPose = new Pose2d(0, 0, new Rotation2d());

    public Limelight(String name, boolean isRed) {
        this.name = "limelight-" + name;
        this.isRed = isRed;
        table = NetworkTableInstance.getDefault().getTable(this.name);
    }

    public void updateLimelightTracking() {
        ta = table.getEntry("ta").getDouble(0);
        tx = table.getEntry("tx").getDouble(0);
        ty = table.getEntry("ty").getDouble(0);
        tv = table.getEntry("tv").getDouble(0);

        double[] pos = table
            .getEntry("botpose_wpi" + (isRed ? "red" : "blue")) 
            .getDoubleArray(new double[6]);

        if (pos.length < 6) {
            return;
        }

        double rz = pos[5];

        botPose = 
            new Pose2d(pos[0], pos[1], new Rotation2d(Math.toRadians(rz)));

        String[] names = {
            "pos x",
            "pos y",
            "pos z",
            "rot x",
            "rot y",
            "rot z",
        };

        for (int i = 0; i < names.length; i++) {
            SmartDashboard.putNumber(names[i], pos[i]);
        }
    }

    public double gettx() {
        updateLimelightTracking();
        return tx;
    }

    public double gettv() {
        updateLimelightTracking();
        return tv;
    }

    @Override
    public void periodic(){
        updateLimelightTracking();
    }

}
