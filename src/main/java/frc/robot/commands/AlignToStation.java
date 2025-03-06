package frc.robot.commands;

import com.reduxrobotics.canand.CanandDeviceDetails.Msg;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class AlignToStation extends Command {

    private Swerve m_swerve;
    private AlignPIDController pidController;
    private Pose2d target;
    private boolean atSetpoint;

    private final Pose2d[] redStations = {
        new Pose2d(
            16.347670560240708,
            1.2212042219638495,
            Rotation2d.fromDegrees(-56.97384521966168)
        ),
        new Pose2d(
            16.255201947463604,
            6.879534286109712,
            Rotation2d.fromDegrees(51.41240189667511)
        ),
    };

    private final Pose2d[] blueStations = {
        new Pose2d(
            1.2692396853783483,
            1.1844133124895877,
            Rotation2d.fromDegrees(-128.7191925970513)
        ),
        new Pose2d(
            1.21958906088757,
            6.8990669868565435,
            Rotation2d.fromDegrees(125.8879236365134)
        ),
    };

    public AlignToStation(Swerve swerve) {
        this.m_swerve = swerve;
        pidController = new AlignPIDController(swerve);

        addRequirements(swerve);
    }

    public double mag(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    @Override
    public void execute() {
        if (
            !atSetpoint &&
            Math.abs(pidController.getXError(target)) < 0.05 &&
            Math.abs(pidController.getYError(target)) < 0.05 &&
            Math.abs(pidController.getAngleError(target)) < 0.5
        ) {
            atSetpoint = true;
        }

        if (
            !atSetpoint &&
            Math.abs(pidController.getXError(target)) < 2 &&
            Math.abs(pidController.getYError(target)) < 2
        ) {
            pidController.alignLimelight(target);
        }

        SmartDashboard.putNumber("Station Target X", target.getX());
        SmartDashboard.putNumber("Station Target Y", target.getY());
        SmartDashboard.putNumber(
            "Station Target R",
            target.getRotation().getDegrees()
        );
    }

    @Override
    public void initialize() {
        atSetpoint = false;

        Pose2d[] poses = m_swerve.isRed() ? redStations : blueStations;

        Pose2d botPose = m_swerve.getPose();

        double dist1 = mag(
            poses[0].getX() - botPose.getX(),
            poses[0].getY() - botPose.getY()
        );
        double dist2 = mag(
            poses[1].getX() - botPose.getX(),
            poses[1].getY() - botPose.getY()
        );
        target = dist1 < dist2 ? poses[0] : poses[1];
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stopSwerve();
    }

    @Override
    public boolean isFinished() {
        return atSetpoint;
    }
}
