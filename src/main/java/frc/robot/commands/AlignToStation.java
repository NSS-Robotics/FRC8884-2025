package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class AlignToStation extends Command {

    private Swerve m_swerve;
    private AlignPIDController pidController;
    private Pose2d target;
    private boolean atSetpoint;

    private final Pose2d[] redStations = {
        // left
        new Pose2d(
            17.5 - 1.2692396853783483,
            1.1844133124895877,
            Rotation2d.fromDegrees(-56.97384521966168)
        ),
        // right
        new Pose2d(
            16.37013145134452,
            6.829709251668506,
            Rotation2d.fromDegrees(-123.48950993988609)
        ),
    };

    private final Pose2d[] blueStations = {
        // left
        new Pose2d(
            1.0926524519258318,
            1.288397843456241,
            Rotation2d.fromDegrees(-127.30302739771987)
        ),
        // right
        new Pose2d(
            1.1367700621520278,
            1.2557006747489021,
            Rotation2d.fromDegrees(55)
        ),
    };

    // private final Pose2d blueAutoStationAlign = new Pose2d(
    //     1.5437975423372992,
    //     1.04450067121170695,
    //     Rotation2d.fromDegrees(51.16146211625527)
    // );
    // private final Pose2d redAutoStationAlign = new Pose2d(
    //     15.985993701862498,
    //     7.004769187034083,
    //     Rotation2d.fromDegrees(-128.3819957894946)
    // );

    public AlignToStation(Swerve swerve) {
        this.m_swerve = swerve;
        pidController = new AlignPIDController(swerve, 0.1, 2.4, 0, 0);

        addRequirements(swerve);
    }

    public double mag(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    @Override
    public void execute() {
        if (!m_swerve.gyroZeroed) return;

        if (
            !atSetpoint &&
            Math.abs(pidController.getXError(target)) < 0.03 &&
            Math.abs(pidController.getYError(target)) < 0.03 &&
            Math.abs(pidController.getAngleError(target)) < 10 //TODO:ahdsbf.kabfljhadfbljkh baslkvkhbadfljhvbmdhfbvjhdsbfvkhdbv.kjbdsjfhvbdsjhbjkdfhsvbjdvhbdfjhbdfkjhbdsf
        ) {
            atSetpoint = true;
        }

        // if (
        //     !atSetpoint &&
        //     Math.abs(pidController.getXError(target)) < 2 &&
        //     Math.abs(pidController.getYError(target)) < 2
        // ) {
        pidController.alignLimelight(target);
        // }

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
        m_swerve.isStationAligning = false;
        m_swerve.stopSwerve();
    }

    @Override
    public boolean isFinished() {
        return atSetpoint;
    }
}
