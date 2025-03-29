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
    private boolean second;

    private final Pose2d[] red1Stations = {
        // left 1
        new Pose2d(
            16.0594969587832,
            0.7830495015824503,
            Rotation2d.fromDegrees(125)
        ),
        // right 1
        new Pose2d(
            16.176141176184014,
            7.133442181606093,
            Rotation2d.fromDegrees(-125)
        ),
    };

    private final Pose2d[] red2Stations = {
        // left 2
        new Pose2d(
            16.7506470264957,
            1.384709157495407,
            Rotation2d.fromDegrees(125)
        ),
        // right 2
        new Pose2d(
            16.709179431253965,
            6.842094499292262,
            Rotation2d.fromDegrees(-125)
        ),
    };

    private final Pose2d[] blue1Stations = {
        // left 1
        new Pose2d(
            1.4635853394401748,
            7.509376541192244,
            Rotation2d.fromDegrees(-55)
        ),
        // right 1
        new Pose2d(
            1.4022679738957065,
            0.6610813692485196,
            Rotation2d.fromDegrees(55)
        ),
    };

    private final Pose2d[] blue2Stations = {
        // left 2
        new Pose2d(
            0.8498332334278809,
            6.817571150863968,
            Rotation2d.fromDegrees(-55)
        ),
        // right 2
        new Pose2d(
            0.7221496971264223,
            1.2856696776887762,
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

    public AlignToStation(Swerve swerve, boolean second) {
        this.m_swerve = swerve;
        this.second = second;
        pidController = new AlignPIDController(swerve, 0.3, 4.5, 0, 0);

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
            Math.abs(pidController.getXError(target)) < 0.1 &&
            Math.abs(pidController.getYError(target)) < 0.1 &&
            Math.abs(pidController.getAngleError(target)) < 3 //TODO:ahdsbf.kabfljhadfbljkh baslkvkhbadfljhvbmdhfbvjhdsbfvkhdbv.kjbdsjfhvbdsjhbjkdfhsvbjdvhbdfjhbdfkjhbdsf
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

        Pose2d[] poses = m_swerve.isRed()
            ? second ? red2Stations : red1Stations
            : second ? blue2Stations : blue1Stations;
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
