package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class AlignToStation extends Command {

    private RobotContainer rob;
    private Swerve m_swerve;
    private AlignPIDController pidController;
    private Pose2d target;
    private boolean atSetpoint;
    private boolean isLeft;

    public AlignToStation(RobotContainer rob, Swerve swerve, boolean isLeft) {
        this.rob = rob;
        this.m_swerve = swerve;
        this.isLeft = isLeft;
        pidController = new AlignPIDController(swerve);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (
            !atSetpoint &&
            Math.abs(pidController.getXError(target)) < 0.1 &&
            Math.abs(pidController.getYError(target)) < 0.1 &&
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
        target = m_swerve.isRed()
            ? new Pose2d(
                16.333905334503033,
                1.17987787819051,
                Rotation2d.fromDegrees(-55)
            )
            : new Pose2d(1.27, 6.9, Rotation2d.fromDegrees(-125));
        // target = m_swerve.isRed()
        //     ? new Pose2d(16.23, 6.9, Rotation2d.fromDegrees(45))
        //     : new Pose2d(1.27, 6.9, Rotation2d.fromDegrees(135));
    }

    @Override
    public void end(boolean interrupted) {
        if (rob.isCoral) m_swerve.stopSwerve();
    }

    @Override
    public boolean isFinished() {
        return atSetpoint;
    }
}
