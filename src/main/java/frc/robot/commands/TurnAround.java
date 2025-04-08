package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class TurnAround extends Command {

    private final RobotContainer rob;
    private Swerve m_swerve;
    private TurnAroundPIDController pidController;

    private boolean atSetpoint;
    private double[] angles = {
        180,
        120,
        120,
        60,
        60,
        0,
        180,
        -120,
        -120,
        -60,
        -60,
        0,
    };

    public TurnAround(RobotContainer rob, Swerve swerve) {
        this.rob = rob;
        this.m_swerve = swerve;
        this.pidController = new TurnAroundPIDController(this.m_swerve);

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        atSetpoint = false;
    }

    @Override
    public void execute() {
        if (
            !atSetpoint &&
            Math.abs(pidController.getError(angles[rob.postIndex])) < 1
        ) {
            atSetpoint = true;
            m_swerve.stopSwerve();
        }

        pidController.turn(angles[rob.postIndex]);
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
