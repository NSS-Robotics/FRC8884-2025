package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class TurnAround extends Command {

    private final RobotContainer rob;
    private Swerve m_swerve;
    private TurnAroundPIDController pidController;
    private boolean stationIntake;

    private boolean atSetpoint;
    private double[] angles = {
        180,
        120,
        120,
        60,
        60,
        0,
        -180,
        -120,
        -120,
        -60,
        -60,
        -0,
    };

    private double[] l1Angles = {
        120,
        180,
        60,
        120,
        0,
        60,
        -120,
        -180,
        -60,
        -120,
        -0,
        -60,
    };

    public TurnAround(
        RobotContainer rob,
        Swerve swerve,
        boolean stationIntake
    ) {
        this.rob = rob;
        this.m_swerve = swerve;
        this.stationIntake = stationIntake;
        this.pidController = new TurnAroundPIDController(this.m_swerve);

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        atSetpoint = false;
    }

    @Override
    public void execute() {
        double[] angles = rob.scoringLevel.equals(Constants.RobotState.l1)
            ? l1Angles
            : this.angles;

        double angle = stationIntake
            ? (rob.postIndex < 6 ? -55 : 55)
            : angles[rob.postIndex];

        if (!atSetpoint && Math.abs(pidController.getError(angle)) < 1) {
            atSetpoint = true;
            m_swerve.stopSwerve();
        }

        pidController.turn(angle);
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
