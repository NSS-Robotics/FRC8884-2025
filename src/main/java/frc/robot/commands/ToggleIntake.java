package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

/** An example command that uses an example subsystem. */
public class ToggleIntake extends Command {

    private final RobotContainer ron;
    private final Intake m_intake;

    public ToggleIntake(RobotContainer ron, Intake m_intake) {
        this.ron = ron;
        this.m_intake = m_intake;

        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ron.intakeDown = !ron.intakeDown;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.setPivot(
            ron.intakeDown
                ? Constants.IntakeConstants.intakePosition
                : Constants.IntakeConstants.upPosition,
            ron.intakeDown ? 1 : 0
        );
    }
}
