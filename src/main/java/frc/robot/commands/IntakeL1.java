package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeL1 extends Command {

    private final Intake m_intake;
    private final RobotContainer robotContainer;
    private final double pose = Constants.IntakeConstants.intakePosition / 6;

    public IntakeL1(RobotContainer robotContainer, Intake m_intake) {
        this.robotContainer = robotContainer;
        this.m_intake = m_intake;

        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.setPivot(pose, m_intake.getPosition() <= pose ? 0 : 1);
        if (Math.abs(m_intake.getPosition() - pose) < 0.02) m_intake.setIntake(
            -Constants.IntakeConstants.stationVelocity * 1.5,
            false
        );
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
