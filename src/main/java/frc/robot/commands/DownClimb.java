package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class DownClimb extends Command {

    private final Climber m_climber;
    private final Intake m_intake;

    public DownClimb(Climber m_climber, Intake m_intake) {
        this.m_climber = m_climber;
        this.m_intake = m_intake;
        addRequirements(m_climber, m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_climber.getPosition() < 1) {
            m_climber.setClimber(Constants.ClimberConstants.climbRot, 1);
            m_intake.setPivot(Constants.IntakeConstants.climbPosition, 0);
        }
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
