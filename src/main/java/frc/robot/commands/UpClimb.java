package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class UpClimb extends Command {

    private final Climber m_climber;
    private final Wrist m_wrist;
    private final Intake m_intake;
    private final Elevator m_elevator;

    public UpClimb(
        Climber m_climber,
        Wrist m_wrist,
        Intake m_intake,
        Elevator m_elevator
    ) {
        this.m_climber = m_climber;
        this.m_wrist = m_wrist;
        this.m_intake = m_intake;
        this.m_elevator = m_elevator;

        addRequirements(m_climber, m_wrist, m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_climber.setClimber(0, 0);
        if (
            m_elevator.getPosition() < Constants.ElevatorConstants.upThreshold
        ) {
            m_wrist.setWrist(
                Constants.WristConstants.pos[Constants.RobotState.algaeGround.ordinal()]
            );
            // m_intake.setPivot(Constants.IntakeConstants.intakePosition, 1);
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
