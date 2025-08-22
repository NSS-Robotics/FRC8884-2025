package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class StationIntake extends Command {

    private Elevator m_elevator;
    private Wrist m_wrist;
    private Claw m_claw;

    public StationIntake(Elevator m_elevator, Wrist m_wrist, Claw m_claw) {
        this.m_elevator = m_elevator;
        this.m_wrist = m_wrist;
        this.m_claw = m_claw;

        addRequirements(m_elevator, m_wrist, m_claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (
            m_elevator.getPosition() <
                Constants.ElevatorConstants.upThreshold &&
            m_wrist.getPosition() <
            Constants.WristConstants.maxElevatorLoweredPos
        ) {
            m_wrist.setWrist(
                Constants.WristConstants.pos[Constants.RobotState.stationIntake.ordinal()]
            );
        }
        if (
            m_wrist.getPosition() >
            Constants.WristConstants.minElevatorRaisedPos
        ) {
            if (
                m_elevator.getPosition() <
                Constants.ElevatorConstants.pos[Constants.RobotState.l4.ordinal()] -
                0.3
            ) {
                m_wrist.setWrist(
                    Constants.WristConstants.pos[Constants.RobotState.stationIntake.ordinal()]
                );
            }
            m_elevator.setElevator(
                Constants.ElevatorConstants.pos[Constants.RobotState.stationIntake.ordinal()],
                m_elevator.getPosition() >
                    Constants.ElevatorConstants.pos[Constants.RobotState.stationIntake.ordinal()]
                    ? 1
                    : 0
            );
            if (!m_claw.gamePieceDetected()) {
                m_claw.setClaw(Constants.EndEffectorConstants.velocity);
            } else {
                m_claw.stopClaw();
            }
        }
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {
        m_claw.stopClaw();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_claw.gamePieceDetected();
    }
}
