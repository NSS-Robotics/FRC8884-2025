package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import java.lang.invoke.ConstantCallSite;

public class Up extends Command {

    private final RobotContainer robotContainer;
    private final Elevator m_elevator;
    private final Wrist m_wrist;
    private RobotState targetState;
    private double targetElevatorPos;
    private double targetWristPos;
    private double outWristPos =
        Constants.WristConstants.pos[RobotState.algaeGround.ordinal()];

    public Up(RobotContainer robotContainer, Elevator elevator, Wrist wrist) {
        this.robotContainer = robotContainer;
        m_elevator = elevator;
        m_wrist = wrist;
        addRequirements(m_elevator, m_wrist);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        targetState = robotContainer.scoringLevel;
        targetElevatorPos =
            Constants.ElevatorConstants.pos[targetState.ordinal()];
        targetWristPos = Constants.WristConstants.pos[targetState.ordinal()];
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (
            m_elevator.getPosition() < Constants.ElevatorConstants.upThreshold
        ) {
            m_wrist.setWrist(outWristPos); // safe position for elev to move up
        } else if (
            m_elevator.getPosition() >
            Constants.ElevatorConstants.wristDownSafeThreshold
        ) {
            m_wrist.setWrist(targetWristPos); // only move wrist down when elevator up (prevents wrist/bumper collision)
        }
        // // only move elevator when wrist out (prevents dismembering)
        if (
            m_wrist.getPosition() >
            Constants.WristConstants.minElevatorRaisedPos
        ) {
            m_elevator.setElevator(
                targetElevatorPos,
                Constants.ElevatorConstants.upSlot
            );
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
