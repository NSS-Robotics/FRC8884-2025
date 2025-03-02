package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class CoralIntake extends Command {

    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Wrist m_wrist;
    private final Claw m_claw;
    private final Elevator m_elevator;
    private final LED l_led;
    private final RobotContainer robotContainer;

    public CoralIntake(
            RobotContainer robotContainer,
            Intake m_intake,
            Indexer m_indexer,
            Wrist m_wrist,
            Claw m_claw,
            Elevator m_elevator,
            LED l_led) {
        this.robotContainer = robotContainer;
        this.m_intake = m_intake;
        this.m_indexer = m_indexer;
        this.m_wrist = m_wrist;
        this.m_claw = m_claw;
        this.m_elevator = m_elevator;
        this.l_led = l_led;

        addRequirements(m_intake, m_indexer, m_wrist, m_claw, l_led);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        robotContainer.runningCommand = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        l_led.intakeLeds();
        if (robotContainer.isCoral) {
            if (m_elevator.getPosition() < Constants.ElevatorConstants.upThreshold) {
                m_wrist.setWrist(
                        Constants.WristConstants.pos[RobotState.handoff.ordinal()]);
            }
            m_intake.setPivot(Constants.IntakeConstants.intakePosition, 1);
            if (m_intake.getPosition() < Constants.IntakeConstants.intakeStartPos) {
                m_intake.setIntake(Constants.IntakeConstants.velocity, false);
                m_indexer.setIndexer(Constants.IndexerConstants.velocity);
                m_claw.setClaw(Constants.EndEffectorConstants.velocity);
            }
        } else {
            if (m_elevator.getPosition() < Constants.ElevatorConstants.upThreshold) {
                m_wrist.setWrist(
                        Constants.WristConstants.pos[RobotState.algaeGround.ordinal()]);
            }
            if (Math.abs(
                    m_wrist.getPosition() -
                            Constants.WristConstants.pos[RobotState.algaeGround
                                    .ordinal()]) < Constants.WristConstants.posTolerance) {
                m_claw.setClaw(Constants.EndEffectorConstants.velocity);
            }
        }
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {
        m_intake.setPivot(Constants.IntakeConstants.upPosition, 0);
        m_intake.stopIntake();
        m_indexer.stopIndexer();
        if (robotContainer.isCoral) {
            m_claw.stopClaw();
        } else {
            m_wrist.setWrist(
                    Constants.WristConstants.pos[RobotState.barge.ordinal()]);
        }
        l_led.stop();
        robotContainer.runningCommand = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
