package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import java.util.function.BooleanSupplier;

public class Up extends Command {

    private final RobotContainer robotContainer;
    private final Elevator m_elevator;
    private final Wrist m_wrist;
    private final Claw m_claw;
    private final Swerve m_swerve;
    private final CommandXboxController m_driverController;
    private Timer timer1;
    private Timer timer2;
    private RobotState targetState;
    private double targetElevatorPos;
    private double targetWristPos;
    private double outWristPos =
        Constants.WristConstants.pos[RobotState.l1.ordinal()];
    private double timerDelay = 1;
    private double SWERVE_STOP = 0.05;
    private boolean ended = false;
    private BooleanSupplier isAtSetpoint;

    public Up(
        RobotContainer robotContainer,
        Elevator elevator,
        Wrist wrist,
        Claw claw,
        Swerve swerve,
        CommandXboxController driverController,
        BooleanSupplier isAtSetpoint
    ) {
        this.isAtSetpoint = isAtSetpoint;
        this.robotContainer = robotContainer;
        m_elevator = elevator;
        m_wrist = wrist;
        m_claw = claw;
        m_swerve = swerve;
        m_driverController = driverController;
        timer1 = new Timer();
        timer2 = new Timer();
        addRequirements(m_elevator, m_wrist, m_claw);
    }

    private boolean canRunClaw() {
        ChassisSpeeds speeds = m_swerve.getRobotRelativeChassisSpeeds();

        return (
            isAtSetpoint.getAsBoolean() &&
            speeds.vxMetersPerSecond < SWERVE_STOP &&
            speeds.vyMetersPerSecond < SWERVE_STOP
        );
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer1 = new Timer();
        timer2 = new Timer();
        timerDelay = 1;
        ended = false;
        robotContainer.runningCommand = true;
        targetState = robotContainer.scoringLevel;
        targetElevatorPos =
            Constants.ElevatorConstants.pos[targetState.ordinal()];
        targetWristPos = Constants.WristConstants.pos[targetState.ordinal()];
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("timer 1", timer1.get());
        SmartDashboard.putNumber("timer 2", timer2.get());
        // CORAL
        if (robotContainer.isCoral) {
            // Wrist logic
            if (
                m_elevator.getPosition() <
                Constants.ElevatorConstants.upThreshold
            ) {
                m_wrist.setWrist(outWristPos); // safe position for elev to move up
            } else if (
                m_elevator.getPosition() >
                Constants.ElevatorConstants.wristDownSafeThreshold
            ) {
                m_wrist.setWrist(targetWristPos); // only move wrist down when elevator up (prevents wrist/bumper collision)
            }

            // only move elevator when wrist out (prevents dismembering)
            if (
                m_wrist.getPosition() >
                Constants.WristConstants.minElevatorRaisedPos
            ) {
                m_elevator.setElevator(
                    targetElevatorPos,
                    Constants.ElevatorConstants.upSlot
                );
            }
            if (
                Math.abs(m_elevator.getPosition() - targetElevatorPos) <
                Constants.ElevatorConstants.posTolerance
            ) {
                if (canRunClaw()) {
                    if (!timer1.isRunning()) {
                        timer1.restart();
                        timerDelay = .25;
                    }
                    m_claw.setClaw(
                        robotContainer.scoringLevel.equals(RobotState.l1)
                            ? -Constants.EndEffectorConstants.l1Velocity
                            : -Constants.EndEffectorConstants.outtakeVelocity
                    );
                }
                if (timer1.hasElapsed(timerDelay)) {
                    m_claw.stopClaw();
                }
            }
        }
        // ALGAE
        else {
            if (targetState.equals(RobotState.processor)) {
                m_wrist.setWrist(targetWristPos);
                if (
                    m_wrist.getPosition() >
                    Constants.WristConstants.minElevatorRaisedPos
                ) {
                    m_elevator.setElevator(
                        targetElevatorPos,
                        Constants.ElevatorConstants.algaeSlot
                    );
                }
                if (
                    Math.abs(m_elevator.getPosition() - targetElevatorPos) <
                        Constants.ElevatorConstants.posTolerance &&
                    canRunClaw()
                ) {
                    m_claw.setClaw(
                        -Constants.EndEffectorConstants.outtakeVelocity
                    );
                    //robotContainer.runningCommand = false;
                    if (!timer1.isRunning()) {
                        timer1.restart();
                    }
                }
                if (
                    !m_claw.gamePieceDetected() && timer1.hasElapsed(timerDelay)
                ) {
                    m_claw.stopClaw();
                }
            } else if (targetState.equals(RobotState.barge)) {
                if (
                    m_wrist.getPosition() >
                    Constants.WristConstants.minElevatorRaisedPos
                ) {
                    m_elevator.setElevator(
                        targetElevatorPos,
                        Constants.ElevatorConstants.algaeSlot
                    );
                }
                if (
                    Math.abs(m_elevator.getPosition() - targetElevatorPos) <
                    Constants.ElevatorConstants.posTolerance
                ) {
                    if (!timer2.isRunning()) {
                        timer2.restart();
                        timerDelay = 0.75;
                    }
                    m_wrist.setWrist(targetWristPos);
                    if (
                        // m_claw.gamePieceDetected() &&
                        Math.abs(m_wrist.getPosition() - targetWristPos) <
                            Constants.WristConstants.posTolerance &&
                        timer2.hasElapsed(timerDelay)
                    ) {
                        if (!timer1.isRunning()) {
                            timer1.restart();
                            timerDelay = 0.5;
                        }
                        //robotContainer.runningCommand = false;
                        m_claw.setClaw(
                            -Constants.EndEffectorConstants.outtakeVelocity
                        );
                    }
                    if (timer1.hasElapsed(timerDelay)) {
                        m_claw.stopClaw();
                    }
                }
            }
            // reef algae intake
            else {
                if (
                    m_elevator.getPosition() <
                    Constants.ElevatorConstants.upThreshold
                ) {
                    m_wrist.setWrist(
                        Constants.WristConstants.pos[RobotState.barge.ordinal()]
                    ); // safe position for elev to move up
                } else if (
                    m_elevator.getPosition() >
                    Constants.ElevatorConstants.wristDownSafeThreshold
                ) {
                    m_wrist.setWrist(targetWristPos); // only move wrist down when elevator up (prevents wrist/bumper collision)
                }
                if (
                    m_wrist.getPosition() >
                    Constants.WristConstants.minElevatorRaisedPos
                ) {
                    m_elevator.setElevator(
                        targetElevatorPos,
                        Constants.ElevatorConstants.algaeSlot
                    );
                }
                if (
                    Math.abs(m_elevator.getPosition() - targetElevatorPos) <
                    Constants.ElevatorConstants.posTolerance
                ) {
                    //robotContainer.runningCommand = false;
                    m_claw.setClaw(Constants.EndEffectorConstants.velocity);
                }
                if (
                    !m_claw.gamePieceDetected() &&
                    targetState.equals(RobotState.barge)
                ) {
                    m_claw.stopClaw();
                    m_driverController.setRumble(RumbleType.kBothRumble, 0);
                }
                if (m_claw.gamePieceDetected()) {
                    m_driverController.setRumble(RumbleType.kBothRumble, 1);
                    if (!timer1.isRunning()) {
                        timer1.restart();
                    }
                }
            }
        }
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {
        robotContainer.runningCommand = false;
        m_driverController.setRumble(RumbleType.kBothRumble, 0);
        timer1.stop();
        timer2.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer1.hasElapsed(timerDelay) || ended;
    }
}
