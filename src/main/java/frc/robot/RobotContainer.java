package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandDeviceDetails.Msg;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotState;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import java.util.concurrent.CancellationException;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final SendableChooser<Command> autoChooser;
    // The robot's subsystems and commands are defined here...
    private final Climber m_climber = new Climber();
    private final Elevator m_elevator = new Elevator();
    private final Claw m_endEffector = new Claw();
    private final Indexer m_indexer = new Indexer();
    private final Intake m_intake = new Intake();
    private final Limelight l_limelightLow = new Limelight("low");
    private final Limelight l_limelightHigh = new Limelight("high");
    private final Swerve m_swerve = new Swerve(l_limelightLow, l_limelightHigh);
    private final Wrist m_wrist = new Wrist(this);
    private final LED l_leds = new LED(this);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final CommandPS4Controller m_operatorController =
        new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);

    private final int translationAxis = XboxController.Axis.kRightY.value;
    private final int strafeAxis = XboxController.Axis.kRightX.value;
    private final int rotationAxis = XboxController.Axis.kLeftX.value;
    private final Trigger povDown = m_driverController.povDown();
    public boolean isCoral = true;
    public boolean isLeft = true;
    public boolean runningCommand = false;
    public boolean fieldCentric = false;
    public RobotState scoringLevel;
    public boolean intakeDown;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        m_swerve.setDefaultCommand(
            new TeleopSwerve(
                m_swerve,
                () -> m_driverController.getRawAxis(translationAxis),
                () -> m_driverController.getRawAxis(strafeAxis),
                () -> m_driverController.getRawAxis(rotationAxis) * 0.75,
                () -> povDown.getAsBoolean()
            )
        );

        scoringLevel = RobotState.l4;

        NamedCommands.registerCommand(
            "Zero Gyro",
            new InstantCommand(m_swerve::zeroGyro)
        );

        NamedCommands.registerCommand(
            "Target Left",
            new InstantCommand(() -> isLeft = true)
        );
        NamedCommands.registerCommand(
            "Target Right",
            new InstantCommand(() -> isLeft = false)
        );

        NamedCommands.registerCommand("Align", new Align(this, m_swerve));
        NamedCommands.registerCommand(
            "Align To Station",
            new AlignToStation(m_swerve)
        );
        NamedCommands.registerCommand(
            "Coral Placing",
            new Up(this, m_elevator, m_wrist, m_endEffector, m_driverController)
        );
        NamedCommands.registerCommand(
            "Down Elevator",
            new ElevatorDown(this, m_elevator, m_wrist, m_endEffector)
        );

        NamedCommands.registerCommand(
            "Intake",
            new SequentialCommandGroup(
                new StationIntake(m_elevator, m_wrist, m_endEffector)
            )
        );
        intakeDown = false;
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `exampleMethodCommand` when the Xbox controller's B button is
        // pressed,
        // cancelling on release.
        m_driverController
            .povRight()
            .whileTrue(
                new RunWrist(
                    m_wrist,
                    m_elevator,
                    Constants.WristConstants.pos[RobotState.algaeGround.ordinal()]
                )
            );
        m_driverController
            .b()
            .whileTrue(
                new Outtake(
                    this,
                    m_intake,
                    m_indexer,
                    m_wrist,
                    m_elevator,
                    m_endEffector,
                    l_leds
                )
            );
        m_driverController
            .povUp()
            .onTrue(
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.75),
                        new RunServos(m_climber, false)
                    ),
                    new UpClimb(m_climber, m_wrist, m_intake, m_elevator, () ->
                        m_operatorController.cross().getAsBoolean()
                    )
                )
            );

        m_driverController
            .povDown()
            .onTrue(
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.5),
                        new RunServos(m_climber, true)
                    ),
                    new InstantCommand(l_leds::climbLeds),
                    new DownClimb(m_climber, () ->
                        m_operatorController.cross().getAsBoolean()
                    )
                )
            );
        m_driverController.a().onTrue(new ToggleIntake(this, m_intake));
        m_driverController
            .x()
            .onTrue(new StationIntake(m_elevator, m_wrist, m_endEffector));
        m_driverController
            .y()
            .whileTrue(new InstantCommand(m_swerve::zeroGyro));

        m_driverController.povLeft().whileTrue(new Align(this, m_swerve));
        m_driverController
            .leftTrigger()
            .whileTrue(
                new SequentialCommandGroup(
                    new GroundIntake(
                        this,
                        m_intake,
                        m_indexer,
                        m_wrist,
                        m_endEffector,
                        m_elevator,
                        m_driverController,
                        m_climber,
                        l_leds
                    )
                )
            );

        m_driverController
            .x()
            .onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(l_leds::stationAlignLeds),
                    new AlignToStation(m_swerve),
                    new StationIntake(m_elevator, m_wrist, m_endEffector)
                )
            );

        m_driverController
            .rightTrigger()
            .onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(l_leds::alignLeds),
                    new Align(this, m_swerve).asProxy(),
                    new InstantCommand(() -> l_leds.score(m_elevator)),
                    new Up(
                        this,
                        m_elevator,
                        m_wrist,
                        m_endEffector,
                        m_driverController
                    )
                )
            );
        m_driverController
            .rightBumper()
            .onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(() -> l_leds.score(m_elevator)),
                    new Up(
                        this,
                        m_elevator,
                        m_wrist,
                        m_endEffector,
                        m_driverController
                    )
                )
            );
        m_driverController
            .leftBumper()
            .onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(() -> l_leds.score(m_elevator)), // score()
                    // just
                    // tracks
                    // the
                    // elevator
                    // position
                    // and
                    // sets
                    // the
                    // LEDs
                    new ElevatorDown(this, m_elevator, m_wrist, m_endEffector),
                    new InstantCommand(l_leds::stop)
                )
            );

        // operator controls
        m_operatorController
            .square()
            .onTrue(
                new InstantCommand(() -> {
                    if (canChangeGamePiece()) {
                        this.isCoral = true;
                        l_leds.updateGamePiece();
                        if (scoringLevel.equals(RobotState.barge)) {
                            this.scoringLevel = RobotState.l4;
                        } else if (
                            scoringLevel.equals(RobotState.algaeReefHigh)
                        ) {
                            this.scoringLevel = RobotState.l3;
                        } else if (
                            scoringLevel.equals(RobotState.algaeReefLow)
                        ) {
                            this.scoringLevel = RobotState.l2;
                        } else if (scoringLevel.equals(RobotState.processor)) {
                            this.scoringLevel = RobotState.l1;
                        }
                    }
                })
            );
        m_operatorController
            .circle()
            .onTrue(
                new InstantCommand(() -> {
                    if (canChangeGamePiece()) {
                        this.isCoral = false;
                        l_leds.updateGamePiece();
                        if (scoringLevel.equals(RobotState.l4)) {
                            this.scoringLevel = RobotState.barge;
                        } else if (scoringLevel.equals(RobotState.l3)) {
                            this.scoringLevel = RobotState.algaeReefHigh;
                        } else if (scoringLevel.equals(RobotState.l2)) {
                            this.scoringLevel = RobotState.algaeReefLow;
                        } else if (scoringLevel.equals(RobotState.l1)) {
                            this.scoringLevel = RobotState.processor;
                        }
                    }
                })
            );
        m_operatorController
            .L1()
            .whileTrue(
                new InstantCommand(() -> {
                    this.isLeft = true;
                    l_leds.updateGamePiece();
                })
            );
        m_operatorController
            .R1()
            .whileTrue(
                new InstantCommand(() -> {
                    this.isLeft = false;
                    l_leds.updateGamePiece();
                })
            );

        m_operatorController
            .povUp()
            .onTrue(
                new InstantCommand(() -> {
                    this.scoringLevel = isCoral
                        ? RobotState.l4
                        : RobotState.barge;
                    l_leds.L4Leds();
                })
            );
        m_operatorController
            .povRight()
            .onTrue(
                new InstantCommand(() -> {
                    this.scoringLevel = isCoral
                        ? RobotState.l3
                        : RobotState.algaeReefHigh;
                    l_leds.L3Leds();
                })
            );
        m_operatorController
            .povLeft()
            .onTrue(
                new InstantCommand(() -> {
                    this.scoringLevel = isCoral
                        ? RobotState.l2
                        : RobotState.algaeReefLow;
                    l_leds.L2Leds();
                })
            );
        m_operatorController
            .povDown()
            .onTrue(
                new InstantCommand(() -> {
                    this.scoringLevel = isCoral
                        ? RobotState.l1
                        : RobotState.processor;
                    l_leds.L1Leds();
                })
            );
    }

    public boolean canChangeGamePiece() {
        return !m_endEffector.gamePieceDetected() && !runningCommand;
    }

    public Command setupRobot() {
        return new SequentialCommandGroup(
            new RunWrist(m_wrist, m_elevator, 0.1),
            new ParallelDeadlineGroup(
                new WaitCommand(0.5),
                new RunServos(m_climber, true)
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.5),
                new RestingClimb(m_climber)
            ),
            new RunWrist(
                m_wrist,
                m_elevator,
                Constants.WristConstants.pos[RobotState.handoff.ordinal()]
            )
        );
    }

    // public RobotState getRobotState() {

    // return RobotState.l2;
    // }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }
}
