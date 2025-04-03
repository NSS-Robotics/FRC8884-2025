package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotState;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import java.util.HashSet;
import java.util.List;

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
    private final Claw m_endEffector = new Claw(this);
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

    private final Joystick m_gamePanel = new Joystick(
        OperatorConstants.kOperatorControllerPort
    );

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
    public int postIndex;
    public boolean canAlign;
    public boolean l1_2;

    public SendableChooser<Boolean> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        l1_2 = false;
        postIndex = 0;
        intakeDown = false;
        scoringLevel = RobotState.l4;

        // Configure the trigger bindings
        configureBindings();
        m_swerve.setDefaultCommand(
            new TeleopSwerve(
                m_swerve,
                () -> m_driverController.getRawAxis(translationAxis),
                () -> m_driverController.getRawAxis(strafeAxis),
                () -> m_driverController.getRawAxis(rotationAxis) * 0.87,
                () -> povDown.getAsBoolean()
            )
        );

        NamedCommands.registerCommand(
            "Zero Gyro",
            new InstantCommand(m_swerve::zeroGyro)
        );
        NamedCommands.registerCommand(
            "Align To Station 1",
            new AlignToStation(m_swerve, false)
        );
        NamedCommands.registerCommand(
            "Align To Station 2",
            new AlignToStation(m_swerve, true)
        );
        NamedCommands.registerCommand(
            "Coral Placing",
            new Up(
                this,
                m_elevator,
                m_wrist,
                m_endEffector,
                m_swerve,
                m_driverController,
                () -> true
            )
        );
        NamedCommands.registerCommand(
            "4 Piece Coral Placing",
            new Up4Piece(
                this,
                m_elevator,
                m_wrist,
                m_endEffector,
                m_swerve,
                m_driverController,
                () -> true
            )
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

        NamedCommands.registerCommand(
            "Ground Intake",
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    intakeDown = true;
                }),
                new GroundIntake(
                    this,
                    m_intake,
                    m_indexer,
                    m_wrist,
                    m_endEffector,
                    m_elevator,
                    m_driverController,
                    m_climber,
                    l_leds,
                    false
                )
            )
        );
        NamedCommands.registerCommand(
            "4 Piece Ground Intake",
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    intakeDown = true;
                }),
                new GroundIntake4Piece(
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
        NamedCommands.registerCommand(
            "Ation Ground Intake",
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    intakeDown = false;
                }),
                new GroundIntake(
                    this,
                    m_intake,
                    m_indexer,
                    m_wrist,
                    m_endEffector,
                    m_elevator,
                    m_driverController,
                    m_climber,
                    l_leds,
                    true
                )
            )
        );
        NamedCommands.registerCommand(
            "Ation Intake",
            new AutoStationIntake(this, m_intake, l_leds)
        );
        NamedCommands.registerCommand(
            "Align Station Intake",
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new AlignToStation(m_swerve, false),
                    new TurnAround(m_swerve)
                ),
                new StationIntake(m_elevator, m_wrist, m_endEffector)
            )
        );
        NamedCommands.registerCommand(
            "Station Intake",
            new StationIntake(m_elevator, m_wrist, m_endEffector)
        );
        NamedCommands.registerCommand(
            "Half Intake",
            new HalfIntake(
                // tommy and tracy sitting in a tree
                this,
                m_intake,
                m_indexer,
                m_driverController,
                m_climber,
                l_leds
            )
        );
        NamedCommands.registerCommand(
            "Ground Intake Pedro Backy",
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    intakeDown = true;
                }),
                new ParallelDeadlineGroup(
                    new GroundIntake(
                        this,
                        m_intake,
                        m_indexer,
                        m_wrist,
                        m_endEffector,
                        m_elevator,
                        m_driverController,
                        m_climber,
                        l_leds,
                        false
                    ),
                    new TeleopSwerve(
                        m_swerve,
                        () -> 0.5,
                        () -> 0,
                        () -> 0,
                        () -> true
                    )
                )
            )
        );
        NamedCommands.registerCommand(
            "Resting Climb",
            new RestingClimb(m_climber)
        );
        NamedCommands.registerCommand(
            "Coral In Claw",
            new CoralInClaw(m_endEffector)
        );
        NamedCommands.registerCommand(
            "Intake Down",
            new InstantCommand(() ->
                m_intake.setPivot(
                    Constants.IntakeConstants.autoIntakePosition,
                    1
                )
            )
        );

        // ALGAE COMMANDS
        NamedCommands.registerCommand(
            "Align Algae",
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    this.isCoral = false;
                    this.scoringLevel = RobotState.algaeReefLow;
                }),
                new Align2(
                    this,
                    m_swerve,
                    m_elevator,
                    m_wrist,
                    m_endEffector,
                    l_limelightLow,
                    m_driverController
                )
            )
        );
        NamedCommands.registerCommand(
            "Reef Intake",
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    this.isCoral = false;
                    this.scoringLevel = RobotState.algaeReefLow;
                }),
                new Up(
                    this,
                    m_elevator,
                    m_wrist,
                    m_endEffector,
                    m_swerve,
                    m_driverController,
                    () -> true
                )
            )
        );
        NamedCommands.registerCommand(
            "Barge Shoot",
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    this.isCoral = false;
                    this.scoringLevel = RobotState.barge;
                }),
                new Up(
                    this,
                    m_elevator,
                    m_wrist,
                    m_endEffector,
                    m_swerve,
                    m_driverController,
                    () -> true
                )
            )
        );

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        m_chooser.addOption("on", true);
        m_chooser.setDefaultOption("off", false);

        SmartDashboard.putData("Limelight Light", m_chooser);
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
            .rightBumper()
            .whileTrue(
                new HalfIntake(
                    this,
                    m_intake,
                    m_indexer,
                    m_driverController,
                    m_climber,
                    l_leds
                )
            );
        m_driverController
            .y()
            .whileTrue(new InstantCommand(m_swerve::zeroGyro));
        m_driverController
            .a()
            .whileTrue(
                // new SequentialCommandGroup(
                //     new ParallelDeadlineGroup(
                //         new AlignToStation(m_swerve),
                //         new AutoStationIntake(this, m_intake)
                //     ),
                //     new ParallelDeadlineGroup(
                //         new WaitCommand(1.5),
                //         new AutoStationIntake(this, m_intake)
                //     )
                // )
                // new ParallelCommandGroup(
                //     new SequentialCommandGroup(
                //         new AlignToStation(m_swerve)
                //         //new TurnAround(m_swerve)
                //     ),
                //     new StationIntake(m_elevator, m_wrist, m_endEffector)
                // )
                new AutoStationIntake(this, m_intake, l_leds)
            );

        // m_driverController
        //     .povLeft()
        //     .whileTrue(new AlignToStation(m_swerve, true));
        m_driverController
            .povLeft()
            .onTrue(
                new Up(
                    this,
                    m_elevator,
                    m_wrist,
                    m_endEffector,
                    m_swerve,
                    m_driverController,
                    () -> true
                )
            );
        m_driverController
            .leftTrigger()
            .whileTrue(
                new ConditionalCommand(
                    new AutoStationIntake(this, m_intake, l_leds),
                    new GroundIntake(
                        this,
                        m_intake,
                        m_indexer,
                        m_wrist,
                        m_endEffector,
                        m_elevator,
                        m_driverController,
                        m_climber,
                        l_leds,
                        false
                    ),
                    () -> l1_2
                )
            );

        m_driverController
            .x()
            .onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(l_leds::stationAlignLeds),
                    // new AlignToStation(m_swerve),
                    new StationIntake(m_elevator, m_wrist, m_endEffector)
                )
            );

        m_driverController
            .rightTrigger()
            .onTrue(
                new ConditionalCommand(
                    new ParallelDeadlineGroup(
                        new WaitCommand(2),
                        new IntakeL1(this, m_intake)
                    ),
                    new SequentialCommandGroup(
                        new InstantCommand(l_leds::alignLeds),
                        new ConditionalCommand(
                            new InstantCommand(),
                            new Align2(
                                this,
                                m_swerve,
                                m_elevator,
                                m_wrist,
                                m_endEffector,
                                l_limelightLow,
                                m_driverController
                            ).asProxy(),
                            () ->
                                scoringLevel.equals(RobotState.barge) ||
                                scoringLevel.equals(RobotState.processor)
                        ),
                        new InstantCommand(() -> l_leds.score(m_elevator)),
                        new ConditionalCommand(
                            new Up(
                                this,
                                m_elevator,
                                m_wrist,
                                m_endEffector,
                                m_swerve,
                                m_driverController,
                                () -> true
                            ),
                            new InstantCommand(),
                            () ->
                                scoringLevel.equals(RobotState.barge) ||
                                scoringLevel.equals(RobotState.processor) ||
                                canAlign
                        )
                    ),
                    () -> l1_2
                )
            );
        m_driverController
            .back()
            .whileTrue(new ToggleIntake(this, m_intake, m_climber));
        m_driverController
            .start()
            .onTrue(
                new InstantCommand(() ->
                    m_intake.setPivot(
                        Constants.IntakeConstants.climbPosition,
                        0
                    )
                )
            );
        // m_driverController
        //     .rightBumper()
        //     .onTrue(
        //         new SequentialCommandGroup(
        //             new InstantCommand(() -> l_leds.score(m_elevator)),
        //             new Up(
        //                 this,
        //                 m_elevator,
        //                 m_wrist,
        //                 m_endEffector,
        //                 m_swerve,
        //                 m_driverController,
        //                 () -> true
        //             )
        //         )
        //     );
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

        // Game panel controls
        HashSet<Integer> highAlgae = new HashSet<>(List.of(0, 3, 4, 6, 9, 10));

        for (int i = 0; i < 12; i++) {
            final int idx = i;
            JoystickButton post = new JoystickButton(m_gamePanel, i + 1);

            post.onTrue(
                new InstantCommand(() -> {
                    this.postIndex = idx;
                    this.isLeft = ((idx % 6) + 1) % 2 != 0 ? idx < 6 : idx >= 6;

                    if (!this.isCoral) {
                        this.scoringLevel = highAlgae.contains(idx)
                            ? RobotState.algaeReefHigh
                            : RobotState.algaeReefLow;
                    }

                    l_leds.updateGamePiece();
                })
            );
            NamedCommands.registerCommand(
                String.format("Align %c%d", i < 6 ? 'L' : 'R', (i % 6) + 1),
                Commands.sequence(
                    Commands.runOnce(() -> {
                        this.postIndex = idx;
                    }),
                    new Align2(
                        this,
                        m_swerve,
                        m_elevator,
                        m_wrist,
                        m_endEffector,
                        l_limelightLow,
                        m_driverController
                    )
                )
            );
        }

        new JoystickButton(m_gamePanel, 13).onTrue(
            new InstantCommand(() -> {
                this.scoringLevel = isCoral
                    ? RobotState.l1
                    : RobotState.processor;
                l_leds.L1Leds();
            })
        );

        new JoystickButton(m_gamePanel, 14).onTrue(
            new InstantCommand(() -> {
                this.scoringLevel = isCoral
                    ? RobotState.l2
                    : RobotState.algaeReefLow;
                l_leds.L2Leds();
            })
        );

        new JoystickButton(m_gamePanel, 15).onTrue(
            new InstantCommand(() -> {
                this.scoringLevel = isCoral
                    ? RobotState.l3
                    : RobotState.algaeReefHigh;
                l_leds.L3Leds();
            })
        );

        new JoystickButton(m_gamePanel, 16).onTrue(
            new InstantCommand(() -> {
                this.scoringLevel = isCoral ? RobotState.l4 : RobotState.barge;
                l_leds.L4Leds();
            })
        );

        new JoystickButton(m_gamePanel, 17).onTrue(
            new InstantCommand(() -> {
                if (canChangeGamePiece()) {
                    this.isCoral = false;

                    this.scoringLevel = highAlgae.contains(postIndex)
                        ? RobotState.algaeReefHigh
                        : RobotState.algaeReefLow;

                    l_leds.updateGamePiece();
                }
            })
        );

        new JoystickButton(m_gamePanel, 18).onTrue(
            new InstantCommand(() -> {
                if (canChangeGamePiece()) {
                    this.isCoral = true;
                    l_leds.updateGamePiece();

                    if (scoringLevel.equals(RobotState.barge)) {
                        this.scoringLevel = RobotState.l4;
                    } else if (scoringLevel.equals(RobotState.algaeReefHigh)) {
                        this.scoringLevel = RobotState.l3;
                    } else if (scoringLevel.equals(RobotState.algaeReefLow)) {
                        this.scoringLevel = RobotState.l2;
                    } else if (scoringLevel.equals(RobotState.processor)) {
                        this.scoringLevel = RobotState.l1;
                    }
                }
            })
        );

        JoystickButton climbConfirm = new JoystickButton(m_gamePanel, 19);

        climbConfirm
            .and(m_driverController.povUp())
            .onTrue(
                new SequentialCommandGroup(
                    new UpClimb(m_climber, m_wrist, m_intake, m_elevator)
                )
            );

        climbConfirm
            .and(m_driverController.povDown())
            .onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(l_leds::climbLeds),
                    new DownClimb(m_climber, m_intake)
                )
            );

        new JoystickButton(m_gamePanel, 20).onTrue(
            new InstantCommand(() -> {
                this.l1_2 = !this.l1_2;
                l_leds.ationIntake();
            })
        );
    }

    public boolean canChangeGamePiece() {
        return !m_endEffector.gamePieceDetected() && !runningCommand;
    }

    public Command setupRobot() {
        intakeDown = false;
        return new SequentialCommandGroup(
            new RunWrist(m_wrist, m_elevator, 0.1),
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
