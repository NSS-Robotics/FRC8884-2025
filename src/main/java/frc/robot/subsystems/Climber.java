package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private static TalonFX lMotor = new TalonFX(Constants.ClimberConstants.lMotorID);
    private static TalonFX rMotor = new TalonFX(Constants.ClimberConstants.rMotorID);
    private static Follower leader = new Follower(Constants.ClimberConstants.lMotorID, true);
    private static Slot0Configs slot0configs = new Slot0Configs();
    private static PositionVoltage positionPID;
    
    public Climber() {
        lMotor.clearStickyFaults();
        rMotor.clearStickyFaults();
        lMotor.setNeutralMode(NeutralModeValue.Brake);
        rMotor.setNeutralMode(NeutralModeValue.Brake);
        rMotor.setControl(leader);


        slot0configs.kP = Constants.ClimberConstants.kP;
        slot0configs.kI = Constants.ClimberConstants.kI;
        slot0configs.kD = Constants.ClimberConstants.kD;

        rMotor.getConfigurator().apply(slot0configs);
    }

    public void resetEncoders() {
        lMotor.setPosition(0);
        rMotor.setPosition(0);
    }

    public void setClimber(double position) {
        position = Math.max(0, Math.min(Constants.ElevatorConstants.maxRotations, position));
        positionPID = new PositionVoltage(position);
        lMotor.setControl(positionPID);
    }
}
