package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class Shooter {
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX hoodMotor = new TalonFX(9, canivore);  
  private final TalonFX shootMotorRight = new TalonFX(12, canivore);  
  private final TalonFX shootMotorLeft = new TalonFX(11, canivore); 
  private final CANcoder hoodEncoder = new CANcoder(28, canivore); 
  private final StatusSignal<AngularVelocity> shooterVelocityRight;
  private final StatusSignal<AngularVelocity> shooterVelocityLeft;
  private final StatusSignal<Angle> hoodPosition;
  private final VelocityVoltage shooterMotorRightVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
  private final VelocityVoltage shooterMotorLeftVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
  private final MotionMagicTorqueCurrentFOC hoodMotorPositionRequest = new MotionMagicTorqueCurrentFOC(0.0); 
  private final double rpmTol = 200.0; // Can adjust
  private final double hoodTol = 0.005; // Can adjust
  public final double hoodMinPosition = 0.020; // Can adjust
  public final double hoodMaxPosition = 0.115; // Can adjust
  private double shootingRPM = 2800.0; // Can adjust
  private double desiredHoodPosition = hoodMinPosition;

  // Simulation
  private final TalonFXSimState hoodMotorSim = hoodMotor.getSimState();
  private final TalonFXSimState shootMotorRightSim = shootMotorRight.getSimState();
  private final TalonFXSimState shootMotorLeftSim = shootMotorLeft.getSimState();
  private final CANcoderSimState hoodEncoderSim = hoodEncoder.getSimState();
  private double desiredRPMSim = 0;
  public static final double hoodGearRatio = 1;

  // Initialize Shooter: configure motor, and obtain a data for velocity
  public Shooter() {
    configHoodEncoder(hoodEncoder);
    configShootMotor(shootMotorRight, true);
    configShootMotor(shootMotorLeft, false); // Configures the motor with counterclockwise rotation positive.
    configHoodMotor(hoodMotor, false);
    shooterVelocityRight = shootMotorRight.getVelocity();
    shooterVelocityLeft = shootMotorLeft.getVelocity();
    hoodPosition = hoodEncoder.getAbsolutePosition();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, shooterVelocityRight, shooterVelocityLeft, hoodPosition);
	  ParentDevice.optimizeBusUtilizationForAll(shootMotorLeft, hoodMotor, hoodEncoder);
  }
  
  // Turns on motor. Sets the speed of the motor in rotations per minute.
  public void spinUp() {
    desiredRPMSim = shootingRPM;
    shootMotorRight.setControl(shooterMotorRightVelocityRequest.withVelocity(shootingRPM/60.0).withEnableFOC(true));
    shootMotorLeft.setControl(shooterMotorLeftVelocityRequest.withVelocity(shootingRPM/60.0).withEnableFOC(true));
  }

  // Turn off motor.
  public void spinDown() {
    desiredRPMSim = 0;
    shootMotorRight.setControl(shooterMotorRightVelocityRequest.withVelocity(0.0).withEnableFOC(true));
    shootMotorLeft.setControl(shooterMotorLeftVelocityRequest.withVelocity(0.0).withEnableFOC(true));
  }

  public void setShootingRPM(double rpm) {
    if (rpm > 5800.0) {
      shootingRPM = 5800.0;
    } else if (rpm < 600.0) {
      shootingRPM = 600.0;
    } else {
      shootingRPM = rpm;
    }
  }

  public void setHoodPosition(double position) {
    if (position < hoodMinPosition) {
      hoodMotor.setControl(hoodMotorPositionRequest.withPosition(hoodMinPosition));
      desiredHoodPosition = hoodMinPosition;
    } else if (position > hoodMaxPosition) {
      hoodMotor.setControl(hoodMotorPositionRequest.withPosition(hoodMaxPosition));
      desiredHoodPosition = hoodMaxPosition;
    } else {
      hoodMotor.setControl(hoodMotorPositionRequest.withPosition(position));
      desiredHoodPosition = position;
    }
  }

  public void lowerHood() {
    hoodMotor.setControl(hoodMotorPositionRequest.withPosition(hoodMinPosition));
    desiredHoodPosition = hoodMinPosition;
  }

  public boolean hoodIsInPosition() {
    // TODO: Fix simulationPeriodic
    if (Robot.isSimulation()) return true;

    return Math.abs(desiredHoodPosition - getHoodPosition()) < hoodTol;
  }

  public double getHoodPosition() {
    return hoodPosition.refresh().getValueAsDouble();
  }

  // Returns true or false based on whether the shooter motor is near the desired RPM.
  public boolean shooterIsAtSpeed() {
    // use abs() on the left/right shooter to compare magnitudes 
    // otherwise you may have (2800 - -2800) = 5600 rpm which shows as not ready
    return Math.abs(shootingRPM - Math.abs(getLeftShooterRPM())) < rpmTol && Math.abs(shootingRPM - Math.abs(getRightShooterRPM())) < rpmTol;
  }

  // Returns the motor velocity in RPM (Rotations Per Minute)
  public double getLeftShooterRPM() {
    return shooterVelocityLeft.refresh().getValueAsDouble()*60.0;
  }

  // Returns the motor velocity in RPM (Rotations Per Minute)
  public double getRightShooterRPM() {
    return shooterVelocityRight.refresh().getValueAsDouble()*60.0;
  }

  public boolean isReady() {
    return shooterIsAtSpeed() && hoodIsInPosition();
  }

  // Publish Shooter information (Motor state, Velocity) to SmartDashboard.
  public void updateDash() {
    // SmartDashboard.putNumber("Shooter getRightShooterRPM", getRightShooterRPM());
    // SmartDashboard.putNumber("Shooter getLeftShooterRPM", getLeftShooterRPM());
    // SmartDashboard.putBoolean("Shooter shooterIsAtSpeed", shooterIsAtSpeed());
    // SmartDashboard.putNumber("Shooter shootingRPM", shootingRPM);
    // SmartDashboard.putNumber("Shooter getHoodPosition", getHoodPosition());
    // SmartDashboard.putBoolean("Shooter hoodIsInPosition", hoodIsInPosition());
    // SmartDashboard.putNumber("Shooter desiredHoodPosition", desiredHoodPosition);
    // SmartDashboard.putBoolean("Shooter isReady", isReady());
  }

  public void simulationPeriodic() {
    // Very basic - just jump to the desired values
    // Update the motors
    shootMotorRightSim.setRotorVelocity(desiredRPMSim / 60.0);
    shootMotorLeftSim.setRotorVelocity(desiredRPMSim / 60.0);

    // Update the hood position.
    // TODO:
    hoodMotorSim.setRawRotorPosition(desiredHoodPosition/hoodGearRatio);
    hoodEncoderSim.setRawPosition(desiredHoodPosition/hoodGearRatio);
  }

  private void configHoodEncoder(CANcoder CANsensor) {
    CANcoderConfiguration sensorConfigs = new CANcoderConfiguration();

    sensorConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; 
    sensorConfigs.MagnetSensor.MagnetOffset = 0.378662109375;
    sensorConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    CANsensor.getConfigurator().apply(sensorConfigs, 0.03);
  }

  // Configs the motor settings and PID
  private void configShootMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // VelocityVoltage closed-loop control configuration.
    motorConfigs.Slot0.kP = 0.25; // Units: volts per 1 motor rotation per second of error.
    motorConfigs.Slot0.kI = 0.5; // Units: volts per 1 motor rotation per second * 1 second of error.
    motorConfigs.Slot0.kD = 0.0; // Units: volts per 1 motor rotation per second / 1 second of error.
    motorConfigs.Slot0.kV = 0.12; // The amount of voltage required to create 1 motor rotation per second.
    motorConfigs.Slot0.kS = 0.16; // The amount of voltage required to barely overcome static friction.

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }

  // Configs the motor settings and PID
  private void configHoodMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    motorConfigs.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfigs.Feedback.SensorToMechanismRatio = 1.0;
    motorConfigs.Feedback.RotorToSensorRatio = 211.68;

    // MotionMagicTorqueFOC closed-loop control configuration.
    motorConfigs.Slot0.kP = 800.0*211.68/18.75; // Units: amperes per 1 swerve wheel rotation of error.
    motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 swerve wheel rotation * 1 second of error.
    motorConfigs.Slot0.kD = 18.0*211.68/18.75; // Units: amperes per 1 swerve wheel rotation / 1 second of error.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 10.0*5800.0/(60.0*211.68); // Units: rotations per second per second.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 5800.0/(60.0*211.68); // Units: roations per second.

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
}