package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;

public class Indexer {
  public enum Mode {FORWARD, IDLE}
  private final CANBus canivore = new CANBus("canivore");
  private final TalonFX hopperIndexMotor = new TalonFX(19, canivore);
  private final TalonFX shooterIndexMotor = new TalonFX(10, canivore);
  private final VoltageOut hopperIndexMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut shooterIndexMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private Mode currMode = Mode.IDLE;
  
  // Initialize indexer: configure motor, start timer, configure sensor, and obtain fuelDetected data
  public Indexer() {
    configIndexMotor(hopperIndexMotor, false);
    configIndexMotor(shooterIndexMotor,true);
	  ParentDevice.optimizeBusUtilizationForAll(hopperIndexMotor, shooterIndexMotor);
  }

  // Runs code once at start: set current state to IDLE, stop shooting, stops motor, and reset the index timer
  public void init() {
    currMode = Mode.IDLE;
  }

  // Runs code periodically: refresh the fuel sensor, run the shooting/jam state machine, and set motor outputs for each state
  public void periodic() {
    switch (currMode) {
      case FORWARD://just going forward
        hopperIndexMotor.setControl(hopperIndexMotorVoltageRequest.withOutput(12.0).withEnableFOC(true));
        shooterIndexMotor.setControl(shooterIndexMotorVoltageRequest.withOutput(12.0).withEnableFOC(true));
      break;

      case IDLE://just stops
        hopperIndexMotor.setControl(hopperIndexMotorVoltageRequest.withOutput(0.0).withEnableFOC(true));
        shooterIndexMotor.setControl(shooterIndexMotorVoltageRequest.withOutput(0.0).withEnableFOC(true));
      break;
    }
  }

  // Marks the Indexer as running forward (not shooting) and resets the jam timer.
  public void start() {
    currMode = Mode.FORWARD;
  }

  //Marks the indexer as idle, stops shooting, and reset the jam timer.
  public void stop() {
    currMode = Mode.IDLE;
  }

  // Returns the current mode that the indexer is in.
  public Mode getMode() {
    return currMode;
  }

  // Publish indexer information (state, sensor, shooting flag, and timer) to SmartDashboard
  public void updateDash() {
    //SmartDashboard.putString("Indexer getMode", getMode().toString());
  }
  
  // Configs the motor settings and PID
  private void configIndexMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
}