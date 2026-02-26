package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController driver = new XboxController(0); // Initializes the driver controller.

  // Limits the acceleration of the drivetrain by smoothing controller inputs.
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(Drivetrain.maxAngAccTeleop / Drivetrain.maxAngVelTeleop);

  private double speedScaleFactor = 0.6; // Scales the translational speed of the robot that results from controller inputs. 1.0 corresponds to full speed. 0.0 is fully stopped.
  private double rotationScaleFactor = 0.3; // Scales the rotational speed of the robot that results from controller inputs. 1.0 corresponds to full speed. 0.0 is fully stopped.
  private boolean boostMode = false; // Stores whether the robot is at 100% speed (boost mode), or at ~65% speed (normal mode).
  private boolean swerveLock = false; // Controls whether the swerve drive is in x-lock (for defense) or is driving. 

  // Initializes the different subsystems of the robot.
  private final Drivetrain swerve = new Drivetrain(); // Contains the Swerve Modules, Gyro, Path Follower, Target Tracking, Odometry, and Vision Calibration.
  private final Climber climber = new Climber(); // Initializes the Climber subsystem.
  private final Shooter shooter = new Shooter(); // Initializes the Shooter subsystem.
  private final Indexer indexer = new Indexer(); // Initializes the Indexer subsystem.
  private final Intake intake = new Intake(); // Initializes the Intake subsystem. 

  // Auto Variables
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String auto1 = "Fuel Collection Via Neutral Zone, Right Side Start."; 
  private static final String auto2 = "Fuel Collection Via Neutral Zone, Left Side Start."; 
  private static final String auto3 = "Fuel Collection Via Neutral Zone, Center Start."; 
  private String autoSelected;
  private int autoStage = 1;
  private boolean autoCompleted = false;

  private final double nearTrenchX = 182.11*0.0254;
  private final double farTrenchX = Drivetrain.fieldLength - nearTrenchX;
  private final double trenchTolerance = 0.5;
  private boolean isScoring = false;

  public void robotInit() { 
    // Configures the auto chooser on the dashboard.
    autoChooser.setDefaultOption(auto1, auto1);
    autoChooser.addOption(auto2, auto2);
    autoChooser.addOption(auto3, auto3);
    SmartDashboard.putData("Autos", autoChooser);

    // Auto 1 Paths : Fuel Collection from Neutral Zone, Right Starting Position. 0-2
    swerve.loadPath("neutral zone right travelling to shooting position", 0.0, 0.0, 0.0, -90.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("neutral zone right travelling to zone", 0.0, 0.0, 0.0, 50.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("neutral zone right travelling to shooting position 2", 0.0, 0.0, 0.0, 180.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    // Auto 2 Paths : Fuel Collection from Neutral Zone, Left Starting Position. 3-5
    swerve.loadPath("neutral zone left travelling to shooting position", 0.0, 0.0, 0.0, 90.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("neutral zone left travelling to zone", 0.0, 0.0, 0.0, -50.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("neutral zone left travelling to shooting position 2", 0.0, 0.0, 0.0, 180.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    // Auto 3 Paths : Climbing Auto, Center Starting Position. 6-9
    swerve.loadPath("climbing travelling to shooting position", 0.0, 0.0, 0.0, 180.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("climbing travelling to depot", 0.0, 0.0, 0.0, -15.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("climbing travelling to shooting position 2", 0.0, 0.0, 0.0, 90.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    swerve.loadPath("climbing travelling to tower", 0.0, 0.0, 0.0, -15.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    // Auto _ Paths : Fuel Collection from Human Player, _____ Starting Position.
    swerve.loadPath("fuel collection via human player", 0.0, 0.0, 0.0, -90.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    runAll(); // Helps prevent loop overruns on startup by running every command before the match starts.
    SignalLogger.enableAutoLogging(false);
  }

  public void robotPeriodic() {
    // Publishes information about the robot and robot subsystems to the Dashboard.
    swerve.updateDash();
    climber.updateDash();
    shooter.updateDash();
    indexer.updateDash();
    intake.updateDash();
    updateDash();
  }

  public void autonomousInit() {
    climber.init(); 
    indexer.init();
    intake.init();

    autoCompleted = true;
    autoStage = 1;
    autoSelected = autoChooser.getSelected();
    switch (autoSelected) {
      case auto1:
        // AutoInit 1 code goes here.
        swerve.pushCalibration(true, -90.0); // Updates the robot's position on the field.
        swerve.resetPathController(0); 
      break;

      case auto2:
        // AutoInit 2 code goes here.
        swerve.pushCalibration(true, 90.0); // Updates the robot's position on the field.
        swerve.resetPathController(3); 
      break;

      case auto3:
        // AutoInit 3 code goes here.
        swerve.pushCalibration(true, 180.0); // Updates the robot's position on the field.
        swerve.resetPathController(6); 
      break;
    }
  }

  public void autonomousPeriodic() {
    climber.perioidic();
    indexer.periodic();
    intake.periodic();

    if ((nearTrenchX - trenchTolerance < swerve.getXPos() && swerve.getXPos() < nearTrenchX + trenchTolerance) || (farTrenchX - trenchTolerance < swerve.getXPos() && swerve.getXPos() < farTrenchX + trenchTolerance)) {
      shooter.lowerHood();
    } else if (swerve.getXPos() < nearTrenchX - trenchTolerance) {
      shooter.setHoodPosition(calcHoodPosition());
    } else {
      shooter.setHoodPosition(shooter.hoodMaxPosition);
    }

    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addVisionEstimate(limelightIndex, true); // Checks to see if there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    }
    switch (autoSelected) {
      case auto1:
        switch (autoStage) {
          case 1:
            // Auto 1, Stage 1 code goes here.
            swerve.followPath(0); // Brings the robot to a shooting position.
            shooter.spinUp(); // Turns the shooter on.
            if (swerve.atPathEndpoint(0)) {
              autoStage = 2; // Advances to the next stage once the robot has gotten to the shooting position.
            }
          break;

          case 2:
            // Auto 1, Stage 2 code goes here.
            swerve.aimDrive(0.0, 0.0, getHubHeading(), true); // Rotates the robot to a rotation where it'll have the least misses.
            if (shooter.isReady() && swerve.atDriveGoal()) {
              autoStage = 3; // Advances to the next stage once the robot has started shooting.
            }
          break;

          case 3:
            // Auto 1, Stage 3 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            indexer.start(); // Turns on the indexer.
            if (indexer.getHopperTimer() > 3.0) {
              shooter.spinDown(); // Turns the shooter off.
              indexer.stop(); // Turns the indexer off.
              swerve.resetPathController(1); 
              autoStage = 4; // Advances to the next stage once the robot has finished shooting.
            }
          break;

          case 4:
            // Auto 1, Stage 4 code goes here.
            swerve.followPath(1); // Brings the robot to the neutral zone to collect fuel.
            if (swerve.getXPos() > 6.5) {
                intake.rightIntake(); // When the X position is greater than 6.5, the right intake will deploy.
            }
            if (swerve.atPathEndpoint(1) && intake.isReady()) {
              autoStage = 5; // Advances to the next stage once the robot has gotten to the neutral zone.
            }
          break;

          case 5:
            // Auto 1, Stage 5 code goes here.
            swerve.drive(0.0, 1.0, 0.0, true, 0.0, 0.0); // Moves the robot in the neutral zone, collecting fuel.
            if (swerve.getYPos() >= 3.575) {
              swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0); // Holds the robot still.
              intake.stow(); // Stows the intake.
              swerve.resetPathController(2);
              autoStage = 6; // Advances to the next stage once the robot has finished intaking.
            }
          break;

          case 6:
            // Auto 1, Stage 6 code goes here.
            swerve.followPath(2); // Brings the robot back to a shooting position from the neutral zone.
            shooter.spinUp(); // Turns the shooter on.
            if (swerve.atPathEndpoint(2)) {
              autoStage = 7; // Advances to the next stage once the robot has reached the shooting position.
            }
          break;

          case 7:
            // Auto 1, Stage 7 code goes here.
            swerve.aimDrive(0.0, 0.0, getHubHeading(), true); // Rotates the robot to a rotation where it'll have the least misses.
            if (shooter.isReady() && swerve.atDriveGoal()) {
              autoStage = 8; // Advances to the next stage once the robot has started shooting.
            }
          break;
          
          case 8:
            // Auto 1, Stage 8 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            indexer.start(); // Turns on the indexer.
            if (indexer.getHopperTimer() > 3.0) {
              shooter.spinDown(); // Turns the shooter off.
              indexer.stop(); // Turns the indexer off.
            }
          break;
        }
      break; 

      case auto2:
        switch (autoStage) {
          case 1:
            // Auto 2, Stage 1 code goes here.
            swerve.followPath(3); // Brings the robot to a shooting position.
            shooter.spinUp(); // Turns the shooter on.
            if (swerve.atPathEndpoint(3)) {
              autoStage = 2; // Advances to the next stage once the robot has gotten to the shooting position.
            }
          break;

          case 2:
            // Auto 2, Stage 2 code goes here.
            swerve.aimDrive(0.0, 0.0, getHubHeading(), true); // Rotates the robot to a rotation where it'll have the least misses.
            if (shooter.isReady() && swerve.atDriveGoal()) {
              autoStage = 3; // Advances to the next stage once the robot has started shooting.
            }
          break;

          case 3:
            // Auto 2, Stage 3 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            indexer.start(); // Turns on the indexer.
            if (indexer.getHopperTimer() > 3.0) {
              shooter.spinDown(); // Turns the shooter off.
              indexer.stop(); // Turns the indexer off.
              swerve.resetPathController(4); 
              autoStage = 4; // Advances to the next stage once the robot has finished shooting.
            }
          break;

          case 4:
            // Auto 2, Stage 4 code goes here.
            swerve.followPath(4); // Brings the robot to the neutral zone to collect fuel.
            if (swerve.getXPos() > 6.5) {
                intake.leftIntake(); // When the X position is less than 6.5, the left intake will deploy.
            }
            if (swerve.atPathEndpoint(4) && intake.isReady()) {
              autoStage = 5; // Advances to the next stage once the robot has gotten to the neutral zone.
            }
          break;

          case 5:
            // Auto 2, Stage 5 code goes here.
            swerve.drive(0.0, 1.0, 0.0, true, 0.0, 0.0); // Moves the robot in the neutral zone, collecting fuel.
            if (swerve.getYPos() <= 4.475) {
              swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0); // Holds the robot still.
              intake.stow(); // Stows the intake.
              swerve.resetPathController(5);
              autoStage = 6; // Advances to the next stage once the robot has finished intaking.
            }
          break;

          case 6:
            // Auto 2, Stage 6 code goes here.
            swerve.followPath(5); // Brings the robot back to a shooting position from the neutral zone.
            shooter.spinUp(); // Turns the shooter on.
            if (swerve.atPathEndpoint(5)) {
              autoStage = 7; // Advances to the next stage once the robot has reached the shooting position.
            }
          break;

          case 7:
            // Auto 2, Stage 7 code goes here.
            swerve.aimDrive(0.0, 0.0, getHubHeading(), true); // Rotates the robot to a rotation where it'll have the least misses.
            if (shooter.isReady() && swerve.atDriveGoal()) {
              autoStage = 8; // Advances to the next stage once the robot has started shooting.
            }
          break;
          
          case 8:
            // Auto 2, Stage 8 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            indexer.start(); // Turns on the indexer.
            if (indexer.getHopperTimer() > 3.0) {
              shooter.spinDown(); // Turns the shooter off.
              indexer.stop(); // Turns the indexer off.
            }
          break;
        }
      break; 

      case auto3:
        switch (autoStage) {
          case 1:
            // Auto 3, Stage 1 code goes here.
            swerve.followPath(6); // Brings the robot to a shooting position.
            shooter.spinUp(); // Turns the shooter on.
            if (swerve.atPathEndpoint(6)) {
              autoStage = 2; // Advances to the next stage once the robot has gotten to the shooting position.
            }
          break;

          case 2:
            // Auto 3, Stage 2 code goes here.
            swerve.aimDrive(0.0, 0.0, getHubHeading(), true); // Rotates the robot to a rotation where it'll have the least misses.
            if (shooter.isReady() && swerve.atDriveGoal()) {
              autoStage = 3; // Advances to the next stage once the robot has started shooting.
            }
          break;

          case 3:
            // Auto 3, Stage 3 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            indexer.start(); // Turns on the indexer.
            if (indexer.getHopperTimer() > 3.0) {
              shooter.spinDown(); // Turns the shooter off.
              indexer.stop(); // Turns the indexer off.
              swerve.resetPathController(7); 
              autoStage = 4; // Advances to the next stage once the robot has finished shooting.
            }
          break;

          case 4:
            // Auto 3, Stage 4 code goes here.
            swerve.followPath(7); // Brings the robot to the depot.
            if (swerve.getXPos() < 1.5) {
                intake.leftIntake(); // When the X position is less than 1.5, the left intake will deploy.
            }
            if (swerve.atPathEndpoint(7) && intake.isReady()) {
              autoStage = 5; // Advances to the next stage once the robot has gotten to the neutral zone.
            }
          break;

          case 5:
            // Auto 3, Stage 5 code goes here.
            swerve.drive(0.0, 1.0, 0.0, true, 0.0, 0.0); // Moves the robot in the depot, collecting fuel.
            if (swerve.getYPos() >= 6.1) {
              swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0); // Holds the robot still.
              intake.stow(); // Stows the intake.
              swerve.resetPathController(8);
              autoStage = 6; // Advances to the next stage once the robot has finished intaking.
            }
          break;

          case 6:
            // Auto 3, Stage 6 code goes here.
            swerve.followPath(8); // Brings the robot back to a shooting position from the neutral zone.
            shooter.spinUp(); // Turns the shooter on.
            if (swerve.atPathEndpoint(8)) {
              autoStage = 7; // Advances to the next stage once the robot has reached the shooting position.
            }
          break;

          case 7:
            // Auto 3, Stage 7 code goes here.
            swerve.aimDrive(0.0, 0.0, getHubHeading(), true); // Rotates the robot to a rotation where it'll have the least misses.
            if (shooter.isReady() && swerve.atDriveGoal()) {
              autoStage = 8; // Advances to the next stage once the robot has started shooting.
            }
          break;
          
          case 8:
            // Auto 3, Stage 8 code goes here.
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Holds the robot still.
            indexer.start(); // Turns on the indexer.
            if (indexer.getHopperTimer() > 3.0) {
              shooter.spinDown(); // Turns the shooter off.
              indexer.stop(); // Turns the indexer off.
              climber.moveUp(); // Moves the climber up.
              autoStage = 9; // Moves onto the next stage once the robot has finished shooting.
            }
          break;

          case 9:
            // Auto 3, Stage 9 code goes here.
            swerve.followPath(9); // Brings the robot to the tower to climb.
            if (swerve.atPathEndpoint(9)) {
              climber.moveDown(); // Moves the climber down so the robot is actually climbing the rung.
            }
          break;
        }
      break;
    }
  }
  
  public void teleopInit() {
    swerve.pushCalibration(true, swerve.getFusedAng()); // Updates the robot's position on the field.
    climber.init(); 
    indexer.init();
    intake.init();
  }

  public void teleopPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addVisionEstimate(limelightIndex, true); // Checks to see ifs there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    }

    climber.perioidic(); 
    indexer.periodic();
    intake.periodic();

    if ((nearTrenchX - trenchTolerance < swerve.getXPos() && swerve.getXPos() < nearTrenchX + trenchTolerance) || (farTrenchX - trenchTolerance < swerve.getXPos() && swerve.getXPos() < farTrenchX + trenchTolerance)) {
      shooter.lowerHood();
    } else if (swerve.getXPos() < nearTrenchX - trenchTolerance) {
      shooter.setHoodPosition(calcHoodPosition());
    } else {
      shooter.setHoodPosition(shooter.hoodMaxPosition);
    }

    if (driver.getRawButtonPressed(4)) {
      shooter.spinUp(); 
      isScoring = swerve.getXPos() < nearTrenchX - trenchTolerance;
      if (isScoring) {
        swerve.resetDriveController(getHubHeading());
      }
    }
    if (driver.getRawButtonReleased(4)) {
      shooter.spinDown();
      indexer.stop();
    }

    if (driver.getLeftBumperButtonPressed()) {
      if (intake.getMode() == Intake.Mode.LEFT) {
        intake.stow();
      } else {
        intake.leftIntake();
      }
    } else if (driver.getRightBumperButtonPressed()) {
      if (intake.getMode() == Intake.Mode.RIGHT) {
        intake.stow();
      } else {
        intake.rightIntake();
      }
    }

    if (driver.getRightTriggerAxis() > 0.25) {
      intake.stow();
      climber.moveUp();
    }
    if (driver.getLeftTriggerAxis() > 0.25) {
      intake.stow();
      climber.moveDown(); 
    }

    if (driver.getRawButtonPressed(1)) boostMode = true; // A button sets boost mode. (100% speed up from default of 60%).
    if (driver.getRawButtonPressed(2)) boostMode = false; // B Button sets default mode (60% of full speed).
    if (boostMode) {
      speedScaleFactor = 1.0;
    } else {
      speedScaleFactor = 0.6;
    }
    
    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftY(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double yVel = yAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftX(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(-driver.getRightX(), 0.05)*rotationScaleFactor)*Drivetrain.maxAngVelTeleop;

    if (driver.getRawButton(3)) { // X button
      swerveLock = true; // Pressing the X-button causes the swerve modules to lock (for defense).
    } else if (Math.abs(driver.getLeftY()) >= 0.05 || Math.abs(driver.getLeftX()) >= 0.05 || Math.abs(driver.getRightX()) >= 0.05) {
      swerveLock = false; // Pressing any joystick more than 5% will cause the swerve modules stop locking and begin driving.
    }

    if (swerveLock) {
      swerve.xLock(); // Locks the swerve modules (for defense).
    } else if (driver.getRawButton(4)) {
      if (isScoring) {
        swerve.aimDrive(xVel, yVel, getHubHeading(), true);
        if (shooter.hoodIsInPosition() && shooter.shooterIsAtSpeed() && swerve.atDriveGoal()) {
          indexer.start();
        } else {
          indexer.stop();
        }
      } else {
        swerve.drive(xVel, yVel, angVel, true, 0.0, 0.0);
        if (shooter.hoodIsInPosition() && shooter.shooterIsAtSpeed()) {
          indexer.start();
        } else {
          indexer.stop();
        }
      }
    } else {
      swerve.drive(xVel, yVel, angVel, true, 0.0, 0.0); // Drive at the velocity demanded by the controller.
    }

    // The following 3 calls allow the user to calibrate the position of the robot based on April Tag information. Should be called when the robot is stationary. Button 7 is "View", the right center button.
    if (driver.getRawButtonPressed(7)) {
      swerve.calcPriorityLimelightIndex();
      swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
    }
    if (driver.getRawButton(7)) swerve.addCalibrationEstimate(swerve.getPriorityLimelightIndex(), false); // Left center button
    if (driver.getRawButtonReleased(7)) swerve.pushCalibration(false, 0.0); // Updates the position of the robot on the field based on previous calculations.  

    if (driver.getRawButtonPressed(8)) swerve.resetGyro(); // Right center button re-zeros the angle reading of the gyro to the current angle of the robot. Should be called if the gyroscope readings are no longer well correlated with the field.
  }
  
  public void disabledInit() { 
    swerve.calcPriorityLimelightIndex();
    swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
  }

  public void disabledPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    autoSelected = autoChooser.getSelected();
    if (!autoCompleted) {
      switch (autoSelected) {
        case auto1:
          swerve.updateVisionHeading(true, -90.0); // Updates the Limelight with a known heading based on the starting position of the robot on the field.
        break;

        case auto2:
          swerve.updateVisionHeading(true, 90.0); // Updates the Limelight with a known heading based on the starting position of the robot on the field.
        break;
        
        case auto3:
          swerve.updateVisionHeading(true, 180.0); // Updates the Limelight with a known heading based on the starting position of the robot on the field.
        break;
      }
    } else {
      swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    }
    swerve.addCalibrationEstimate(swerve.getPriorityLimelightIndex(), true);
  }

  public double getHubHeading() {
    double hubX = 182.11 * 0.0254; // The x-position of the hub on the field in meters.
    double hubY = 158.84 * 0.0254; // The y-position of the hub on the field in meters.
    double robotX = swerve.getXPos(); // The current x-position of the robot on the field in meters.
    double robotY = swerve.getYPos(); // The current y-position of the robot on the field in meters.

    if (robotX > hubX) {
      return Math.toDegrees(Math.atan((hubY - robotY) / (hubX - robotX))) - 90.0; // Returns the heading from the robot to the hub in degrees.
    } else if (robotX < hubX) {
      return Math.toDegrees(Math.atan((hubY - robotY) / (hubX - robotX))) + 90.0; // Returns the heading from the robot to the hub in degrees.
    } else {
      if (robotY > hubY) {
        return 0.0;
      } else {
        return 180.0;
      }
    }
  }

  private double[] distanceArray = {1.0, 2.0, 5.0, 5.1, 8.5}; // Distance array (need tested👈)
  private double[] hoodArray = {0.12, 0.10, 0.08, 0.06, 0.04}; // Hood array (need tested👈)
  public double calcHoodPosition() {
    double hubX = 182.11 * 0.0254; // The x-position of the hub on the field in meters.
    double hubY = 158.84 * 0.0254; // The y-position of the hub on the field in meters.
    double robotX = swerve.getXPos(); // The current x-position of the robot on the field in meters.
    double robotY = swerve.getYPos(); // The current y-position of the robot
    double distance = Math.sqrt(Math.pow(hubX - robotX, 2) + Math.pow(hubY - robotY, 2)); // distance to hub
    
    if (distance >= distanceArray[distanceArray.length - 1]) {
      return hoodArray[hoodArray.length - 1]; // Return RPM for largest distance
    } 
    else if (distance <= distanceArray[0]) {
      return hoodArray[0]; // Return RPM for smallest distance
    } 
    else {
      int lowerIndex = -1; // Index for distance immediately smaller than current distance
      for (int i = 0; i < distanceArray.length - 1; i++) {
        if (distanceArray[i + 1] > distance && lowerIndex == -1) {
          lowerIndex = i;
        }
      } 
      return hoodArray[lowerIndex] + ((hoodArray[lowerIndex + 1] - hoodArray[lowerIndex]) / (distanceArray[lowerIndex + 1] - distanceArray[lowerIndex])) * (distance - distanceArray[lowerIndex]);
    }
  }

  // Publishes information to the dashboard.
  public void updateDash() {
    //SmartDashboard.putBoolean("Boost Mode", boostMode);
    //SmartDashboard.putNumber("Speed Scale Factor", speedScaleFactor);
    //SmartDashboard.putNumber("Auto Stage", autoStage);
  }

  // Helps prevent loop overruns on startup by running every user created command in every class before the match starts. Not sure why this helps, but it does.
  public void runAll() { 
    swerve.resetDriveController(0.0);
    swerve.xLock();
    swerve.aimDrive(-3.0, 2.0, 105.0, false);
    swerve.driveTo(1.0, -2.0, -75.0);
    swerve.resetPathController(0);
    swerve.followPath(0);
    swerve.addCalibrationEstimate(0, false);
    swerve.pushCalibration(true, 180.0);
    swerve.resetCalibration();
    swerve.resetGyro();
    swerve.updateVisionHeading(true, 180.0);
    swerve.addVisionEstimate(0, true);
    swerve.updateOdometry();
    swerve.drive(0.01, 0.0, 0.0, true, 0.0, 0.0);
    System.out.println("swerve atDriveGoal: " + swerve.atDriveGoal());
    System.out.println("swerve atPathEndpoint: " + swerve.atPathEndpoint(0));
    System.out.println("swerve getAngVel: " + swerve.getAngVel());
    System.out.println("swerve getCalibrationTimer: " + swerve.getCalibrationTimer());
    System.out.println("swerve getAccurateCalibrationTimer: " + swerve.getAccurateCalibrationTimer());
    System.out.println("swerve getFusedAng: " + swerve.getFusedAng());
    System.out.println("swerve getGyroAng: " + swerve.getGyroAng());
    System.out.println("swerve getGyroPitch: " + swerve.getGyroPitch());
    System.out.println("swerve getGyroRoll: " + swerve.getGyroRoll());
    System.out.println("swerve getGyroAngVel: " + swerve.getGyroAngVel());
    System.out.println("swerve getGyroPitchVel: " + swerve.getGyroPitchVel());
    System.out.println("swerve getGyroRollVel: " + swerve.getGyroRollVel());
    System.out.println("swerve getPathAngleError: " + swerve.getPathAngleError());
    System.out.println("swerve getPathPosError: " + swerve.getPathPosError());
    System.out.println("swerve getXPos: " + swerve.getXPos());
    System.out.println("swerve getXVel: " + swerve.getXVel());
    System.out.println("swerve getYPos: " + swerve.getYPos());
    System.out.println("swerve getYVel: " + swerve.getYVel());
    System.out.println("swerve isBlueAlliance: " + swerve.isBlueAlliance());
    System.out.println("swerve isRedAlliance: " + swerve.isRedAlliance());
    System.out.println("swerve getGyroPitch: " + swerve.getGyroPitch());
    System.out.println("swerve getAngleDist: " + swerve.getAngleDistance(30.0, -120.0));
    swerve.calcPriorityLimelightIndex();
    System.out.println("swerve getPriorityLimelightIndex: " + swerve.getPriorityLimelightIndex());
    swerve.updateDash();

    climber.init();
    climber.perioidic();
    climber.moveUp();
    climber.moveDown();
    System.out.println("climber getMode: " + climber.getMode().toString());
    System.out.println("climber atDesiredPosition: " + climber.atDesiredPosition());
    System.out.println("climber getPosition: " + climber.getPosition());
    System.out.println("climber getVelocity: " + climber.getVelocity());
    climber.updateDash();
    
    shooter.spinUp();
    shooter.spinDown();
    shooter.setShootingRPM(5800.0);
    shooter.setHoodPosition(calcHoodPosition());
    shooter.lowerHood();
    System.out.println("shooter hoodIsInPosition: " + shooter.hoodIsInPosition());
    System.out.println("shooter getHoodPosition: " + shooter.getHoodPosition());
    System.out.println("shooter isAtSpeed(): " + shooter.shooterIsAtSpeed());    
    System.out.println("shooter getLeftShooterRPM: " + shooter.getLeftShooterRPM());
    System.out.println("shooter getRightShooterRPM: " + shooter.getRightShooterRPM());
    System.out.println("shooter isReady: " + shooter.isReady());
    shooter.updateDash();

    indexer.init();
    indexer.periodic();
    indexer.start();
    indexer.stop();
    System.out.println("indexer getMode: " + indexer.getMode().toString());
    System.out.println("indexer getHopperSensor: " + indexer.getHopperSensor());
    System.out.println("indexer getShooterSensor: " + indexer.getShooterSensor());
    System.out.println("indexer getJamTimer: " + indexer.getJamTimer());
    System.out.println("indexer getShooterTimer: " + indexer.getShooterTimer());
    System.out.println("indexer getHopperTimer: " + indexer.getHopperTimer());
    indexer.updateDash();

    intake.init();
    intake.periodic();
    intake.leftIntake();
    intake.rightIntake();
    intake.stow();
    System.out.println("intake getMode: " + intake.getMode().toString());
    System.out.println("intake getLeftArmEncoder: " + intake.getLeftArmEncoder());
    System.out.println("intake getRightArmEncoder: " + intake.getRightArmEncoder());
    System.out.println("intake getLeftArmPosition: " + intake.getLeftArmPosition());
    System.out.println("intake getLeftArmVelocity: " + intake.getLeftArmVelocity());
    System.out.println("intake getLeftArmDesiredPosition: " + intake.getLeftArmDesiredPosition());
    System.out.println("intake leftArmInPosition: " + intake.leftArmInPosition());
    System.out.println("intake getRightArmPosition: " + intake.getRightArmPosition());
    System.out.println("intake getRightArmVelocity: " + intake.getRightArmVelocity());
    System.out.println("intake getRightArmDesiredPosition: " + intake.getRightArmDesiredPosition());
    System.out.println("intake rightArmInPosition: " + intake.rightArmInPosition());
    System.out.println("intake isReady: " + intake.isReady());
    intake.updateDash();

    System.out.println("calcHoodAngle: " + calcHoodPosition());
    System.out.println("getHubHeading(): " + getHubHeading());
    updateDash();
  }
}