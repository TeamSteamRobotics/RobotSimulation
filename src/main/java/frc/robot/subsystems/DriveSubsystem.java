// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants.DriveTalonIDs;
import frc.robot.Constants.PhysicalRobotConstants;

public class DriveSubsystem extends SubsystemBase {
  WPI_TalonSRX leftFront = new WPI_TalonSRX(DriveTalonIDs.leftFrontID);
  WPI_TalonSRX leftBack = new WPI_TalonSRX(DriveTalonIDs.leftBackID);
  WPI_TalonSRX rightFront = new WPI_TalonSRX(DriveTalonIDs.rightFrontID);
  WPI_TalonSRX rightBack = new WPI_TalonSRX(DriveTalonIDs.rightBackID);

  TalonSRXSimCollection leftFrontSim = leftFront.getSimCollection();
  TalonSRXSimCollection rightFrontSim = rightFront.getSimCollection();

  

  SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftFront, leftBack);
  SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightFront, rightBack);

  DifferentialDrive diffDrive = new DifferentialDrive(leftGroup, rightGroup); 

  DifferentialDrivetrainSim diffDriveSim = new DifferentialDrivetrainSim(
    DCMotor.getCIM(2), //two motors per side of the drivetrain 
    PhysicalRobotConstants.kGearRatio,
    4, //Moment of Intertia of Robot in kg * m^2
    26.5, //mass of robot in kg
    Units.inchesToMeters(3), //radius of wheel
    .552976, //trackwidth of robot in m
  // The standard deviations for measurement noise:
    // x and y:          0.001 m
    // heading:          0.001 rad
    // l and r velocity: 0.1   m/s
    // l and r position: 0.005 m
  VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
  );

  AHRS gyro = new AHRS();
  SimDeviceSim gyroSim = new SimDeviceSim("NavX Gyro");

  Field2d fieldSim = new Field2d();
  
  DifferentialDriveOdometry m_odometry;



  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    resetEncoders();
    m_odometry  = new DifferentialDriveOdometry(gyro.getRotation2d());

    SmartDashboard.putData("Field", fieldSim);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(gyro.getRotation2d(),getLeftDistanceMeters(), getRightDistanceMeters());
    fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }

  public void simulationPeriodic() {
    /*Set the inputs to the system. Note that we need to convert
    the [-1, 1] PWM signal to voltage by multiplying it by the
    robot controller voltage.*/
    diffDriveSim.setInputs(leftFront.getMotorOutputVoltage(), rightFront.getMotorOutputVoltage());

    diffDriveSim.update(.02); //update the drivetrain every .02 sec

    // Update sensor values
    leftFrontSim.setQuadratureRawPosition(distanceToNativeUnits(diffDriveSim.getLeftPositionMeters()));
    leftFrontSim.setQuadratureVelocity(velocityToNativeUnits(diffDriveSim.getLeftVelocityMetersPerSecond()));
    rightFrontSim.setQuadratureRawPosition(distanceToNativeUnits(diffDriveSim.getRightPositionMeters()));
    rightFrontSim.setQuadratureVelocity(velocityToNativeUnits(diffDriveSim.getRightVelocityMetersPerSecond()));

    // Update voltage to talons
    leftFrontSim.setBusVoltage(RobotController.getBatteryVoltage());
    rightFrontSim.setBusVoltage(RobotController.getBatteryVoltage());
  }

  public void drive(double speed, double rotation, boolean squareInputs) {
    diffDrive.arcadeDrive(-speed, rotation, squareInputs);
  }

  public void setMaxOutput(double maxOutput) {
    diffDrive.setMaxOutput(maxOutput);
  }

  public void stop() {
    leftGroup.set(0);
    rightGroup.set(0);
  }

  public Pose2d getPoseMeters() {
    return m_odometry.getPoseMeters();
  }

  public TalonSRX getLeftPIDController() {
    return leftFront;
  }
  public TalonSRX getRightPIDController() {
    return rightFront;
  }

  public double getHeading() {  
    return gyro.getRotation2d().getDegrees();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public void resetEncoders() {
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
  }

  public double getAverageEncoderDistance() {
    return (leftFront.getSelectedSensorPosition() - rightFront.getSelectedSensorPosition()) / 2.0;
  }
  /**
   * Translates the ticks that the encoders provide to meters that we can use.
   * @param ticks
   * @return distance in meters
   */
  public double nativeUnitsToDistanceMeters(double ticks) {
    //return (PhysicalRobotConstants.feetPerTick * ticks) / 3.28;
    double motorRotations = ticks / PhysicalRobotConstants.kFalconCPR;
    double wheelRotations = motorRotations / PhysicalRobotConstants.kGearRatio;
    double positionMeters = wheelRotations * (Math.PI * PhysicalRobotConstants.kWheelDiameterMeters);
    return positionMeters;
  }

   /**
   * Tells us the distance that the motor has traveled
   * @return Meters that the left motor has traveled
   * @see nativeUnitsToDistanceMeters
   * {@link #nativeUnitsToDistanceMeters(double)}
   */
  public double getLeftDistanceMeters() {
    return nativeUnitsToDistanceMeters((double) leftFront.getSelectedSensorPosition());
  }

/**
   * Tells us the distance that the motor has traveled
   * @return Meters that the right motor has traveled
   * @see nativeUnitsToDistanceMeters
   * {@link #nativeUnitsToDistanceMeters(double)}
   */
  public double getRightDistanceMeters() {
    return nativeUnitsToDistanceMeters((double) rightFront.getSelectedSensorPosition());
  }

  /**
   * Translates the encoders native units of ticks per 100ms to meters per second
   * @param ticksPerDecisecond
   * @return Meters per second
   */
  public double nativeUnitsToMetersPerSec(double ticksPerDeciSecond) {
    double ticksPerSec = ticksPerDeciSecond * 10; 
    //return (ticksPerSec * PhysicalRobotConstants.feetPerTick)/ 3.28;

    double motorRotationsPerSec = ticksPerSec / PhysicalRobotConstants.kFalconCPR;
    double wheelRotationsPerSec = motorRotationsPerSec / PhysicalRobotConstants.kGearRatio;
    double metersPerSec = wheelRotationsPerSec *(PhysicalRobotConstants.kWheelDiameterMeters * Math.PI);
    return metersPerSec;
  }

  /**
   * Tells us the velocity of the left motor in meters per second.
   * @return m/s
   * @see
   * {@link #nativeUnitsToMetersPerSec(double)}
   */
  public double getLeftMotorVelocity() {
    //System.out.println("VELOCITY OF LEFT In m/s:::::: " + nativeUnitsToMetersPerSec(leftFront.getSelectedSensorVelocity()));
    return nativeUnitsToMetersPerSec((double)leftFront.getSelectedSensorVelocity());
 
   }
 
   /**
    * Tells us the velocity of the right motor in meters per second/
    * @return m/s
    * @see
    * {@link #nativeUnitsToMetersPerSec(double)}
    */
   public double getRightMotorVelocity() {
   //  System.out.println("VELOCITY OF RIGHT In m/s:::::: " + nativeUnitsToMetersPerSec(rightFront.getSelectedSensorVelocity()));
     return nativeUnitsToMetersPerSec((double)rightFront.getSelectedSensorVelocity());
   }
 
  
   public DifferentialDriveWheelSpeeds getWheelSpeeds() {
     return new DifferentialDriveWheelSpeeds(getLeftMotorVelocity(), getRightMotorVelocity());
   }

   public void tankDriveVolts(double leftVolts, double rightVolts) {
     leftGroup.setVoltage(leftVolts);
     rightGroup.setVoltage(-rightVolts);
     diffDrive.feed();
   }

   private int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(3));
    double motorRotations = wheelRotations * PhysicalRobotConstants.kGearRatio;
    int sensorCounts = (int)(motorRotations * PhysicalRobotConstants.kFalconCPR);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(3));
    double motorRotationsPerSecond = wheelRotationsPerSecond * PhysicalRobotConstants.kGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * PhysicalRobotConstants.kFalconCPR);
    return sensorCountsPer100ms;
  }

 }
