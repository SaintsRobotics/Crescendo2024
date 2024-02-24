// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * <p>
 * In 3D poses and transforms:
 * <ul>
 * <li>+X is north/forward,
 * <li>+Y is west/left,
 * <li>+Z is up.
 * </ul>
 * 
 * <p>
 * On the field (0, 0, 0) in 3D space is the right corner of the blue alliance
 * driver station
 * Therefore, from the blue driver station: +X is forward, +Y is left, +Z is up
 * 
 * <p>
 * In 2D poses and transforms:
 * <ul>
 * <li>+X is away from the driver,
 * <li>+Y is toward the blue alliance driver's left and to the red alliance
 * driver's right
 * <li>+Rotation is clockwise
 * </ul>
 */
public class VisionSubsystem extends SubsystemBase {

  private NetworkTable m_visionNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");

  private final DoubleArraySubscriber m_botPose;

  private final IntegerSubscriber m_tv;

  private List<Consumer<Measurement>> m_consumerList;

  /** Creates a new Limelight. */
  public VisionSubsystem() {
    // Provide the limelight with the camera pose relative to the center of the
    // robot
    m_visionNetworkTable.getEntry("camerapose_robotspace_set").setDoubleArray(VisionConstants.kLimelightCamPose);

    // Create subscribers to get values from the limelight
    m_botPose = m_visionNetworkTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(null);
    m_tv = m_visionNetworkTable.getIntegerTopic("tv").subscribe(0);
  }

  /** Add a consumer, which the vision subsystem will push an update to every time there is an updated measurement */
  public void addConsumer(Consumer<Measurement> consumer) {
    m_consumerList.add(consumer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Measurement latestMeasurement = getMeasurement();

    if (latestMeasurement != null) {
      for (Consumer<Measurement> consumer : m_consumerList) {
        consumer.accept(latestMeasurement);
      }
    }

    SmartDashboard.putBoolean("Limelight Has Target", m_tv.get() == 1);
  }

  public Measurement getMeasurement() {
    TimestampedDoubleArray[] updates = m_botPose.readQueue();

    // If we have had no updates since the last time this method ran then return
    // nothing
    if (updates.length == 0) {
      return null;
    }

    TimestampedDoubleArray update = updates[updates.length - 1];

    // If the latest update is empty or we don't see an april tag then return
    // nothing
    if (Arrays.equals(update.value, new double[6]) || m_tv.get() == 0) {
      return null;
    }

    double x = update.value[0];
    double y = update.value[1];
    double z = update.value[2];
    double roll = Units.degreesToRadians(update.value[3]);
    double pitch = Units.degreesToRadians(update.value[4]);
    double yaw = Units.degreesToRadians(update.value[5]);

    double timestamp = Timer.getFPGATimestamp() - (update.value[6] / 1000.0);
    Pose3d pose = new Pose3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));

    return new Measurement(
        timestamp,
        pose,
        VisionConstants.kVisionSTDDevs);
  }

  public static class Measurement {
    public double timestamp;
    public Pose3d pose;
    public Matrix<N3, N1> stdDeviation;

    public Measurement(double timestamp, Pose3d pose, Matrix<N3, N1> stdDeviation) {
      this.timestamp = timestamp;
      this.pose = pose;
      this.stdDeviation = stdDeviation;
    }
  }
}
