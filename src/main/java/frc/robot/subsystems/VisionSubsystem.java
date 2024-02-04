// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
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

  NetworkTable m_visionNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");

  private final DoubleArraySubscriber m_botPose;
  private final DoubleSubscriber m_cl;
  private final DoubleSubscriber m_tl;

  /** Creates a new Limelight. */
  public VisionSubsystem() {
    // Provide the limelight with the camera pose relative to the center of the
    // robot
    m_visionNetworkTable.getEntry("camerapose_robotspace_set").setDoubleArray(VisionConstants.kLimelightCamPose);

    // Create subscribers to get values from the limelight
    m_botPose = m_visionNetworkTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(null);
    m_cl = m_visionNetworkTable.getDoubleTopic("cl").subscribe(0);
    m_tl = m_visionNetworkTable.getDoubleTopic("tl").subscribe(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Optional<Measurement> getMeasurement() {
    TimestampedDoubleArray[] updates = m_botPose.readQueue();

    // If we have had no updates since the last time this method ran then return
    // nothing
    if (updates.length == 0) {
      return Optional.empty();
    }

    TimestampedDoubleArray update = updates[updates.length - 1];

    // If the latest update is empty then return nothing
    if (Arrays.equals(update.value, new double[6])) {
      return Optional.empty();
    }

    double x = update.value[0];
    double y = update.value[1];
    double z = update.value[2];
    double roll = Units.degreesToRadians(update.value[3]);
    double pitch = Units.degreesToRadians(update.value[4]);
    double yaw = Units.degreesToRadians(update.value[5]);

    double latency = m_cl.get() + m_tl.get();

    double timestamp = (update.timestamp * 1e-6) - (latency * 1e-3);
    Pose3d pose = new Pose3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));

    /*
     * The limelight returns 3D field poses where (0, 0, 0) is located at the center
     * of the field
     * 
     * So to input this pose into our pose estimator we need to tranform so that (0,
     * 0, 0) is the right corner of the blue driver stations
     */
    // TODO: Check if we actually need to do this...
    // pose.transformBy(new Transform3d(new Translation3d(VisionConstants.kFieldLength, VisionConstants.kFieldWidth, 0.0), new Rotation3d()));

    return Optional.of(new Measurement(
        timestamp,
        pose,
        VisionConstants.kVisionSTDDevs));
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
