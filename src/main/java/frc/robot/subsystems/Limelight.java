// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {

  private static NetworkTableInstance table;
  private final java.util.logging.Logger logger;

  public Limelight() {
    logger = java.util.logging.Logger.getLogger(Limelight.class.getName());
    table = null;
    logger.info("Limelight initialized");
  }


  /**
	 * Helper method to get an entry from the Limelight NetworkTable.
	 * 
	 * @param key
	 *            Key for entry.
	 * @return NetworkTableEntry of given entry.
	 */
  private static NetworkTableEntry getValue(String key) {
		if (table == null) {
			table = NetworkTableInstance.getDefault();
		}

		return table.getTable("limelight").getEntry(key);
  }

  /**
   * @return the distance in the X direction to the tag
   */
  public double getX() {
    return getValue("tx").getDouble(0);
  }

  /**
   * @return the distance in the Y direction to the tag
   */
  public double getY() {
    return getValue("ty").getDouble(0);
  }

  /**
   * @return The Percentage of area of the tag that is seen (100% means 100% visable)
   */
  public double getArea() {
    return getValue("ta").getDouble(0);
  }

  /**
   * @return If Camera is detecting a tag
   */
  public boolean detectTag() {
    return getValue("tv").getDouble(0) == 1;
  }


  @Override
  public void periodic() {
    Logger.recordOutput("LimeLight/DistanceToTarget(x)",table.getEntry("tx").getDouble(0.0));
    Logger.recordOutput("LimeLight/DistanceToTarget(y)",table.getEntry("ty").getDouble(0.0));
    Logger.recordOutput("LimeLight/TargetAreaVisable",table.getEntry("ta").getDouble(0.0));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
