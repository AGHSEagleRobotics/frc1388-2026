package frc.robot.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;

public class VisionAcceptor {
    public static final double robotMargin = 0.5;
    
    ChassisSpeeds m_robotVelocity = new ChassisSpeeds(0, 0, 0);
    int m_jumpCount = 0;
    int m_jumpCountMax = 0;
    double m_angle = 0;
    boolean m_isMegaTag2;


    public VisionAcceptor(boolean isMegaTag2) {
        m_isMegaTag2 = isMegaTag2;
    }

    public boolean shouldAccept(Pose2d currentPosition, Pose2d lastPosition, ChassisSpeeds robotVelocity) {

        if(m_robotVelocity == null || currentPosition == null) {
             System.out.println("null check");
            return false;
        }
        m_robotVelocity = robotVelocity;

        if(currentPosition.getX() == 0 && currentPosition.getY() == 0) {
            return false;
        }

        // if this is the first ever check, then initialize class variable and trust the position
        if(lastPosition == null) {
            lastPosition = currentPosition;
            // System.out.println("first check");
            return true;
        }

        SmartDashboard.putNumber("difference of x", Math.abs(currentPosition.getX() - lastPosition.getX()));
        SmartDashboard.putNumber("difference of y", Math.abs(currentPosition.getY() - lastPosition.getY()));

        double velocityPerTick =  DriveTrainConstants.DISTANCE_PER_TICK;

        double velocityPerTickClamped = MathUtil.clamp(velocityPerTick, 0.05, velocityPerTick);

        double clamedJumpCount = MathUtil.clamp(m_jumpCount, 0, 1000);

        double velocityTimesJumpCount = velocityPerTickClamped * (clamedJumpCount + 1);

        SmartDashboard.putNumber("normalizedVelocity", norm());
        SmartDashboard.putNumber("velocityPerTick", velocityPerTick);
        SmartDashboard.putNumber("velocityPerTickClamped", velocityPerTickClamped);
        SmartDashboard.putNumber("jumpCountVelocity", velocityTimesJumpCount);

        // check if the current position compared to the last position is greater than the velocity per tick of the robot
        
        // if(Math.abs(currentPosition.getX() - m_lastPosition.getX()) > velocityTimesJumpCount
        // || Math.abs(currentPosition.getY() - m_lastPosition.getY()) > velocityTimesJumpCount) {
        //     m_jumpCount++;
        //     if(m_jumpCount > m_jumpCountMax) {
        //         m_jumpCountMax = m_jumpCount;
        //     }
        //     if(m_jumpCount > 4) {
        //         // m_lastPosition = currentPosition;
        //     }
        // }
        // else {
        //     m_jumpCount = 0;
        // }

        if(currentPosition.getTranslation().getDistance(lastPosition.getTranslation()) > DriveTrainConstants.ROBOT_MAX_SPEED * DriveTrainConstants.DT_SECONDS) {
            return false;
        }

        //checks if robot is outside of field
        if(currentPosition.getX() < -robotMargin 
          || currentPosition.getX() > Constants.FieldLayout.FIELD_LENGTH + robotMargin
          || currentPosition.getY() < -robotMargin
          || currentPosition.getY() > Constants.FieldLayout.FIELD_WIDTH + robotMargin) {
            return false;
        }
        if (norm() > 0) {
            double differenceOfPositionX = currentPosition.getX() - lastPosition.getX();
            double differenceOfPositionY = currentPosition.getY() - lastPosition.getY();
            
            Translation2d positionChange = new Translation2d(differenceOfPositionX, differenceOfPositionY);
            Translation2d robotDirection = new Translation2d(m_robotVelocity.vxMetersPerSecond, m_robotVelocity.vyMetersPerSecond);

            Translation2d robotDirectionNormalized = robotDirection.div(robotDirection.getNorm());
            Translation2d positionChangeNormalized = positionChange.div(positionChange.getNorm());

            // dot product of the robot direction and the position change (physics calculation dot product formula)
            double dotProduct = (robotDirectionNormalized.getX() * positionChangeNormalized.getX()) + (robotDirectionNormalized.getY() * positionChangeNormalized.getY());

            m_angle = Math.acos(dotProduct);

            SmartDashboard.putNumber("VisionAcceptor/angleBetweenRobotAndCamera", Math.toDegrees(m_angle));

            double allignmentThreshold = Math.toRadians(15);

            if (m_angle < allignmentThreshold) {
                return false;
            }
        }
        else {
           m_angle = 0;
        }

        // checks if robot is moving too fast for camera to update
        if (norm() > 2.5) { //changed from 4
            return false;
        }
        // m_lastPosition = currentPosition;
        
        return true;      
    }

    public boolean shouldResetGyro() {
        if(norm() == 0.0) {
        return true;
        }
    return false;
    }

    public double norm() {
        double dx = m_robotVelocity.vxMetersPerSecond;
        double dy = m_robotVelocity.vyMetersPerSecond;
        if (dy == 0.0)
            return Math.abs(dx);
        return Math.hypot(dx, dy);
    }
}