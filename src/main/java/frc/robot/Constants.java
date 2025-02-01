package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
     public static final class FieldConstants{

        public static final Translation2d FIELD_DIMENSIONS = new Translation2d(16.541, 8.211);
        public static final Rectangle2d FIELD_AREA = new Rectangle2d(new Translation2d(0, 0), FIELD_DIMENSIONS); //Blue origin
    }




    }

