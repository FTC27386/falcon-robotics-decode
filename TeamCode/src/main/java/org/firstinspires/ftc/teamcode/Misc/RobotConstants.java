
package org.firstinspires.ftc.teamcode.Misc;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Configurable
public class RobotConstants {

        public static String first_shooter_motor_name = "flywheel1",
                second_shooter_motor_name = "flywheel2",
                left_turret_servo_name = "turretServo1",
                right_turret_servo_name = "turretServo2",
                hood_servo_name = "hood",
                intake_motor_name = "intake",
                left_front_drive_motor_name = "left_front_drive",
                right_front_drive_motor_name = "right_front_drive",
                left_back_drive_motor_name = "left_back_drive",
                right_back_drive_motor_name = "right_back_drive",
                transfer_servo_name = "gate",
                camera = "Webcam 1",
                turret_encoder_name = "turret_encoder";

        public static double
                shooterTolerance = 0.05,
                vera_coefficient = .4167,
                shooter_kP = 0.0,
                shooter_kD = 0.0,
                shooter_kFF = 0.0,
                shooter_kL = 0.0, //"lower limit" power
                turret_kP = 0.0,
                turret_kD = 0.0,
                turret_kL = 0.0,
                offset_between_servos = 0,
                turret_conversion_factor_DEGREES = (double)(1/5) * (double)(170/60) * (double)(1/355),
                turret_conversion_factor_RADIANS = (double)(1/5) * (double)(170/60) * (double)(1/355) * (double)(360/(2*Math.PI)), // for mason's weird ahh
        transfer_closed_pos = 0.5, // temp
        transfer_open_pos = .25; // temp
        public static Pose autoEndPose;
        public enum robotState
        {
                FAR_SHOOT,
                CLOSE_SHOOT,
                INTAKING,
                IDLE
        }



}
