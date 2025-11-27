
package org.firstinspires.ftc.teamcode.Misc;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Configurable
public class RobotConstants {

        public static String first_shooter_motor_name = "flywheel_top",
                second_shooter_motor_name = "flywheel_bottom",
                left_turret_servo_name = "left_turret_servo",
                right_turret_servo_name = "right_turret_servo",
                hood_servo_name = "hood",
                intake_motor_name = "intake",
                left_front_drive_motor_name = "front_left_drive",
                right_front_drive_motor_name = "front_right_drive",
                left_back_drive_motor_name = "back_left_drive",
                right_back_drive_motor_name = "back_right_drive",
                transfer_servo_name = "blocker",
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
                rpm_conversion_factor = (double)(Math.PI * 2) * (double)(1/60),
                far_flywheel_speed = (double)(5750),
                close_flywheel_speed = (double)(2500),
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
