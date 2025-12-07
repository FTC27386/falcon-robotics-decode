
package org.firstinspires.ftc.teamcode.Utility;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class RobotConstants {

        public static String first_shooter_motor_name = "flywheel_top",
                second_shooter_motor_name = "flywheel_bottom",
                left_turret_servo_name = "left_turret_servo",
                right_turret_servo_name = "right_turret_servo",
                hood_servo_name = "hood",
                intake_motor_name = "intake",
    intake_servo_name = "intake_pivot_servo",
                left_front_drive_motor_name = "front_left_drive",
                right_front_drive_motor_name = "front_right_drive",
                left_back_drive_motor_name = "back_left_drive",
                right_back_drive_motor_name = "back_right_drive",
                transfer_servo_name = "blocker",
                camera = "Webcam 1",
                turret_encoder_name = "turret_encoder",
    lift_motor_name = "lift_motor",
    left_lift_servo_name = "left_lift_servo",
    right_lift_servo_name = "right_lift_servo";

        public static double
                shooterTolerance = 45,
                vera_coefficient = .4167,

                lift_kP = 0.0,
                lift_kD = 0.0,
                lift_kF = 0.0,
                shooter_kP = 0.02,
                shooter_kD = 0.00000000,
                shooter_kFF = 0,
                shooter_kL = 0.0, //"lower limit" power
                turret_kP = 0.012,
                turret_kD = 0.0,
                turret_kL = 0.025,
                offset_between_servos = 0,
                turret_conversion_factor_DEGREES = (0.0015962441314554),
                turret_conversion_factor_RADIANS = (double)(1/5) * (double)(170/60) * (double)(1/355) * (double)(360/(2*Math.PI)), // for mason's weird ahh
                rpm_conversion_factor = (double)(Math.PI * 2) * (double)(1/60),
                far_flywheel_speed = -310,
                close_flywheel_speed = -250,
        transfer_closed_pos = 0.5,
        transfer_open_pos = .25,
        latch_close_pos = 0, // temp
    latch_open_pos = .25; //temp
        public static Pose autoEndPose;
        public enum robotState
        {
                FAR_SHOOT,
                CLOSE_SHOOT,
                INTAKING,
                IDLE
        }
        public static int top_climb_position = 34000;




}
