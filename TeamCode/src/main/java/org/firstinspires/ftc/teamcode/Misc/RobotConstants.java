
package org.firstinspires.ftc.teamcode.Misc;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConstants {

        public static String first_shooter_motor_name = "flywheel1";
        public static String second_shooter_motor_name = "flywheel2";
        public static String left_turret_servo_name = "turretServo1";
        public static String right_turret_servo_name = "turretServo2";
        public static String hood_servo_name = "hood";
        public static String intake_motor_name = "intake";
        public static String left_front_drive_motor_name = "left_front_drive";
        public static String right_front_drive_motor_name = "right_front_drive";
        public static String left_back_drive_motor_name = "left_back_drive";
        public static String right_back_drive_motor_name = "right_back_drive";
        public static String transfer_servo_name = "gate";

        public static double shooterTolerance = 0.05;
        public static double shooter_kP = 0.0;
        public static double shooter_kD = 0.0;
        public static double shooter_kFF = 0.0;
        public static double shooter_kL = 0.0; //"lower limit" power
        public static double offset_between_servos = 0;
        public static double turret_conversion_factor_DEGREES = (double)(1/5) * (double)(170/60) * (double)(1/355);
        public static double turret_conversion_factor_RADIANS = (double)(1/5) * (double)(170/60) * (double)(1/355) * (double)(360/(2*Math.PI)); // for mason's weird ahh
        public static double transfer_closed_pos = 0; // temp
        public static double transfer_open_pos = 1; // temp

}
