package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.teamcode.Misc.UtilMethods.squareMagnitude;
import static org.firstinspires.ftc.teamcode.Misc.UtilMethods.squareRootMagnitude;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Misc.RobotConstants;
import org.firstinspires.ftc.teamcode.Misc.UtilMethods;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;
@Configurable
public class Drive implements Subsystem {


    private static boolean sillyDriveMode;
    private static boolean driveLocked;
    private static boolean headingLocked;
    private static GoBildaPinpointDriver pinpoint;
    private static DcMotorEx lf;
    private static DcMotorEx rf;
    private static DcMotorEx lb;
    private static DcMotorEx rb;
    private static PIDController headingControl;
    public static double kYaw = 1.3;

    @Retention(RetentionPolicy.RUNTIME) //Call @Lift.Attach
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach {
    }

    private Dependency<?> dependency =
            Subsystem.DEFAULT_DEPENDENCY
                    .and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    private Drive() {}

    public static final Drive INSTANCE = new Drive();
    public void preUserInitHook(@NonNull Wrapper opMode) {
        headingControl = new PIDController(RobotConstants.lockP, RobotConstants.lockI, RobotConstants.lockD);
        HardwareMap hwmap = opMode.getOpMode().hardwareMap;
        pinpoint = hwmap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(156, -96, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        lb = hwmap.get(DcMotorEx.class, "left_back");
        lf = hwmap.get(DcMotorEx.class, "left_front");
        rf = hwmap.get(DcMotorEx.class, "right_front");
        rb = hwmap.get(DcMotorEx.class, "right_back");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        sillyDriveMode = false;
        driveLocked = true;
        headingLocked = false;
        setDefaultCommand(updateDrive());
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

    public static void update() {
        pinpoint.update();
        BoundGamepad gamepad1 = Mercurial.gamepad1();
        double y = gamepad1.leftStickY().state();
        double x = gamepad1.leftStickX().state();
        double rx = gamepad1.rightStickX().state();
        double targHeading = 0;


        if (gamepad1.options().onTrue()) {
            pinpoint.resetPosAndIMU();
        }

        double botHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        if (gamepad1.rightStickButton().onTrue())
        {
            targHeading = botHeading;
        }
        if(gamepad1.dpadLeft().onTrue())
        {sillyDriveMode = !sillyDriveMode;} //CHANGE ME TO TEST FUNNY DRIVE



        if (!driveLocked) {
            if (headingLocked) {
                double angleDiffLock = UtilMethods.AngleDifference(Math.toDegrees(botHeading), Math.toDegrees(targHeading));
                double headingLock = headingControl.calculate(angleDiffLock, 0);
                headingLock = squareRootMagnitude(headingLock);
                setDrivePowers(-y, x, headingLock, botHeading);
            } else if (sillyDriveMode) {
                double stickHeading = Math.atan2(-(Math.abs(y)), -Math.abs(x)) + Math.PI / 2;
                double angleDiffStick;
                if (y == 0 && x == 0) {
                    angleDiffStick = 0;
                } else {
                    angleDiffStick = UtilMethods.AngleDifference(Math.toDegrees(botHeading), Math.toDegrees(stickHeading));
                }
                double tangentLock = headingControl.calculate(angleDiffStick, 0); //Locking the heading of the bot to its path tangent. Will this work, Kobe Bryant?
                tangentLock = squareRootMagnitude(tangentLock);
                setDrivePowers(-y, x, tangentLock, botHeading);
            } else {
                setDrivePowers(-y, x, rx, botHeading);
            }
        }
        else
        {
            setDrivePowers(0, 0, 0, botHeading);
        }
    }

    public static void setDrivePowers(double axial, double lateral, double yaw, double heading)
    {
        double rotX = (squareMagnitude(lateral)) * Math.cos(-heading) + (squareMagnitude((axial)) * Math.sin(-heading));
        double rotY = (squareMagnitude(lateral)) * Math.sin(-heading) - (squareMagnitude(axial) * Math.cos(-heading));
        double adjYaw = yaw  * kYaw;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(adjYaw), 1);
        double frontLeftPower = (rotY + rotX + adjYaw) / denominator;
        double backLeftPower = (rotY - rotX + adjYaw) / denominator;
        double frontRightPower = (rotY - rotX - adjYaw) / denominator;
        double backRightPower = (rotY + rotX - adjYaw) / denominator;

        lf.setPower(frontLeftPower);
        lb.setPower(backLeftPower);
        rf.setPower(frontRightPower);
        rb.setPower(backRightPower);
    }
    public static Lambda updateDrive()
    {
        return new Lambda("drive")
                .setExecute(Drive::update)
                .setInterruptible(() -> true)
                .setFinish(() -> false);
    }
    public static Lambda toggleDriveLock()
    {
        return new Lambda("unlock-drive")
                .setExecute(() -> driveLocked = !driveLocked)
                .setInterruptible(() -> false)
                .setFinish(() -> true);
    }

    public static Lambda setYawMulti(double toSet)
    {
        return  new Lambda("yaw-multiplier")
                .setExecute(() -> kYaw = toSet)
                .setFinish(()->true);
    }

}