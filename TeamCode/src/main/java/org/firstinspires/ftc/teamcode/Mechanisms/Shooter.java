package org.firstinspires.ftc.teamcode.Mechanisms;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.Misc.cachedMotor;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

@Configurable
//Shooter with a selector
public class Shooter implements Subsystem {


    private static Servo hood;
    private static cachedMotor flywheel;
    private static double hoodPosition;
    private static double hoodPositionReal; //If we use an axon w/ the encoder
    private static double currentSpeed;
    private static PIDFCoefficients velocityControlParams;
    private static PIDFController velocityControl;
    private static double targetVelocity;
    private static double currentVelocity;

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach {
    }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Shooter.Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

    private Shooter() {
    }

    public static final Shooter INSTANCE = new Shooter();

    public void preUserInitHook(@NonNull Wrapper opMode) { // THIS CODE RUNS ONCE WHEN YOU HIT THE INIT BUTTON.
        velocityControlParams = new PIDFCoefficients(0, 0, 0, 0);
        velocityControl = new PIDFController(velocityControlParams);
        HardwareMap hwMap = opMode.getOpMode().hardwareMap;
        flywheel = new cachedMotor(hwMap.get(DcMotorEx.class, "flywheel"), 0);
        hood = hwMap.get(Servo.class, "hood");
        setDefaultCommand(updateFlywheel());
    }

    public static void update() //BEHAVIOR EVERY LOOP AFTER "START" (not init)
    {
        //How to get target velocity? Will try with odometry and april tags
        //Same with hood position
        currentVelocity = flywheel.thisMotor.getVelocity();
        velocityControl.setTargetPosition(targetVelocity);
        velocityControl.updatePosition(currentVelocity);
        flywheel.setPower(velocityControl.runPIDF());
        hood.setPosition(hoodPosition);
    }

    public static void setTargetVelocity(double target) { //hopefully unused or for weakly ejecting balls
        targetVelocity = target;
    }

    public static Lambda updateFlywheel() {
        return new Lambda("flywheel")
                .setExecute(Shooter::update)
                .setInterruptible(() -> true)
                .setFinish(() -> false);
    }


}
