package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Utility.RobotConstants;
import org.firstinspires.ftc.teamcode.Utility.UtilMethods;

public class shooterSystem extends SubsystemBase
{
    boolean isWoundUp = false;
    AnalogInput turretEnc;
    PIDController headingControl,speedControl;
    Servo turret1;
    Servo turret2;
    Servo hood;
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    double rawCalcPower,
    axonRead = 0,
    degreeRead,
    deltaRead,
    previousread,
    rotations,
    degreestravelled,
    turretDeg,
    error,
    signal,
    powerToSet,
    turretPosition = 0,
    hoodPosition = 0,
    currentSpeed,
            nominalVoltage = 12.00; //voltage at which the shooter was tuned
    public shooterSystem(final HardwareMap hMap)
    {
        turretEnc = hMap.get(AnalogInput.class, RobotConstants.turret_encoder_name);
        shooter1 = hMap.get(DcMotorEx.class, RobotConstants.first_shooter_motor_name);
        shooter2 = hMap.get(DcMotorEx.class, RobotConstants.second_shooter_motor_name);
        shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setDirection(DcMotorEx.Direction.FORWARD);
        turret1 = hMap.get(Servo.class, RobotConstants.left_turret_servo_name);
        turret2 = hMap.get(Servo.class, RobotConstants.right_turret_servo_name);
        hood = hMap.get(Servo.class, RobotConstants.hood_servo_name);
        speedControl = new PIDController(RobotConstants.shooter_kP, 0, RobotConstants.shooter_kD);
        headingControl = new PIDController(RobotConstants.turret_kP, 0, RobotConstants.turret_kD);
    }
    public PIDController getSpeedControl()
    {
        return speedControl;
    }
    public double getFlywheelSignal()
    {
        return powerToSet;
    }
    public double getCurrentSpeed()
    {
        return currentSpeed;
    }
    @Override
    public void periodic()
    {
        // Get voltage and compare, hopefully should filter out drops.
        speedControl.setTolerance(RobotConstants.shooterTolerance);
        speedControl.setPID(RobotConstants.shooter_kP, 0, RobotConstants.shooter_kD);
        headingControl.setPID(RobotConstants.turret_kP, 0, RobotConstants.turret_kD);

        currentSpeed = shooter1.getVelocity();
        rawCalcPower = speedControl.calculate(-currentSpeed);
        powerToSet = rawCalcPower + (RobotConstants.shooter_kL) + RobotConstants.shooter_kFF;


        shooter1.setPower(powerToSet);
        shooter2.setPower(powerToSet);
        turret1.setPosition(turretPosition);
        turret2.setPosition(turretPosition);
        hood.setPosition(hoodPosition);
        previousread = degreeRead;
    }
    public void setSpeed(double speed)
    {
        speedControl.setSetPoint(speed);
    }
    public boolean atSpeed()
    {
        return speedControl.atSetPoint();
    }
    public void setTurretPosition(double turretPosition)
    {
        this.turretPosition = turretPosition;
    }
    public void setHoodPosition(double hoodPosition)
    {
        this.hoodPosition = hoodPosition;
    }

}