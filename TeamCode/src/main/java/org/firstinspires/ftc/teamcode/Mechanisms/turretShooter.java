package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.teamcode.Misc.RobotConstants;
import org.firstinspires.ftc.teamcode.Misc.cachedMotor;

public class turretShooter
{
    AnalogInput turretEnc;
    PIDController headingControl,speedControl;
    CRServo turret1;
    CRServo turret2;
    Servo hood;
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    double rawCalcPower,
    axonRead,
    degreeRead,
    deltaRead,
    previousread,
    rotations,
    degreestravelled,
    turretDeg,
    error,
    odo_turretservo_angle,
    signal;
    double powerToSet;
    double turretPosition = 0;
    double hoodPosition = 0;
    double currentSpeed;
    double nominalVoltage = 12.00; //voltage at which the shooter was tuned
    public turretShooter(final HardwareMap hMap)
    {
        turretEnc = hMap.get(AnalogInput.class, RobotConstants.turret_encoder_name);
        shooter1 = hMap.get(DcMotorEx.class, RobotConstants.first_shooter_motor_name);
        shooter2 = hMap.get(DcMotorEx.class, RobotConstants.second_shooter_motor_name);
        turret1 = hMap.get(CRServo.class, RobotConstants.left_turret_servo_name);
        turret2 = hMap.get(CRServo.class, RobotConstants.right_turret_servo_name);
        hood = hMap.get(Servo.class, RobotConstants.hood_servo_name);
        speedControl = new PIDController(RobotConstants.shooter_kP, 0, RobotConstants.shooter_kD);
        headingControl = new PIDController(RobotConstants.turret_kP, 0, RobotConstants.turret_kD);
    }
    public void update()
    {
        speedControl.setPID(RobotConstants.shooter_kP, 0, RobotConstants.shooter_kD);
        headingControl.setPID(RobotConstants.turret_kP, 0, RobotConstants.turret_kD);
        axonRead = turretEnc.getVoltage();
        degreeRead = axonRead * (360/3.3);
        deltaRead = degreeRead - previousread;
        if (deltaRead > 180)
        {
            rotations -= 1;
        }
        if (deltaRead < -180)
        {
            rotations += 1;
        }
        degreestravelled = rotations*360.0 + degreeRead;
        turretDeg = degreestravelled*(5.0)*(60/170.0);
        error = UtilMethods.AngleDifference(odo_turretservo_angle, turretDeg) ;
        signal = headingControl.calculate(-error, 0);
        signal += Math.signum(signal) * RobotConstants.turret_kL;
        currentSpeed = shooter1.getVelocity();
        rawCalcPower = speedControl.calculate(currentSpeed);
        powerToSet = rawCalcPower + (Math.signum(rawCalcPower) * RobotConstants.shooter_kL) + RobotConstants.shooter_kFF;

        shooter1.setPower(powerToSet);
        shooter2.setPower(powerToSet);
        turret1.setPower(signal);
        turret2.setPower(signal);
        hood.setPosition(hoodPosition);
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
    public void primeShooter(double speed, double turretPosition, double hoodPosition)
    {
        setSpeed(speed);
        setTurretPosition(turretPosition);
        setHoodPosition(hoodPosition);
    }

}