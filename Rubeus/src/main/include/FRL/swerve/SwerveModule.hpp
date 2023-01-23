/* By Luke White and Tyler Clarke
    Custom swerve module library using PIDController.
*/
// Note: This and PIDController.hpp need some more cleanup work.

#pragma once

#include <FRL/motor/BaseMotor.hpp>
#include <ctre/Phoenix.h>
#include <iostream>
#include <FRL/motor/PIDController.hpp>

/**
 @author Luke White and Tyler Clarke
 @version 1.0
 
 * Swerve module for FRC. Manages 2 BaseMotor pointers (which because of the polymorphism can be any motor type in FRC)
 */
class SwerveModule {
    /**
     * Motor that controls the rotation of the wheel
     */
    BaseMotor* speed;
    /**
     * Motor that controls the direction of the wheel
     */
    BaseMotor* direction;
    /**
     * PIDController that manages the direction motor
     */
    PIDController* directionController;
    /**
     * CANCoder to use for PID; heap allocated by an ID provided on construction.
     */
    CANCoder* cancoder;

    /**
     * Current percentage that will be applied to the wheel
     */
    double curPercent; // So multiple commands can alter speed
       
    /**
     * SwerveModules are a linked list! This means you can have any number of 'em configured with separate offsets and command them all at once with a single call.
     */
    SwerveModule* linkSwerve;

    /**
     * Whether or not the SwerveModule is linked to another one.
     */
    bool isLinked = false;  

    /**
     * Configured offset.
     */
    double encoderOffset;
    
public:
    short swerveRole;
    bool readyToOrient = false;
    /**
     * Constructor
     @param speedMotor The motor to use for wheel speed control
     @param directionMotor The motor to use for wheel direction control
     @param CanCoderID The CAN id of the CANCoder
     @param offset The offset of the wheel, in encoder ticks
     @param speedInverted Whether or not to invert the wheel speed motor
     @param direcInverted Whether or not to invert the wheel direction motor
     */
    SwerveModule(BaseMotor* speedMotor, BaseMotor* directionMotor, int CanCoderID, short role, double offset, bool speedInverted=false, bool direcInverted=false) {
        encoderOffset = offset;
        speed = speedMotor;
        direction = directionMotor;
        cancoder = new CANCoder {CanCoderID};
        
        swerveRole = role;
        directionController = new PIDController (direction);
        directionController -> constants.P = 0.0005;
        //directionController -> constants.I = 0.0001;
        directionController -> constants.MaxOutput = 0.2;
        directionController -> constants.MinOutput = -0.2;
        directionController -> SetCircumference(4096);
        
        speed -> SetInverted(speedInverted);
        direction -> SetInverted(direcInverted);
    }
    
    /**
     * Link to another swerve module
     @param LinkSwerve The swerve module to link to
     */
    void Link(SwerveModule* LinkSwerve) {
        isLinked = true;           
        linkSwerve = LinkSwerve; 
    }

    double coterminal(double angle) {
        while (angle >= 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        return angle;
    }
    
        /**
     * Get the current (physical) direction of the module
     */
    long GetDirection() {
        return smartLoop(cancoder -> GetAbsolutePosition() - encoderOffset);
    }

    bool withinDeadband(double num, double dead, double reference = 0) {
        return reference + num <= reference + dead && reference - num >= reference - dead;
    }

    /**
     * Set the direction of the motor.
     @param targetPos The encoder tick to aim for
     @param followLink Whether or not to follow its link
     @param followRole What role to follow
     */
    
    void SetDirection(double targetPos, bool followLink = true) {
        directionController -> SetPosition(targetPos);
        directionController -> Update(GetDirection());

        if (isLinked && followLink){
            linkSwerve -> SetDirection(targetPos);
        }
    }

    /**
     Increase the speed of the wheel motor
     @param spd Percentage to add
     @param followLink Whether or not to command the linked swerve module, if it exists
     */
    void MovePercent(double spd, bool followLink = true){
        curPercent += spd;
        if (isLinked && followLink){
            linkSwerve -> MovePercent(spd);
        }
    }

    /**
     Apply a percentage to the wheel motor
     */
    void ApplySpeed(){
        speed -> SetPercent(curPercent);
        curPercent = 0; // Velocity ain't "sticky", this is a safety thing
        if (isLinked){
            linkSwerve -> ApplySpeed();
        }
    } 

    /**
     * Orient the swerve drive 
     */
    
    bool allReadyToOrient() {
        if (swerveRole == 4) {
            return ((readyToOrient) && (linkSwerve -> readyToOrient) && (linkSwerve -> linkSwerve -> readyToOrient) && (linkSwerve -> linkSwerve -> linkSwerve -> readyToOrient));
        }
        return false;
    }

    bool Orient(int angle, int currentAngle) {
        if (angle == -1) {        // If the POV is not being currently pressed
            return true;
        }

        else {
            if ((swerveRole == 1 || swerveRole == 3)) {          // If top-left or botton-right
                SetDirection((4096/360) * 45, false);          // Go at 45 degrees
                if (withinDeadband(GetDirection(), 3, (4096/360) * 45)) {         // If there
                    readyToOrient = true;                    // Ready to orient; when all of them are ready, the speed will set
                }
            }   

            else {
                SetDirection((4096/360) * 315, false);
                if (withinDeadband(GetDirection(), 3, (4096/360) * 315)) {
                    readyToOrient = true;
                }
            }

            if (allReadyToOrient()) {
                
            }

            if (isLinked) {
                bool _voidBool = linkSwerve -> Orient(angle, currentAngle);
            }
        }
    }
};