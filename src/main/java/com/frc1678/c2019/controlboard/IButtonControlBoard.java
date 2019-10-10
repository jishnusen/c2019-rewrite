package com.frc1678.c2019.controlboard;

public interface IButtonControlBoard {
    // Elevator-Wrist combos
    boolean goToGround();

    boolean goToStow();

    boolean goToFirstLevel();

    boolean goToSecondLevel();

    boolean goToThirdLevel();
    
    boolean goToFirstLevelBackwards();

    boolean goToShip();

    // Elevator
    double getJogElevatorThrottle();

    // Wrist
    double getJogWristThrottle();

    // Cargo Intake
    boolean getRunIntake();
    
    boolean getRunOuttake();

    // Hatch Intake
    boolean getScoreHatch();

    void setRumble(boolean on);

    // Climbing
    boolean climbMode();
    boolean exitClimbMode();
    boolean dropCrawlers();
    boolean Crawl();
    // boolean dropForks();
}
