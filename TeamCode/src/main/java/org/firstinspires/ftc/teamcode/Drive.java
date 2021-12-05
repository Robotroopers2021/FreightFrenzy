package org.firstinspires.ftc.teamcode;

public interface Drive {
    enum State {
        STOPPED,
        MOVING
    }

    void move( double power );

    void turn( double power );

    void stop( );

    void drive( double move, double turn );

    int convertDistTicks( double distanceToMove, double circumference );

    State getState( );
}
