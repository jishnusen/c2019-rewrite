package com.frc1678.c2019.subsystems;

import com.frc1678.c2019.Constants;
import com.frc1678.c2019.loops.ILooper;
import com.frc1678.c2019.loops.Loop;
import com.frc1678.c2019.states.SuperstructureConstants;
import java.util.List;


/**
 * Choose limelight to use
 * 
 * @see Limelight
 */
public class LimelightManager extends Subsystem {
    private static LimelightManager sInstance = null;
    private Limelight mTopLimelight;
    private Limelight mBottomLimelight;
    private List<Limelight> mAllLimelights;

    enum ActiveLimelight {
        TOP,
        BOTTOM,
    }

    private ActiveLimelight mActiveLimelight = ActiveLimelight.TOP;

    private LimelightManager() {
        mTopLimelight = new Limelight(Constants.kTopLimelightConstants);
        mBottomLimelight = new Limelight(Constants.kBottomLimelightConstants);
        mAllLimelights = List.of(mTopLimelight, mBottomLimelight);
    }

    public static LimelightManager getInstance() {
        if (sInstance == null) {
            sInstance = new LimelightManager();
        }
        return sInstance;
    }


    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        Loop mLoop = new Loop() {
            @Override
            public void onStart(double timestamp) {
                mAllLimelights.forEach(limelight -> limelight.setLed(Limelight.LedMode.OFF));
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (LimelightManager.this) {
                    Limelight limelight;
                    if (mActiveLimelight == ActiveLimelight.TOP) {
                        limelight = mTopLimelight;
//                        System.out.println("Top limelight is active with a target distance of: " + limelight.getTargetDist());
                        //System.out.println("Top limelight is active with a offset of: " + limelight.getXOffset());

                    } else {
                        limelight = mBottomLimelight;
                        //System.out.println("Bottom limelight is active with a target distance of: " + limelight.getTargetDist());
                    }
                }
                setActiveLimelight();
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        };
        mEnabledLooper.register(mLoop);
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mAllLimelights.forEach(limelight -> limelight.readPeriodicInputs());
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mAllLimelights.forEach(limelight -> limelight.writePeriodicOutputs());
    }

    @Override
    public synchronized void stop() {
        mAllLimelights.forEach(limelight -> limelight.stop());
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        mAllLimelights.forEach(limelight -> limelight.outputTelemetry());
    }

    public synchronized ActiveLimelight getActiveLimelight() {
        return mActiveLimelight;
    }

    public synchronized double getXOffset() {
        return getActiveLimelightObject().getXOffset();  
    }


    public synchronized double getTargetDist() {
        return getActiveLimelightObject().getTargetDist();  
    }

    // public synchronized void setUseTopLimelight(boolean useTop) {
    //    mActiveLimelight = useTop ? ActiveLimelight.TOP : ActiveLimelight.BOTTOM;
    //    getInactiveLimelightObject().setLed(Limelight.LedMode.OFF);
    //    getActiveLimelightObject().setLed(Limelight.LedMode.PIPELINE);
   // }

    public synchronized void setActiveLimelight() {
        mActiveLimelight = Elevator.getInstance().getInchesOffGround() < SuperstructureConstants.kSwitchLimelightHeight ? ActiveLimelight.TOP : ActiveLimelight.BOTTOM;
        getInactiveLimelightObject().setLed(Limelight.LedMode.OFF);
        getActiveLimelightObject().setLed(Limelight.LedMode.PIPELINE);
    }

    public synchronized Limelight getActiveLimelightObject() {
        if (mActiveLimelight == ActiveLimelight.TOP) {
            return mTopLimelight;
        } else {
            return mBottomLimelight;
        }
    }

    private synchronized Limelight getInactiveLimelightObject() {
        if (mActiveLimelight == ActiveLimelight.TOP) {
            return mBottomLimelight;
        } else {
            return mTopLimelight;
        }
    }

    public Limelight getTopLimelight() {
        return mTopLimelight;
    }

    public Limelight getBottomLimelight() {
        return mBottomLimelight;
    }

    public synchronized void setAllLeds(Limelight.LedMode mode) {
        mAllLimelights.forEach(limelight -> limelight.setLed(mode));
    }

}
