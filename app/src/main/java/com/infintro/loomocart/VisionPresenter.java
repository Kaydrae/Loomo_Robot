package com.infintro.loomocart;

import android.util.Log;
import android.view.Surface;

import com.segway.robot.algo.dts.BaseControlCommand;
import com.segway.robot.algo.dts.DTSPerson;
import com.segway.robot.algo.dts.PersonTrackingListener;
import com.segway.robot.algo.dts.PersonTrackingProfile;
import com.segway.robot.algo.dts.PersonTrackingWithPlannerListener;
import com.segway.robot.sdk.base.bind.ServiceBinder;
import com.segway.robot.sdk.locomotion.head.Angle;
import com.segway.robot.sdk.locomotion.sbv.Base;
import com.segway.robot.sdk.locomotion.head.Head;
import com.segway.robot.sdk.vision.DTS;
import com.segway.robot.sdk.vision.Vision;
import com.segway.robot.support.control.HeadPIDController;

public class VisionPresenter {
    private final static String TAG = "VisionPresenter";

    private enum States {INIT_TRACK, END_TRACK};

    private final static int TIME_OUT = 10*1000;

    private PresenterChangeInterface mPresInterface;
    private ViewChangeInterface mViewInterface;

    private Vision mVision;
    private Head mHead;
    private Base mBase;
    private HeadPIDController mHeadPID = new HeadPIDController();

    private PersonTrackingProfile mPersonTracking;
    private DTSPerson[] mPersons;

    private boolean isHeadBind;
    private boolean isBaseBind;
    private boolean isVisionBind;

    private boolean noObstacles;

    private long startTime;

    private DTS mDTS;

    private States mState;

    /* Initialize the Vision Presenter */
    public VisionPresenter(ViewChangeInterface _ViewInterface) {
        mViewInterface = _ViewInterface;
    }

    public void startPresenter() {
        mVision = Vision.getInstance();
        mHead = Head.getInstance();
        mBase = Base.getInstance();

        mVision.bindService(LoomoCart.getContext(), mVisionBindStateListener);
        mHead.bindService(LoomoCart.getContext(), mHeadStateListener);
        mBase.bindService(LoomoCart.getContext(), mBaseStateListener);

        mPersonTracking = new PersonTrackingProfile(3, 1.0f);
    }

    /* Stop the vision presenter */
    public void stopPresenter() {
        if (mDTS != null) {
            mDTS.stop();
            mDTS = null;
        }

        mVision.unbindService();
    }

    /* Helper functions */
    public boolean isServicesAvailable() {

        return isVisionBind && isBaseBind && isHeadBind;
    }

    private void resetHead() {
        mHead.setMode(Head.MODE_SMOOTH_TACKING);
        mHead.setWorldYaw(0);
        mHead.setWorldPitch(0.7f);
    }

    public void beginFollow() {
        if (mState == States.INIT_TRACK) {
            return;
        }
        mPersons = mDTS.detectPersons(3*1000*1000);
        startTime = System.currentTimeMillis();
        mState = States.INIT_TRACK;
        mDTS.startPlannerPersonTracking(mPersons[0], mPersonTracking, 60*1000*1000, mTrackingPlanner);
    }

    public void endFollow() {
        if (mState == States.INIT_TRACK) {
            mState = States.END_TRACK;
            mDTS.stopPlannerPersonTracking();
        }
    }

    /* Tracking Listeners */
    private PersonTrackingWithPlannerListener mTrackingPlanner = new PersonTrackingWithPlannerListener() {
        @Override
        public void onPersonTrackingWithPlannerResult(DTSPerson person, BaseControlCommand baseControlCommand) {
            if (person == null) {
                if (System.currentTimeMillis() - startTime > TIME_OUT) {
                    resetHead();
                }

                return;
            }


            startTime = System.currentTimeMillis();
            mHead.setMode(Head.MODE_ORIENTATION_LOCK);
            mHeadPID.updateTarget(person.getTheta(), person.getDrawingRect(), 480);

            AutoFitDrawableView autoFitDrawableView = mViewInterface.getAutoDrawable();
            autoFitDrawableView.drawRect(person.getDrawingRect());

            switch (baseControlCommand.getFollowState()) {
                case BaseControlCommand.State.NORMAL_FOLLOW:
                    mBase.setControlMode(Base.CONTROL_MODE_RAW);
                    mBase.setLinearVelocity(baseControlCommand.getLinearVelocity());
                    mBase.setAngularVelocity(baseControlCommand.getAngularVelocity());
                    break;
                case BaseControlCommand.State.HEAD_FOLLOW_BASE:
                    mBase.setControlMode(Base.CONTROL_MODE_FOLLOW_TARGET);
                    mBase.updateTarget(0, person.getTheta());
                    break;
                case BaseControlCommand.State.SENSOR_ERROR:
                    mBase.setControlMode(Base.CONTROL_MODE_RAW);
                    mBase.setLinearVelocity(0);
                    mBase.setAngularVelocity(0);
                    break;
            }
        }

        @Override
        public void onPersonTrackingWithPlannerError(int errorCode, String message) {
            Log.d("PersonTracking", message);
        }
    };

    /* Service Bind State Listeners */
    private ServiceBinder.BindStateListener mVisionBindStateListener = new ServiceBinder.BindStateListener() {
        @Override
        public void onBind() {
            isVisionBind = true;
            mDTS = mVision.getDTS();
            mDTS.setVideoSource(DTS.VideoSource.CAMERA);
            AutoFitDrawableView autoFitDrawableView = mViewInterface.getAutoDrawable();
            Surface surface = new Surface(autoFitDrawableView.getPreview().getSurfaceTexture());
            mDTS.setPreviewDisplay(surface);
            mDTS.start();
        }

        @Override
        public void onUnbind(String reason) {
            isVisionBind = false;
        }
    };

    private ServiceBinder.BindStateListener mBaseStateListener = new ServiceBinder.BindStateListener() {
        @Override
        public void onBind() {
            isBaseBind = true;
        }

        @Override
        public void onUnbind(String reason) {
            isBaseBind = false;
        }
    };

    private ServiceBinder.BindStateListener mHeadStateListener = new ServiceBinder.BindStateListener() {
        @Override
        public void onBind() {
            isHeadBind = true;
            resetHead();
            mHeadPID.init(HeadControlHandlerImpl);
            mHeadPID.setHeadFollowFactor(1.0f);
        }

        @Override
        public void onUnbind(String reason) {
            isHeadBind = false;
        }
    };

    /* Head PID Controller Handler */
    private HeadPIDController.HeadControlHandler HeadControlHandlerImpl = new HeadPIDController.HeadControlHandler() {
        @Override
        public float getJointYaw() {
            Angle angle = mHead.getHeadJointYaw();
            if (angle == null) {
                return 0;
            }
            return angle.getAngle();
        }

        @Override
        public float getJointPitch() {
            Angle angle = mHead.getHeadJointPitch();
            if (angle == null) {
                return 0;
            }
            return angle.getAngle();
        }

        @Override
        public void setYawAngularVelocity(float velocity) {
            mHead.setYawAngularVelocity(velocity);
        }

        @Override
        public void setPitchAngularVelocity(float velocity) {
            mHead.setPitchAngularVelocity(velocity);
        }
    };
}
