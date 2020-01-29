package com.infintro.loomocart;

import android.speech.tts.Voice;
import android.util.Log;
import android.view.Surface;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import com.segway.robot.algo.Pose2D;
import com.segway.robot.algo.PoseVLS;
import com.segway.robot.algo.VLSPoseListener;
import com.segway.robot.algo.dts.BaseControlCommand;
import com.segway.robot.algo.dts.DTSPerson;
import com.segway.robot.algo.dts.PersonTrackingListener;
import com.segway.robot.algo.dts.PersonTrackingProfile;
import com.segway.robot.algo.dts.PersonTrackingWithPlannerListener;
import com.segway.robot.algo.minicontroller.CheckPoint;
import com.segway.robot.algo.minicontroller.CheckPointStateListener;
import com.segway.robot.sdk.base.bind.ServiceBinder;
import com.segway.robot.sdk.locomotion.head.Angle;
import com.segway.robot.sdk.locomotion.sbv.Base;
import com.segway.robot.sdk.locomotion.head.Head;
import com.segway.robot.sdk.locomotion.sbv.StartVLSListener;
import com.segway.robot.sdk.vision.DTS;
import com.segway.robot.sdk.vision.Vision;
import com.segway.robot.sdk.voice.Speaker;
import com.segway.robot.sdk.voice.Recognizer;
import com.segway.robot.sdk.voice.VoiceException;
import com.segway.robot.sdk.voice.recognition.RecognitionResult;
import com.segway.robot.sdk.voice.recognition.WakeupListener;
import com.segway.robot.sdk.voice.recognition.WakeupResult;
import com.segway.robot.support.control.HeadPIDController;
import com.segway.robot.sdk.voice.recognition.RecognitionListener;
import com.segway.robot.sdk.voice.tts.TtsListener;
import com.segway.robot.sdk.voice.grammar.GrammarConstraint;
import com.segway.robot.sdk.voice.grammar.Slot;
import com.segway.robot.sdk.voice.tts.TtsListener;


public class VisionPresenter {
    private final static String TAG = "VisionPresenter";

    private enum States {INIT_TRACK, END_TRACK, INIT_NAV, END_NAV}

    private final static int TIME_OUT = 10*1000;

    private PresenterChangeInterface mPresInterface;
    private ViewChangeInterface mViewInterface;

    private Vision mVision;
    private Head mHead;
    private Base mBase;
    private HeadPIDController mHeadPID = new HeadPIDController();
    private Recognizer mRecognizer;
    private Speaker mSpeaker;

    private PersonTrackingProfile mPersonTracking;
    private DTSPerson[] mPersons;

    private boolean isHeadBind;
    private boolean isBaseBind;
    private boolean isVisionBind;
    //private boolean isRecognizerBind;

    private boolean noObstacles;

    private long startTime;

    private DTS mDTS;

    private States mState;

    //audio specific variables
    private ServiceBinder.BindStateListener mRecognitionBindStateListener;
    private ServiceBinder.BindStateListener mSpeakerBindStateListener;
    private boolean bindRecognitionService = false;
    private boolean bindSpeakerService = false;
    private TtsListener mTtsListener;

    //path variables
    private Float[][][] paths = {
            {{2.44f, 0f, 0f}},
            {{7.32f, 0f, 0f}},
            {{12.19f, 0f, 0f}},
            {{0.91f, 0f, (float) (Math.PI/2)}, {0.91f, 9.14f, 0f}}
    };

    private List<Float[]> homePath;

    private enum PATH{BRD1, BRD2, BRD3, LOBBY, HOME}
    private PATH mPath;



    /* Initialize the Vision Presenter */
    public VisionPresenter(ViewChangeInterface _ViewInterface) {
        mViewInterface = _ViewInterface;
        homePath = new ArrayList<Float[]>();
        homePath.add(new Float[]{0.0f, 0.0f, (float) (Math.PI)});
    }

    public void startPresenter() {
        mVision = Vision.getInstance();
        mHead = Head.getInstance();
        mBase = Base.getInstance();
        mRecognizer = Recognizer.getInstance();
        mSpeaker = Speaker.getInstance();

        setBindRecognitionService();

        mVision.bindService(LoomoCart.getContext(), mVisionBindStateListener);
        mHead.bindService(LoomoCart.getContext(), mHeadStateListener);
        mBase.bindService(LoomoCart.getContext(), mBaseStateListener);
        mRecognizer.bindService(LoomoCart.getContext(), mRecognitionBindStateListener);
        mSpeaker.bindService(LoomoCart.getContext(), mSpeakerBindStateListener);

        mPersonTracking = new PersonTrackingProfile(3, 1.0f);
    }

    /* Stop the vision presenter */
    public void stopPresenter() {
        if (mDTS != null) {
            mDTS.stop();
            mDTS = null;
        }

        mVision.unbindService();
        mHead.unbindService();
        mBase.unbindService();
        mRecognizer.unbindService();
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
        if (mState == States.INIT_TRACK) return;
        Log.d(TAG, "Beginning follow...");
        speak("I am following!", 100);
        mPersons = mDTS.detectPersons(3*1000*1000);
        startTime = System.currentTimeMillis();
        mState = States.INIT_TRACK;
        mDTS.startPlannerPersonTracking(mPersons[0], mPersonTracking, 60*1000*1000, mTrackingPlanner);
    }

    public void endFollow() {
        if (mState == States.INIT_TRACK) {
            Log.d(TAG, "Ending follow...");
            speak("I am not following anymore.", 100);
            mState = States.END_TRACK;
            mDTS.stopPlannerPersonTracking();
            mHeadPID.stop();
            mBase.clearCheckPointsAndStop();
            resetHead();
            Log.d(TAG, "Follow stopped.");
        }
    }

    public void beginNav() {
        if (mState == States.INIT_NAV) return;
        Log.d(TAG, "Begging nav...");
        speak("I am on my way.", 100);

        mBase.setControlMode(Base.CONTROL_MODE_NAVIGATION);

        Log.d(TAG, "Control Mode: " + mBase.getControlMode());

        mBase.startVLS(true, true, mStartVLS);

        while (!(mBase.isVLSStarted()));

        mBase.setOnCheckPointArrivedListener(mCheckPointListener);

        mBase.cleanOriginalPoint();
        Pose2D pose = mBase.getOdometryPose(-1);
        mBase.setOriginalPoint(pose);

        Log.d(TAG, "Original Checkpoint: " + pose);

        mPath = PATH.BRD1;
        int i = 0;
        for (Float[] checkpoint : paths[mPath.ordinal()]) {
            mBase.addCheckPoint(checkpoint[0], checkpoint[1], checkpoint[2]);

            if (mPath != PATH.HOME) {
                homePath.add((++i), checkpoint);
            }
        }
        Log.d(TAG, "Added checkpoints...");

        mState = States.INIT_NAV;
    }

    public void endNav() {
        if (mState == States.INIT_NAV) {
            Log.d(TAG, "Ending nav...");
            mBase.clearCheckPointsAndStop();
            mBase.stopVLS();
            mBase.setControlMode(Base.CONTROL_MODE_RAW);
            mBase.stop();
            mState = States.END_NAV;
            Log.d(TAG, "Nav stopped.");
            speak("I have arrived.", 100);
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

    private StartVLSListener mStartVLS = new StartVLSListener() {
        @Override
        public void onOpened() {
            mBase.setNavigationDataSource(Base.NAVIGATION_SOURCE_TYPE_VLS);
            mBase.setOnCheckPointArrivedListener(mCheckPointListener);
        }

        @Override
        public void onError(String errorMessage) {
            Log.d(TAG, "ERROR: " + errorMessage);
        }
    };

    private CheckPointStateListener mCheckPointListener = new CheckPointStateListener() {
        @Override
        public void onCheckPointArrived(CheckPoint checkPoint, Pose2D realPose, boolean isLast) {
            Log.d(TAG, "Arrived at check point: " + checkPoint);

            if (isLast) endNav();
        }

        @Override
        public void onCheckPointMiss(CheckPoint checkPoint, Pose2D realPose, boolean isLast, int reason) {

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

    /*
    * All Voice Recognition Definitions
    * */

    //voice wake up listener
    private WakeupListener mWakeupListener = new WakeupListener() {
        @Override
        public void onStandby() {
            Log.d(TAG, "[WakeupListener](onStandby)");
        }

        @Override
        public void onWakeupResult(WakeupResult wakeupResult) {
            Log.d(TAG, "[WakeupListener](onWakeupResult): Wakeup word:" + wakeupResult.getResult() + ", angle " + wakeupResult.getAngle());

        }

        @Override
        public void onWakeupError(String error) {
            Log.d(TAG, "[WakeupListener](nWakeupError): " + error);
        }
    };

    //voice recognition listener
    private RecognitionListener mRecognitionListener = new RecognitionListener() {
        @Override
        public void onRecognitionStart() {
            Log.d(TAG, "Starting voice recognition...");
        }

        @Override
        public boolean onRecognitionResult(RecognitionResult recognitionResult) {
            Log.d(TAG, String.valueOf(recognitionResult.getConfidence()));
            Log.d(TAG, recognitionResult.toString());

            String result = recognitionResult.getRecognitionResult();

            if (result.equals("say hello") || result.equals(("hi"))) {
                speak("Hello.", 100);
            }
            else if (result.equals("say goodbye") || result.equals("say bye")) {
                speak("Goodbye.", 100);
            }

            return false;
        }

        @Override
        public boolean onRecognitionError(String error) {
            return false;
        }
    };

    private void setBindRecognitionService() {
        Log.d(TAG, "SETTING UP BIND STATE LISTENERS");

        //service binder listener service
        mRecognitionBindStateListener = new ServiceBinder.BindStateListener() {
            @Override
            public void onBind() {
                Log.d(TAG, "BIND STATE LISTENER BINDING");

                try {
                    //get recognition language when service bind
                    addEnglishGrammar();
                } catch (VoiceException e) {
                    Log.e(TAG, "BIND STATE LISTENER EXCEPTION: ", e);
                }

                //try and start the voice recognition service
                try {
                    Log.d(TAG, "STARTING WAKEUP AND RECOGNITION LISTENERS");
                    mRecognizer.startWakeupAndRecognition(mWakeupListener, mRecognitionListener);
                } catch (VoiceException e) {
                    Log.d(TAG, "STARTUP VOICE EXCEPTION: ",e);
                }

                bindRecognitionService = true;
            }

            @Override
            public void onUnbind(String reason) {
                bindRecognitionService = false;
            }
        };

        mSpeakerBindStateListener = new ServiceBinder.BindStateListener() {
            @Override
            public void onBind() {
                Log.d(TAG, "STARTING SPEAKER BIND LISTENER");

                bindSpeakerService = true;

               //tell the user Loomo's speech is now working
                speak("I am alive.", 100);
            }

            @Override
            public void onUnbind(String reason) {

            }
        };

        mTtsListener = new TtsListener() {
            @Override
            public void onSpeechStarted(String s) {
                //s is speech content, callback this method when speech is starting.
                Log.d(TAG, "onSpeechStarted() called with: s = [" + s + "]");
            }

            @Override
            public void onSpeechFinished(String s) {
                //s is speech content, callback this method when speech is finish.
                Log.d(TAG, "onSpeechFinished() called with: s = [" + s + "]");
            }

            @Override
            public void onSpeechError(String s, String s1) {
                //s is speech content, callback this method when speech occurs error.
                Log.d(TAG, "onSpeechError() called with: s = [" + s + "], s1 = [" + s1 + "]");
            }
        };
    }

    private void addEnglishGrammar() throws VoiceException {
        Log.d(TAG, "ADDING ENGLISH GRAMMAR");
        String grammarJson = "{\n" +
                "         \"name\": \"play_media\",\n" +
                "         \"slotList\": [\n" +
                "             {\n" +
                "                 \"name\": \"play_cmd\",\n" +
                "                 \"isOptional\": false,\n" +
                "                 \"word\": [\n" +
                "                     \"say\",\n" +
                "                     \"close\"\n" +
                "                 ]\n" +
                "             },\n" +
                "             {\n" +
                "                 \"name\": \"media\",\n" +
                "                 \"isOptional\": false,\n" +
                "                 \"word\": [\n" +
                "                     \"hello\",\n" +
                "                     \"hi\",\n" +
                "                     \"goodbye\",\n" +
                "                     \"bye\"\n" +
                "                 ]\n" +
                "             }\n" +
                "         ]\n" +
                "     }";

        GrammarConstraint mTwoSlotGrammar = mRecognizer.createGrammarConstraint(grammarJson);
        mRecognizer.addGrammarConstraint(mTwoSlotGrammar);
    }

    //making speech easier and also a class call
    private void speak(String message, int volume) {
        try {
            mSpeaker.setVolume(volume);
            mSpeaker.speak(message, mTtsListener);
        } catch (VoiceException e) {
            Log.e(TAG, "SPEAKING EXCEPTION: ", e);
        }
    }
}
