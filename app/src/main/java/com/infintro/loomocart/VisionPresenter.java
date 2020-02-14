package com.infintro.loomocart;

import android.util.Log;
import android.view.Surface;

import java.util.ArrayList;
import java.util.List;

import com.segway.robot.algo.Pose2D;
import com.segway.robot.algo.dts.BaseControlCommand;
import com.segway.robot.algo.dts.DTSPerson;
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


public class VisionPresenter {
    private final static String TAG = "VisionPresenter";

    public enum States {INIT_TRACK, END_TRACK, INIT_NAV, END_NAV}

    private final static int TIME_OUT = 10*1000;

    private PresenterChangeInterface mPresInterface;
    private ViewChangeInterface mViewInterface;

    //service interfaces
    private Vision mVision;
    private Head mHead;
    private Base mBase;
    private HeadPIDController mHeadPID = new HeadPIDController();
    private Recognizer mRecognizer;
    private Speaker mSpeaker;

    //tracking variables
    private PersonTrackingProfile mPersonTracking;
    private DTSPerson[] mPersons;

    //booleans for checking of a given service is bound
    private boolean isHeadBind;
    private boolean isBaseBind;
    private boolean isVisionBind;
    private boolean isRecognizerBind;
    private boolean isSpeakerBind;

    private boolean noObstacles;

    private long startTime;

    private DTS mDTS;

    private States mState;

    //audio specific variables
    private ServiceBinder.BindStateListener mRecognitionBindStateListener;
    private ServiceBinder.BindStateListener mSpeakerBindStateListener;
    private TtsListener mTtsListener;

    //path variables
    private Float[][][] paths = {
            {{2.44f, 0f, 0f}},
            {{7.32f, 0f, 0f}},
            {{12.19f, 0f, 0f}},
            {{0.91f, 0f, (float) (Math.PI/2)}, {0.91f, 9.14f, 0f}},
            {{0f, 0f, 0f}}
    };

    private Pose2D pose;

    private List<Float[]> homePath;

    public enum PATH{BRD1, BRD2, BRD3, LOBBY, HOME}
    private PATH mPath;

    /* Initialize the Vision Presenter */
    public VisionPresenter(ViewChangeInterface _ViewInterface) {
        mViewInterface = _ViewInterface;
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
        return isVisionBind && isBaseBind && isHeadBind && isRecognizerBind && isSpeakerBind;
    }

    public States getState() {
        return mState;
    }

    private void resetHead() {
        mHead.setMode(Head.MODE_SMOOTH_TACKING);
        mHead.setWorldYaw(0);
        mHead.setWorldPitch(0.7f);
    }

    public void beginFollow() {
        if (mState == States.INIT_NAV) {
            speak("I cannot follow, I am currently navigating.", 100);
            return;
        }

        if (mState == States.INIT_TRACK) {
            speak("I am already following.", 100);
            return;
        }
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

            if (mPath == PATH.HOME) {
                mBase.cleanOriginalPoint();
                pose = mBase.getOdometryPose(-1);
                mBase.setOriginalPoint(pose);
            }

            resetHead();
            Log.d(TAG, "Follow stopped.");
        } else {
            speak("I am not currently following.", 100);
        }
    }

    public void beginNav(PATH _path) {
        //prevent starting navigation if it is in following mode
        if (mState == States.INIT_TRACK) {
            speak("I cannot navigate, I am currently following.", 100);
            return;
        }

        if (mState == States.INIT_NAV) {
            speak("I am already navigating.", 100);
            return;
        }
        Log.d(TAG, "Begging nav...");
        speak("I am on my way.", 100);

        mBase.setControlMode(Base.CONTROL_MODE_NAVIGATION);

        Log.d(TAG, "Control Mode: " + mBase.getControlMode());

        mBase.startVLS(true, true, mStartVLS);

        while (!(mBase.isVLSStarted()));

        mBase.setOnCheckPointArrivedListener(mCheckPointListener);

        if (mPath == PATH.HOME || mPath == null) {
            mBase.cleanOriginalPoint();
            pose = mBase.getOdometryPose(-1);
            mBase.setOriginalPoint(pose);
        }

        Log.d(TAG, "Original Checkpoint: " + pose);

        mPath = _path;
        for (Float[] checkpoint : paths[mPath.ordinal()]) {
            mBase.addCheckPoint(checkpoint[0], checkpoint[1], checkpoint[2]);
        }
        Log.d(TAG, "Added checkpoints...");
        Log.d(TAG, "Current Path: " + mPath);

        mState = States.INIT_NAV;
    }

    public void endNav() {
        if (mState == States.INIT_NAV) {
            Log.d(TAG, "Ending nav...");
            mBase.clearCheckPointsAndStop();
            mBase.stopVLS();
            //mBase.setControlMode(Base.CONTROL_MODE_RAW);
            //mBase.stop();
            mState = States.END_NAV;
            Log.d(TAG, "Nav stopped.");
            speak("I have arrived.", 100);
        } else {
            speak("I am not currently navigating.", 100);
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

            if (isLast) {
                Log.d(TAG, "ARRIVED AT LAST CHECKPOINT");

                try {
                    Log.d(TAG, "ENDING NAVIGATION VIA CHECKPOINT");
                    endNav();
                    Log.d(TAG, "ENDED NAVIGATION SUCCESSFULLY VIA CHECKPOINT");
                } catch (Exception e) {
                    Log.d(TAG, "CHECK POINT EXCEPTION: ", e);
                }
            }
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

    //function will tell the voice listeners if they are allowed to turn the head
    private boolean can_move_head() {
      return true;
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

            String result = wakeupResult.getResult();

            if (result.equals("stop")) {
                if (mState == States.INIT_NAV) {
                    endNav();
                } else if (mState == States.INIT_TRACK) {
                    endFollow();
                } else {
                    speak("I am not currently moving.", 100);
                }
                return;
            }

            float angle = (float)wakeupResult.getAngle()/(float)90;

            Log.d(TAG, "WAKEUP ANGLE: " + angle + " - " + wakeupResult.getAngle());

            resetHead();
            mHead.setIncrementalYaw(angle);
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

            Log.d(TAG, "SPEECH RECOGNITION RESULT: " + result);

            if (result.contains("say")) {
                if (result.contains("hello") || result.contains("hi")) {
                    speak("Hello.", 100);
                }
                else if (result.contains("goodbye") || result.contains("bye")) {
                    speak("Goodbye.", 100);
                }
            }
            else if (result.equals("follow me")) {
                beginFollow();
            }
            else if (result.contains("navigate to") || result.contains("go to")) {
                if (result.contains("boardroom one")) {
                    beginNav(PATH.BRD1);
                }
                else if (result.contains("boardroom two")) {
                    beginNav(PATH.BRD2);
                }
                else if (result.contains("boardroom three")) {
                    beginNav(PATH.BRD3);
                }
                else if (result.contains("lobby")) {
                    beginNav(PATH.LOBBY);
                }
                else if (result.contains("home")) {
                    beginNav(PATH.HOME);
                }
                else{
                    speak("I am not sure where you asked me to go.", 100);
                    return false;
                }
                Log.d(TAG, "BEGINNING SPEECH NAV");
            }
            else if (result.contains("stop")) {
                if (result.contains("following")) {
                    endFollow();
                } else if (result.contains("navigating")) {
                    endNav();
                } else {
                    if (mState == States.INIT_NAV) {
                        endNav();
                    } else if (mState == States.INIT_TRACK) {
                        endFollow();
                    } else {
                        speak("I am not currently moving.", 100);
                    }
                }
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

                isRecognizerBind = true;
            }

            @Override
            public void onUnbind(String reason) {
                isRecognizerBind = false;
            }
        };

        mSpeakerBindStateListener = new ServiceBinder.BindStateListener() {
            @Override
            public void onBind() {
                Log.d(TAG, "STARTING SPEAKER BIND LISTENER");

                isSpeakerBind = true;

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
        Log.d(TAG, "ADDING CONVERSATION GRAMMAR");
        String conversationGrammarJson = "{\n" +
                "         \"name\": \"conversation_start\",\n" +
                "         \"slotList\": [\n" +
                "             {\n" +
                "                 \"name\": \"play_cmd\",\n" +
                "                 \"isOptional\": false,\n" +
                "                 \"word\": [\n" +
                "                     \"say\"\n" +
                "                 ]\n" +
                "             },\n" +
                "             {\n" +
                "                 \"name\": \"conversation_middle\",\n" +
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

        Log.d(TAG, "ADDING COMMAND GRAMMAR");
        String commandGrammarJson = "{\n" +
                "         \"name\": \"command_start\",\n" +
                "         \"slotList\": [\n" +
                "             {\n" +
                "                 \"name\": \"play_cmd\",\n" +
                "                 \"isOptional\": false,\n" +
                "                 \"word\": [\n" +
                "                     \"follow me\",\n" +
                "                     \"go to\",\n" +
                "                     \"navigate to\"\n" +
                "                 ]\n" +
                "             },\n" +
                "             {\n" +
                "                 \"name\": \"command_middle\",\n" +
                "                 \"isOptional\": true,\n" +
                "                 \"word\": [\n" +
                "                     \"boardroom one\",\n" +
                "                     \"boardroom two\",\n" +
                "                     \"boardroom three\",\n" +
                "                     \"lobby\",\n" +
                "                     \"home\"\n" +
                "                 ]\n" +
                "             }\n" +
                "         ]\n" +
                "     }";


        //add the grammar to the recognizer
        GrammarConstraint conversationGrammar = mRecognizer.createGrammarConstraint(conversationGrammarJson);
        mRecognizer.addGrammarConstraint(conversationGrammar);

        GrammarConstraint commandGrammar = mRecognizer.createGrammarConstraint(commandGrammarJson);
        mRecognizer.addGrammarConstraint(commandGrammar);
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
