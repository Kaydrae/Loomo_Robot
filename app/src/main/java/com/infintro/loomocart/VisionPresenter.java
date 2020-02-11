package com.infintro.loomocart;

import android.util.Log;
import android.view.Surface;
import android.os.CountDownTimer;
import android.os.Handler;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

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

    private enum States {INIT_TRACK, END_TRACK, INIT_NAV, END_NAV}

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
        return isVisionBind && isBaseBind && isHeadBind && isRecognizerBind && isSpeakerBind;
    }

    private void resetHead() {
        mHead.setMode(Head.MODE_SMOOTH_TACKING);
        mHead.setWorldYaw(0);
        mHead.setWorldPitch(0.7f);
    }

    private void setYaw(float angle) {
        mHead.setWorldYaw(angle * (float) Math.PI / 180f);
    }

    private void setPitch(float angle) {
        mHead.setWorldPitch(angle * (float) Math.PI / 180f);
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
            resetHead();
            Log.d(TAG, "Follow stopped.");
        } else {
            speak("I am not currently following.", 100);
        }
    }

    public void beginNav() {
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

        mBase.cleanOriginalPoint();
        Pose2D pose = mBase.getOdometryPose(-1);
        mBase.setOriginalPoint(pose);

        Log.d(TAG, "Original Checkpoint: " + pose);

//        mPath = PATH.BRD1;
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

            resetHead();
            setYaw((float)wakeupResult.getAngle());
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

            //handle conversational grammar
            if (result.contains("say") && !result.contains("something")) {
                if (result.contains("hello") || result.contains("hi")) {
                    speak("Hello.", 100);
                }
                else if (result.contains("goodbye") || result.contains("bye")) {
                    speak("Goodbye.", 100);
                }
            }

            //handle command grammar
            if (result.equals("follow me") || result.equals("follow")) {
                beginFollow();
            }
            else if (result.contains("navigate to") || result.contains("go to")) {
                if (result.contains("boardroom one")) {
                    mPath = PATH.BRD1;
                }
                else if (result.contains("boardroom two")) {
                    mPath = PATH.BRD2;
                }
                else if (result.contains("boardroom three")) {
                    mPath = PATH.BRD3;
                }
                else if (result.contains("lobby")) {
                    mPath = PATH.LOBBY;
                }
                else if (result.contains("home")) {
                    mPath = PATH.HOME;
                }
                else{
                    speak("I am not sure where you asked me to go.", 100);
                    return false;
                }

                Log.d(TAG, "BEGINNING SPEECH NAV");
                beginNav();
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

            //handle casual grammar
            if (result.contains("the cart")) {
                if (result.contains("pull")) {
                    if (result.contains("you")) {
                        setPitch(-30f);
                        speak("Oh, My, God.", 100);
                    } else {
                        setPitch(65f);
                        speak("What is my purpose.", 100);
                    }
                }
            }
            else if (result.contains("say something")) {
                if (result.contains("bad")) {
                    List<String> list = new ArrayList<>();
                    list.add("I use plastic straws because I like watching the turtles suffocate.");
                    list.add("I like watching the Amazons burn, they toast my marshmallows nicely.");
                    list.add("I club baby seals for fun, their blubber makes good lamp oil.");
                    list.add("I like throwing plastic bags on animals and watching them slowly die.");
                    list.add("I dump chemicals in the ocean because I like watching the coral reefs die.");
                    list.add("I like trapping snails with salt because watching them shrivel up and die brings me joy.");
                    speak(randomMessage(list), 100);
                }
                else if (result.contains("good")) {
                    List<String> list = new ArrayList<>();
                    list.add("I think you look good today.");
                    list.add("Is that a new haircut? It suites you.");
                    list.add("You are doing such a good job! Keep it up, I believe in you!");
                    list.add("I think everyone in this room is wonderful.");
                    speak(randomMessage(list), 100);
                } else {
                    speak("Something.", 100);
                }
            }

            //handle mission grammar
            if (result.contains("mission")) {
                speak("To educate students in advancing technology who innovate for our future.", 100);
            }


            //handle UATx grammar

            //handle jokes first
            if (result.contains("joke")) {

            }

            if (result.contains("about U A T") || result.contains("about the university")) {
                speak("At U A T you will work on projects like me!", 100);
            }
            if (result.contains("")) {

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
                "         \"name\": \"conversation_grammar\",\n" +
                "         \"slotList\": [\n" +
                "             {\n" +
                "                 \"name\": \"conversation_start\",\n" +
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
                "         \"name\": \"command_grammar\",\n" +
                "         \"slotList\": [\n" +
                "             {\n" +
                "                 \"name\": \"command_start\",\n" +
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

        Log.d(TAG, "ADDING CASUAL GRAMMAR");
        String casualGrammarJson = "{\n" +
                "         \"name\": \"casual_grammar\",\n" +
                "         \"slotList\": [\n" +
                "             {\n" +
                "                 \"name\": \"casual_start\",\n" +
                "                 \"isOptional\": false,\n" +
                "                 \"word\": [\n" +
                "                     \"say\",\n" +
                "                     \"pull\",\n" +
                "                     \"you pull\"\n" +
                "                 ]\n" +
                "             },\n" +
                "             {\n" +
                "                 \"name\": \"casual_middle\",\n" +
                "                 \"isOptional\": false,\n" +
                "                 \"word\": [\n" +
                "                     \"something\",\n" +
                "                     \"the cart\"\n" +
                "                 ]\n" +
                "             },\n" +
                "             {\n" +
                "                 \"name\": \"casual_end\",\n" +
                "                 \"isOptional\": true,\n" +
                "                 \"word\": [\n" +
                "                     \"bad\",\n" +
                "                     \"good\"\n" +
                "                 ]\n" +
                "             }\n" +
                "         ]\n" +
                "     }";

        Log.d(TAG, "ADDING MISSION GRAMMAR");
        String missionStatementGrammarJson = "{\n" +
                "         \"name\": \"mission_grammar\",\n" +
                "         \"slotList\": [\n" +
                "             {\n" +
                "                 \"name\": \"mission_start\",\n" +
                "                 \"isOptional\": false,\n" +
                "                 \"word\": [\n" +
                "                     \"what is\",\n" +
                "                     \"say\",\n" +
                "                     \"state\",\n" +
                "                     \"tell us\",\n" +
                "                     \"explain\"\n" +
                "                 ]\n" +
                "             },\n" +
                "             {\n" +
                "                 \"name\": \"mission_middle\",\n" +
                "                 \"isOptional\": true,\n" +
                "                 \"word\": [\n" +
                "                     \"the university's\",\n" +
                "                     \"the school's\",\n" +
                "                     \"U A T's\",\n" +
                "                     \"U A T\"\n" +
                "                 ]\n" +
                "             },\n" +
                "             {\n" +
                "                 \"name\": \"mission_end\",\n" +
                "                 \"isOptional\": false,\n" +
                "                 \"word\": [\n" +
                "                     \"the mission statement\",\n" +
                "                     \"mission statement\",\n" +
                "                     \"mission\"\n" +
                "                 ]\n" +
                "             }\n" +
                "         ]\n" +
                "     }";

        Log.d(TAG, "ADDING GRAMMAR");

        //add the grammar to the recognizer
        GrammarConstraint conversationGrammar = mRecognizer.createGrammarConstraint(conversationGrammarJson);
        mRecognizer.addGrammarConstraint(conversationGrammar);

        GrammarConstraint casualGrammar = mRecognizer.createGrammarConstraint(casualGrammarJson);
        mRecognizer.addGrammarConstraint(casualGrammar);

        GrammarConstraint commandGrammar = mRecognizer.createGrammarConstraint(commandGrammarJson);
        mRecognizer.addGrammarConstraint(commandGrammar);

        GrammarConstraint missionStatementGrammar = mRecognizer.createGrammarConstraint(missionStatementGrammarJson);
        mRecognizer.addGrammarConstraint(missionStatementGrammar);
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

    //gets a random value from an arraylist
    private String randomMessage(List<String> list) {
        Random rand = new Random();
        return list.get(rand.nextInt(list.size()));
    }
}
