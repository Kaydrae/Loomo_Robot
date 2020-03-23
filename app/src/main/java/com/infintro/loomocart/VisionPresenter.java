package com.infintro.loomocart;

import android.util.Log;
import android.view.Surface;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Stack;

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
    private LinearLayout mButtonLayout;

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


    //pathing variables

    private Stack reversePath;      // stack for storing the checkpoints we have already been through

    private Float[][][] paths = {
            {{0.91f, 0f, (float) (Math.PI/2)}, {0.91f, 9.14f, 0f}}, // lobby path
            {{2.44f, 0f, 0f}},                                      // board room 1
            {{7.32f, 0f, 0f}},                                      // board room 2
            {{12.19f, 0f, 0f}},                                     // board room 3
            {{1.52f, 0f, 0f}},                                      // table 1
            {{5.79f, 0f, 0f}},                                      // table 2
            {{7.62f, 0f, 0f}},                                      // table 3
            {{9.45f, 0f, 0f}},                                      // table 4
            {{12.8f, 0f, 0f}},                                      // table 5
            {{0f, 2.74f, 0f}},                                      // table 6
            {{0f, 0f, 0f}}                                          // home
    };

    private Pose2D pose;

    public enum PATH{LOBBY, BRD1, BRD2, BRD3, TAB1, TAB2, TAB3, TAB4, TAB5, TAB6, HOME};
    private PATH mPath;

    /* Initialize the Vision Presenter */
    public VisionPresenter(ViewChangeInterface _ViewInterface, LinearLayout _ButtonLayout) {
        mViewInterface = _ViewInterface;
        mButtonLayout = _ButtonLayout;
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

        reversePath = new Stack();

        mPersonTracking = new PersonTrackingProfile(3, 1.0f);

        mPath = PATH.HOME;
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

    private void setYaw(float angle) {
        mHead.setWorldYaw(angle * (float) Math.PI / 180f);
    }

    private void setPitch(float angle) {
        mHead.setWorldPitch(angle * (float) Math.PI / 180f);
    }

    private void disableButtons() {
        int buttonCount = mButtonLayout.getChildCount();
        Log.d(TAG, "Button Count: " + buttonCount);
        for (int i = 0; i < buttonCount; i++) {
            if (i != 1) {
                mButtonLayout.getChildAt(i).setEnabled(false);
            }
        }
    }

    private void enableButtons() {
        int buttonCount = mButtonLayout.getChildCount();
        Log.d(TAG, "Button Count: " + buttonCount);
        for (int i = 0; i < buttonCount; i++) {
            if (i != 1) {
                mButtonLayout.getChildAt(i).setEnabled(true);
            }
        }
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
        resetHead();

        mBase.setControlMode(Base.CONTROL_MODE_NAVIGATION);

        Log.d(TAG, "Control Mode: " + mBase.getControlMode());

        mBase.startVLS(true, true, mStartVLS);

        while (!(mBase.isVLSStarted()));

        mBase.setOnCheckPointArrivedListener(mCheckPointListener);

        if (mPath == PATH.HOME || mPath == null) {
            mBase.cleanOriginalPoint();
            pose = mBase.getOdometryPose(-1);
            mBase.setOriginalPoint(pose);
            reversePath.clear();
            reversePath.push(paths[PATH.HOME.ordinal()]);
        }

        Log.d(TAG, "Original Checkpoint: " + pose);

        mPath = _path;
        for (Float[] checkpoint : paths[mPath.ordinal()]) {
            int indexOfReversePath = reversePath.search(checkpoint);
            if (indexOfReversePath > -1) {
                for (int i = 1; i <= indexOfReversePath; i++) {
                    Float[] point = (Float[])reversePath.pop();
                    mBase.addCheckPoint(point[0], point[1], point[2]);
                }
            }
            else {
                int numCheckPoints = 0;
                for (int i = 0; i < mPath.ordinal()+1; i++) {
                    numCheckPoints++;
                }

                if (reversePath.size() > numCheckPoints) {
                    while (reversePath.size() > numCheckPoints) {
                        Float[] point = (Float[])reversePath.pop();
                        mBase.addCheckPoint(point[0], point[1], point[2]);
                    }
                }

                mBase.addCheckPoint(checkpoint[0], checkpoint[1], checkpoint[2]);
                reversePath.push(checkpoint);
            }
        }
        Log.d(TAG, "Added checkpoints...");
        Log.d(TAG, "Current Path: " + mPath);

        mState = States.INIT_NAV;
//        disableButtons();
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
//            enableButtons();
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
//                else if (result.contains("table one")) {
//                    beginNav(PATH.TAB1);
//                }
//                else if (result.contains("table two")) {
//                    beginNav(PATH.TAB2);
//                }
//                else if (result.contains("table three")) {
//                    beginNav(PATH.TAB3);
//                }
//                else if (result.contains(("table four"))) {
//                    beginNav(PATH.TAB4);
//                else if (result.contains("table five")) {
//                }
//                    beginNav(PATH.TAB5);
//                }
//                else if (result.contains("table six")) {
//                }
//                    beginNav(PATH.TAB6);
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


            //handle mission grammar
            if (result.contains("mission")) {
                speak("To educate students in advancing technology who innovate for our future.", 100);
            }


            //handle UATx grammar

            if (result.equals("what is zero divided by zero")) {
                speak("Imagine you have zero cookies to divide among 0 friends. See? It makes no sense, and cookie monster is sad there are no cookies, and you are sad you have no friends.", 100);
            }

            //handle jokes first
            if (result.contains("joke")) {
                List<String> list = new ArrayList<>();
                list.add("Why did the developer go broke? Because he used up all his cache");
                list.add("There's a band called 1023 Megabytes. They haven't had any gigs yet.");
                list.add("What am I, Alexa? Just kidding, I'm much more fun.");
                list.add("01100110 01110101");
                list.add("Why did the robot cross the road? Because it was programmed by a chicken!");
                list.add("Did you hear about Google's new AI powered robotic broom? Soon, it'll be sweeping the nation.");
                list.add("Have you guys heard of this new AI robot that can take off all your clothes, and then give you a whole new outfit? I've seen it change people.");
                list.add("Why is a robot mechanic never lonely? Because he is always making new friends!");
                speak(randomMessage(list), 100);
            }

            else if (result.contains("something")) {
                if (result.contains("about U A T") || result.contains("about the university") || result.contains("about the school")) {
                    List<String> list = new ArrayList<>();
                    list.add("At U A T you will work on projects like me!");
                    list.add("At U A T we offer degrees such as Computer Science, Game Programming, and Network Security");
                    list.add("At U A T we have cool projects for students such as the I o T Lab");
                    list.add("At U A T there are many clubs such as the E-sports Club, Social Gaming Club, and Robotics Club.");
                    list.add("At U A T we host events such as the Global Game Jam and I o T Dev-fest");
                    speak(randomMessage(list), 100);
                }
                else if (result.contains("about you")) {
                    List<String> list = new ArrayList<>();
                    list.add("I am a robot made by segway.");
                    list.add("I was programmed by Brandon, Dylan, and Ke'ondrae.");
                    list.add("I was made to be a transport and entertainment robot for perimeter 83.");
                    list.add("I am able to follow people around, just say 'okay loomo, follow me'");
                    list.add("I have multiple sensors including a depth sensing camera, a ultra sonic distance sensor, and a 5 way directional microphone.");
                    speak(randomMessage(list), 100);
                }
                else if (result.contains("bad")) {
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
                }
            }
            else {
                speak("Something.", 100);
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
                "                     \"follow\",\n" +
                "                     \"follow me\",\n" +
                "                     \"go to\",\n" +
                "                     \"navigate to\"\n" +
                "                 ]\n" +
                "             },\n" +
                "             {\n" +
                "                 \"name\": \"command_middle\",\n" +
                "                 \"isOptional\": true,\n" +
                "                 \"word\": [\n" +
                "                     \"table one\",\n" +
                "                     \"table two\",\n" +
                "                     \"table three\",\n" +
                "                     \"table four\",\n" +
                "                     \"table five\",\n" +
                "                     \"table six\",\n" +
                "                     \"admissions\",\n" +
                "                     \"student services\",\n" +
                "                     \"clubs\",\n" +
                "                     \"faculty\",\n" +
                "                     \"housing\",\n" +
                "                     \"financial aid\",\n" +
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
                "  \"name\": \"uatx_grammar\",\n" +
                "  \"slotList\": [\n" +
                "    {\n" +
                "      \"name\": \"uatx_start\",\n" +
                "      \"isOptional\": false,\n" +
                "      \"word\": [\n" +
                "        \"say\",\n" +
                "        \"state\",\n" +
                "        \"what is\",\n" +
                "        \"tell us\",\n" +
                "        \"explain\",\n" +
                "        \n" +
                "        \"what is zero divided by zero\",\n" +
                "        \n" +
                "        \"pull the cart\",\n" +
                "        \"you pull the cart\",\n" +
                "        \n" +
                "        \"tell me\",\n" +
                "        \"navigate to\"\n" +
                "      ]\n" +
                "    },\n" +
                "    {\n" +
                "     \"name\": \"uatx_middle\",\n" +
                "      \"isOptional\": true,\n" +
                "      \"word\": [\n" +
                "        \"hi\",\n" +
                "        \"hello\",\n" +
                "        \"something\",\n" +
                "        \"something about\",\n" +
                "        \"a joke\",\n" +
                "        \"a joke about\",\n" +
                "        \"home\",\n" +
                "        \"U A T\",\n" +
                "        \"U A T's\",\n" +
                "        \"the university\",\n" +
                "        \"the school\",\n" +
                "        \"the university's\",\n" +
                "        \"the school's\"\n" +
                "      ]\n" +
                "    },\n" +
                "    {\n" +
                "     \"name\": \"uatx_end\",\n" +
                "      \"isOptional\": true,\n" +
                "      \"word\": [\n" +
                "        \"U A T\",\n" +
                "        \"U A T's\",\n" +
                "        \"the university\",\n" +
                "        \"the school\",\n" +
                "        \"the university's\",\n" +
                "        \"the school's\",\n" +
                "        \"you\",\n" +
                "        \"a joke\",\n" +
                "        \"good\",\n" +
                "        \"bad\",\n" +
                "        \"the mission statement\",\n" +
                "        \"mission statement\",\n" +
                "        \"mission\"\n" +
                "      ]\n" +
                "    }\n" +
                "  ]\n" +
                "}";

        Log.d(TAG, "ADDING GRAMMAR");

        //add the grammar to the recognizer
        //GrammarConstraint conversationGrammar = mRecognizer.createGrammarConstraint(conversationGrammarJson);
        //mRecognizer.addGrammarConstraint(conversationGrammar);

        //GrammarConstraint casualGrammar = mRecognizer.createGrammarConstraint(casualGrammarJson);
        //mRecognizer.addGrammarConstraint(casualGrammar);

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
