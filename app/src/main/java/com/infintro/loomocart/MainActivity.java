package com.infintro.loomocart;

import android.app.Activity;
import android.graphics.SurfaceTexture;
import android.os.Bundle;
import android.util.Log;
import android.view.TextureView;
import android.view.View;
import android.widget.CheckBox;

public class MainActivity extends Activity  {

    public static final String TAG = "FollowMeActivity";

    private static final int PREVIEW_WIDTH = 640;
    private static final int PREVIEW_HEIGHT = 480;

    private AutoFitDrawableView mAutoDrawable;
    private VisionPresenter mVisionPresenter;

    private CheckBox mFollowSwitch;
    private CheckBox mNavSwitch;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        initView();
        initListener();
    }

    @Override
    protected void onResume() {
        super.onResume();
        int rotation = getWindowManager().getDefaultDisplay().getRotation();
        mAutoDrawable.setPreviewSizeAndRotation(PREVIEW_WIDTH, PREVIEW_HEIGHT, rotation);
        mAutoDrawable.setSurfaceTextureListenerForPerview(mSurfaceTextureListener);
    }

    @Override
    protected void onStop() {
        super.onStop();
        mVisionPresenter.stopPresenter();
        finish();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        mVisionPresenter.stopPresenter();
        finish();
    }

    //@SuppressLint("WrongViewCast")
    private void initView() {
        mAutoDrawable = (AutoFitDrawableView) findViewById(R.id.drawableView);

        mFollowSwitch = (CheckBox) findViewById(R.id.followMode);
        mNavSwitch = (CheckBox) findViewById(R.id.navigationMode);
    }

    private void initListener() {
        mFollowSwitch.setOnClickListener(mFollowListener);
        mNavSwitch.setOnClickListener(mNavListener);
    }

    private TextureView.SurfaceTextureListener mSurfaceTextureListener = new TextureView.SurfaceTextureListener() {
        @Override
        public void onSurfaceTextureAvailable(SurfaceTexture surfaceTexture, int i, int i1) {
            mVisionPresenter = new VisionPresenter(mViewChangeInterface);
            mVisionPresenter.startPresenter();
        }

        @Override
        public void onSurfaceTextureSizeChanged(SurfaceTexture surfaceTexture, int i, int i1) {

        }

        @Override
        public boolean onSurfaceTextureDestroyed(SurfaceTexture surfaceTexture) {
            return false;
        }

        @Override
        public void onSurfaceTextureUpdated(SurfaceTexture surfaceTexture) {

        }
    };

    private ViewChangeInterface mViewChangeInterface = new ViewChangeInterface() {
        @Override
        public AutoFitDrawableView getAutoDrawable() {
            return mAutoDrawable;
        }
    };

//    @Override
//    public void onClick(View v) {
//        if (!mVisionPresenter.isServicesAvailable()) return;
//
//        switch (v.getId()) {
//            case R.id.followMode:
//                mFollowSwitch.toggle();
//                mNavSwitch.setChecked(false);
//                break;
//            case R.id.navigationMode:
//                mNavSwitch.toggle();
//                mFollowSwitch.setChecked(false);
//                break;
//            default:
//                break;
//        }
//    }


    View.OnClickListener mFollowListener = (new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            Log.d("Follow Listener", "Follow Mode: Clicked");
            if (mFollowSwitch.isChecked()) {
                mNavSwitch.setChecked(false);
                mVisionPresenter.endNav();
                mVisionPresenter.beginFollow();
            }
            else {
                mVisionPresenter.endFollow();
            }
        }
    });

    View.OnClickListener mNavListener = (new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            Log.d("Nav Listener", "Nav Mode: Clicked");
            if (mNavSwitch.isChecked()) {
                mFollowSwitch.setChecked(false);
                mVisionPresenter.endFollow();
                mVisionPresenter.beginNav();
            }
            else {
                mVisionPresenter.endNav();
            }
        }
    });
}
