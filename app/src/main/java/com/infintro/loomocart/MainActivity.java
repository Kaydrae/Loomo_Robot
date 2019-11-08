package com.infintro.loomocart;

import android.app.Activity;
import android.graphics.SurfaceTexture;
import android.os.Bundle;
import android.view.TextureView;
import android.view.View;

public class MainActivity extends Activity implements View.OnClickListener {

    public static final String TAG = "FollowMeActivity";

    private static final int PREVIEW_WIDTH = 640;
    private static final int PREVIEW_HEIGHT = 480;

    private AutoFitDrawableView mAutoDrawable;
    private VisionPresenter mVisionPresenter;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        initView();
        initListener();
        mVisionPresenter = new VisionPresenter(mViewChangeInterface);
        mVisionPresenter.startPresenter();
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

    //@SuppressLint("WrongViewCast")
    private void initView() {
        mAutoDrawable = (AutoFitDrawableView) findViewById(R.id.drawableView);
    }

    private void initListener() {

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

    @Override
    public void onClick(View v) {

    }
}
