package com.infintro.loomocart;

import android.app.Activity;
import android.graphics.SurfaceTexture;
import android.os.Bundle;
import android.util.Log;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;

public class MainActivity extends Activity  {

    public static final String TAG = "FollowMeActivity";

    private static final int PREVIEW_WIDTH = 640;
    private static final int PREVIEW_HEIGHT = 480;

    private AutoFitDrawableView mAutoDrawable;
    private VisionPresenter mVisionPresenter;

    private Button mFollowButton;
    private Button mHomeButton;
    private Button mBrd1Button;
    private Button mBrd2Button;
    private Button mBrd3Button;
    private Button mLobbyButton;

    private LinearLayout mButtonLayout;

    private boolean isFollowing;

    private interface NavReachedListner {
        public void onChange();
    }

    private NavReachedListner mNavComplete = new NavReachedListner() {
        @Override
        public void onChange() {
            if (mVisionPresenter.getState() == VisionPresenter.States.END_NAV) {
                mButtonLayout.setEnabled(true);
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        initView();
        initListener();
        isFollowing = false;
    }

    @Override
    protected void onResume() {
        super.onResume();
        int rotation = getWindowManager().getDefaultDisplay().getRotation();
        mAutoDrawable.setPreviewSizeAndRotation(PREVIEW_WIDTH, PREVIEW_HEIGHT, rotation);
        mAutoDrawable.setSurfaceTextureListenerForPerview(mSurfaceTextureListener);
        isFollowing = false;
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
        mAutoDrawable = findViewById(R.id.drawableView);

        mFollowButton = findViewById(R.id.followButton);
        mHomeButton = findViewById(R.id.homeButton);
        mBrd1Button = findViewById(R.id.brd1Button);
        mBrd2Button = findViewById(R.id.brd2Button);
        mBrd3Button = findViewById(R.id.brd3Button);
        mLobbyButton = findViewById(R.id.lobbyButton);

        mButtonLayout = findViewById(R.id.buttonLayout);
    }

    private void initListener() {
        mFollowButton.setOnClickListener(followListener);
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

    private View.OnClickListener followListener = new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            if (!isFollowing) {
                mVisionPresenter.beginFollow();
                isFollowing = true;
            }
            else  {
                mVisionPresenter.endFollow();
                isFollowing = false;
            }

            Log.d(TAG, "Following State: " + isFollowing);
        }
    };

    private View.OnClickListener homeListener = new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            if (isFollowing) mVisionPresenter.endNav();

            mVisionPresenter.beginNav(VisionPresenter.PATH.HOME);
        }
    };
}
