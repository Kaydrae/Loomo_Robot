package com.infintro.loomocart;

import android.view.Surface;

import com.segway.robot.sdk.base.bind.ServiceBinder;
import com.segway.robot.sdk.vision.DTS;
import com.segway.robot.sdk.vision.Vision;

public class VisionPresenter {
    private final static String TAG = "VisionPresenter";

    private final static int TIME_OUT = 10*1000;

    private PresenterChangeInterface mPresInterface;
    private ViewChangeInterface mViewInterface;

    private Vision mVision;

    private boolean isVisionBind;

    private DTS mDTS;

    public VisionPresenter(ViewChangeInterface _ViewInterface) {
        mViewInterface = _ViewInterface;
    }

    public void startPresenter() {
        mVision = Vision.getInstance();

        mVision.bindService(CustomApplication.getContext(), mVisionBindStateListener);
    }

    public void stopPresenter() {
        if (mDTS != null) {
            mDTS.stop();
            mDTS = null;
        }

        mVision.unbindService();
    }

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
            //checkAvailable();
        }

        @Override
        public void onUnbind(String reason) {
            isVisionBind = false;
            //mPresInterface.showToast("Vision service: " + reason);
        }
    };
}
