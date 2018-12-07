/*

ASUforia.java - AR library to


Main Activity : We implemented default android callbacks such as OnCreate(), OnResume(), On Pause(), OnDestroy() and our ASUforia framework's callback which is OnPose(). In MainActivity we are calling ASUforia constructor as mwntioned in the assignment requirement.
We have called the startEstimation() method in the OnResume() that starts the Camera Stream and sends the YUV420_888 format image to the OnImageListener defined in the ASUforia library.
We implemented the endEstimation() in the OnPause() callback to end the camera feed and start it again in by calling startEstimation() in onResume()
 */


package com.labs.leonardo.ar_test;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.Paint;
import android.graphics.PixelFormat;
import android.graphics.Rect;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureFailure;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.util.Size;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import static org.opencv.core.CvType.CV_8UC1;


public class MainActivity extends AppCompatActivity implements ASUforia.PoseListner{

    // Output files will be saved as /sdcard/Pictures/AR_test_*.jpg
    static final String CAPTURE_FILENAME_PREFIX = "AR_test_";

    // Tag to distinguish log prints.
    static final String TAG = "AR_Test";

    // An additional thread for running tasks that shouldn't block the UI.
    HandlerThread mBackgroundThread;

    // Handler for running tasks in the background.
    Handler mBackgroundHandler;

    // Handler for running tasks on the UI thread.
    Handler mForegroundHandler;

    // View for displaying the camera preview.
    SurfaceView mSurfaceView,transparentView;

    Surface imgSurface;

    // Reference Image Bitmap
    Bitmap ref_img;
    private Activity mActivity;

    SurfaceHolder holder,holderTransparent;

    // Used to retrieve the captured image when the user takes a snapshot.
    ImageReader mCaptureBuffer;

    // Handle to the Android camera services.
    CameraManager mCameraManager;

    // The specific camera device that we're using.
    CameraDevice mCamera;

    ASUforia myforia;

    // Our image capture session.
    CameraCaptureSession mCaptureSession;

    boolean mGotSecondCallback;

    int  deviceHeight,deviceWidth;
    private float RectLeft, RectTop,RectRight,RectBottom ;


    private  static  final int REQUEST_CAMERA_PERMISSION = 200;
    Activity mainactivty=this;
    private String mCameraId;
    private Runnable runnable;


    static Size chooseBigEnoughSize(Size[] choices, int width, int height) {
        // Collect the supported resolutions that are at least as big as the preview Surface
        List<Size> bigEnough = new ArrayList<Size>();
        for (Size option : choices) {
            if (option.getWidth() >= width && option.getHeight() >= height) {
                bigEnough.add(option);
            }
        }
        // Pick the smallest of those, assuming we found any
        if (bigEnough.size() > 0) {
            return Collections.min(bigEnough, new CompareSizesByArea());
        } else {
            Log.e(TAG, "Couldn't find any suitable preview size");
            return choices[0];
        }
    }
    /*
     Paint a user interface surface with the camera image with a cube as an overlay
    on the marker in the image.
     Sets up OpenCV to paint the image with Java code.
    o Create an Asuforia object
      */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        super.onResume();


        if (!OpenCVLoader.initDebug()) {
            Log.e(this.getClass().getSimpleName(), "  OpenCVLoader.initDebug(), not working.");
        } else {
            Log.d(this.getClass().getSimpleName(), "  OpenCVLoader.initDebug(), working.");
        }

//        mSurfaceView.setSecure(true);
        System.out.println("In On Create");


        BitmapFactory.Options options = new BitmapFactory.Options();
        options.inPreferredConfig = Bitmap.Config.ARGB_8888;
        try {
            ref_img = BitmapFactory.decodeResource(getApplicationContext().getResources(), R.drawable.lena, options);
            System.out.println("Ref Img height"+ref_img.getHeight()+" Width is:"+ref_img.getWidth());
        } catch (Exception e){

            e.printStackTrace();
        }


        deviceWidth=getScreenWidth();
        deviceHeight=getScreenHeight();
    }

    /**
     * Called when our Activity gains focus. Starts initializing the camera.
     * Initializes ASUforia object and sets up the reference image.
     */
    @Override
    protected void onResume() {
        super.onResume();

//        // Inflate the SurfaceView, set it as the main layout, and attach a listener
        setContentView(R.layout.activity_main);
        mSurfaceView = (SurfaceView) findViewById(R.id.mainSurfaceView);
        transparentView = (SurfaceView) findViewById(R.id.TransparetnView);
//        mSurfaceView.getHolder().addCallback((SurfaceHolder.Callback) this);
        mActivity = this;
        myforia = new ASUforia(ref_img,mSurfaceView);
        Context context = getApplicationContext();
        myforia.startEstimation(context,transparentView, mActivity);
        myforia.onPose();
        System.out.println("After ASUforia");

    }


    /**
     * Compares two sizes based on their areas.
     */
    static class CompareSizesByArea implements Comparator<Size> {
        @Override
        public int compare(Size lhs, Size rhs) {
            // We cast here to ensure the multiplications won't overflow
            return Long.signum((long) lhs.getWidth() * lhs.getHeight() -
                    (long) rhs.getWidth() * rhs.getHeight());
        }
    }

    // Gives screen width
    public static int getScreenWidth() {

        return Resources.getSystem().getDisplayMetrics().widthPixels;

    }


    // Gives screen height
    public static int getScreenHeight() {

        return Resources.getSystem().getDisplayMetrics().heightPixels;

    }


    /**
     * Called when our Activity loses focus.Tears everything back down.
     */
    @Override
    protected void onPause() {
        super.onPause();

        myforia.endEstimation();


    }
    // Implementeation of onPose callback
    @Override
    public void callbackonPose() {

        Log.d(null,"callback successful");
    }

}