package com.labs.leonardo.ar_test;


import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.Paint;
import android.graphics.PixelFormat;
import android.graphics.Rect;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.support.v4.app.ActivityCompat;
import android.util.Log;
import android.util.Size;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.widget.ImageView;

import org.opencv.core.CvType;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.android.Utils;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
//import org.opencv.features2d.DescriptorExtractor;
//import org.opencv.features2d.DescriptorMatcher;
import org.opencv.core.Scalar;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.Features2d;
import org.opencv.features2d.ORB;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;


import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.AbstractSequentialList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import static android.content.Context.CAMERA_SERVICE;
import static android.graphics.Color.GREEN;
import static android.hardware.camera2.params.RggbChannelVector.RED;
import static android.os.Looper.getMainLooper;
import static com.labs.leonardo.ar_test.MainActivity.TAG;
import static com.labs.leonardo.ar_test.MainActivity.chooseBigEnoughSize;
import static com.labs.leonardo.ar_test.MainActivity.getScreenHeight;
import static com.labs.leonardo.ar_test.MainActivity.getScreenWidth;
import static org.opencv.core.CvType.CV_8UC1;


public class ASUforia implements SurfaceHolder.Callback{
    static Bitmap img2,outImg;
    static Mat img1;
    private static Mat mat1 = new Mat();

    // Camera2 part
    private CameraManager mCameraManager;
    private String mCameraId=new String();
    private CameraCharacteristics cameraCharacteristics;
    private ImageReader mCaptureBuffer;
    SurfaceView mSurfaceView, transparentView;
    Surface imgSurface;
    boolean mGotSecondCallback;
    CameraDevice mCamera;
    CameraCaptureSession mCaptureSession;
    SurfaceHolder holder, holderTransparent;
    Context context;


    // An additional thread for running tasks that shouldn't block the UI.
    HandlerThread mBackgroundThread;

    // Handler for running tasks in the background.
    Handler mBackgroundHandler;

    Handler mForegroundHandler;

    Activity mActivity;

    private  static  final int REQUEST_CAMERA_PERMISSION = 200;

    private float RectLeft, RectTop,RectRight,RectBottom ;
    int  deviceHeight,deviceWidth;

//    FeatureDetector detector;
//    DescriptorExtractor descriptor;
    static ORB detector;
//    static DescriptorMatcher matcher;
    static Mat descriptors2,descriptors1;
    static MatOfKeyPoint keypoints1,keypoints2;

    ASUforia() {
//        img1=lena_img;
//        img2=lena_img2;
    }








    public void startEstimation(SurfaceView _SurfaceView, Context _context, SurfaceView _transparentView, Activity _mActivity){

        mSurfaceView = _SurfaceView;
        transparentView = _transparentView;
        mGotSecondCallback = false;
        context = _context;
        mActivity = _mActivity;

        deviceWidth=getScreenWidth();
        deviceHeight=getScreenHeight();


        mCameraManager = (CameraManager) context.getSystemService(CAMERA_SERVICE);
        mBackgroundThread = new HandlerThread("background");
        mBackgroundThread.start();
        mBackgroundHandler = new Handler(mBackgroundThread.getLooper());
        mForegroundHandler = new Handler(getMainLooper());

        holderTransparent = transparentView.getHolder();
        holderTransparent.setFormat(PixelFormat.TRANSPARENT);
        holderTransparent.addCallback(this);
        transparentView.setZOrderMediaOverlay(true);


        try {
            mCameraId = mCameraManager.getCameraIdList()[0];
            cameraCharacteristics = mCameraManager.getCameraCharacteristics(mCameraId);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
        mSurfaceView.getHolder().addCallback((SurfaceHolder.Callback) this);

    }

    public void endEstimation(){
        try {
            // Ensure SurfaceHolderCallback#surfaceChanged() will run again if the user returns
            mSurfaceView.getHolder().setFixedSize(/*width*/0, /*height*/0);
            // Cancel any stale preview jobs
            if (mCaptureSession != null) {
                mCaptureSession.close();
                mCaptureSession = null;
            }
        } finally {
            if (mCamera != null) {
                mCamera.close();
                mCamera = null;
            }
        }
        mBackgroundThread.quitSafely();
        try {
            mBackgroundThread.join();
        } catch (InterruptedException ex) {
            Log.e(TAG, "Background worker thread was interrupted while joined", ex);
        }
        // Close the ImageReader now that the background thread has stopped
        if (mCaptureBuffer != null) mCaptureBuffer.close();

    }


    private void frame_refresh(SurfaceHolder holder, int width, int height) {
        // On the first invocation, width and height were automatically set to the view's size
        if (mCameraId == null) {
            // Find the device's back-facing camera and set the destination buffer sizes
            try {

                for(String cameraId : mCameraManager.getCameraIdList()){


                    if (cameraCharacteristics.get(cameraCharacteristics.LENS_FACING) ==
                                CameraCharacteristics.LENS_FACING_BACK) {
                            Log.i(TAG, "Found a back-facing camera");
                            StreamConfigurationMap info = cameraCharacteristics
                                    .get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                            // Bigger is better when it comes to saving our image
                            Size largestSize = Collections.max(
                                    Arrays.asList(info.getOutputSizes(ImageFormat.JPEG)),
                                    new MainActivity.CompareSizesByArea());
                            // Prepare an ImageReader in case the user wants to capture images
                            Log.i(TAG, "Capture size: " + largestSize);
                            mCaptureBuffer = ImageReader.newInstance(largestSize.getWidth(),
                                    largestSize.getHeight(), ImageFormat.JPEG, /*maxImages*/2);
                            imgSurface = mCaptureBuffer.getSurface();

                            mCaptureBuffer.setOnImageAvailableListener(
                                    mImageCaptureListener, mBackgroundHandler);
                            // Danger, W.R.! Attempting to use too large a preview size could
                            // exceed the camera bus' bandwidth limitation, resulting in
                            // gorgeous previews but the storage of garbage capture data.
                            Log.i(TAG, "SurfaceView size: " +
                                    mSurfaceView.getWidth() + 'x' + mSurfaceView.getHeight());
                            Size optimalSize = chooseBigEnoughSize(
                                    info.getOutputSizes(SurfaceHolder.class), width, height);
                            // Set the SurfaceHolder to use the camera's largest supported size
                            Log.i(TAG, "Preview size: " + optimalSize);
                            SurfaceHolder surfaceHolder = mSurfaceView.getHolder();
                            surfaceHolder.setFixedSize(optimalSize.getWidth(),
                                    optimalSize.getHeight());
                            mCameraId = cameraId;
                            return;
                            // Control flow continues with this method one more time
                            // (since we just changed our own size)
                    }
                }
            } catch (CameraAccessException ex) {
                Log.e(TAG, "Unable to list cameras", ex);
            }
            Log.e(TAG, "Didn't find any back-facing cameras");
            // This is the second time the method is being invoked: our size change is complete
        } else if (!mGotSecondCallback) {
            if (mCamera != null) {
                Log.e(TAG, "Aborting camera open because it hadn't been closed");
                return;
            }
            // Open the camera device
            try {
                if(ActivityCompat.checkSelfPermission(context,Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(context,Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED)
                {
                    ActivityCompat.requestPermissions(mActivity, new String[]{Manifest.permission.CAMERA, Manifest.permission.WRITE_EXTERNAL_STORAGE}, REQUEST_CAMERA_PERMISSION);
                    return;
                }
                mCameraManager.openCamera(mCameraId, mCameraStateCallback,
                        mBackgroundHandler);
            } catch (CameraAccessException ex) {
                Log.e(TAG, "Failed to configure output surface", ex);
            }
            mGotSecondCallback = true;
            // Control flow continues in mCameraStateCallback.onOpened()
        }
    }

    final CameraCaptureSession.StateCallback mCaptureSessionListener =
            new CameraCaptureSession.StateCallback() {
                @Override
                public void onConfigured(CameraCaptureSession session) {
                    Log.i(TAG, "Finished configuring camera outputs");
                    mCaptureSession = session;
                    holder = mSurfaceView.getHolder();
                    if (holder != null) {
                        try {
                            // Build a request for preview footage
                            CaptureRequest.Builder requestBuilder =
                                    mCamera.createCaptureRequest(mCamera.TEMPLATE_PREVIEW);
                            requestBuilder.addTarget(holder.getSurface());
                            CaptureRequest previewRequest = requestBuilder.build();
                            // Start displaying preview images
                            try {
                                session.setRepeatingRequest(previewRequest, /*listener*/null,
                                        /*handler*/null);
                            } catch (CameraAccessException ex) {
                                Log.e(TAG, "Failed to make repeating preview request", ex);
                            }
                        } catch (CameraAccessException ex) {
                            Log.e(TAG, "Failed to build preview request", ex);
                        }
                    }
                    else {
                        Log.e(TAG, "Holder didn't exist when trying to formulate preview request");
                    }
                }
                @Override
                public void onClosed(CameraCaptureSession session) {
                    mCaptureSession = null;
                }
                @Override
                public void onConfigureFailed(CameraCaptureSession session) {
                    Log.e(TAG, "Configuration error on device '" + mCamera.getId());
                }
    };


    /**
     * Calledbacks invoked upon state changes in our {@code CameraDevice}. <p>These are run on
     * {@code mBackgroundThread}.</p>
     */
    final CameraDevice.StateCallback mCameraStateCallback =
            new CameraDevice.StateCallback() {
                @Override
                public void onOpened(CameraDevice camera) {
                    Log.i(TAG, "Successfully opened camera");
                    mCamera = camera;
                    try {
                        List<Surface> outputs = Arrays.asList(
                                mSurfaceView.getHolder().getSurface(), mCaptureBuffer.getSurface());
                        camera.createCaptureSession(outputs, mCaptureSessionListener,
                                mBackgroundHandler);
                    } catch (CameraAccessException ex) {
                        Log.e(TAG, "Failed to create a capture session", ex);
                    }
                    // Control flow continues in mCaptureSessionListener.onConfigured()
                }
                @Override
                public void onDisconnected(CameraDevice camera) {
                    Log.e(TAG, "Camera was disconnected");
                }
                @Override
                public void onError(CameraDevice camera, int error) {
                    Log.e(TAG, "State error on device '" + camera.getId() + "': code " + error);
                }
            };


    final ImageReader.OnImageAvailableListener mImageCaptureListener =
            new ImageReader.OnImageAvailableListener() {
                @Override
                public void onImageAvailable(ImageReader reader) {
                    // Save the image once we get a chance
                    //mBackgroundHandler.post(new CapturedImageSaver(reader.acquireNextImage()));
                    // Control flow continues in CapturedImageSaver#run()
                    Image capture = reader.acquireNextImage();
                    System.out.println("**************** Captured Image ***************************");
                    Mat yuvMat = new Mat(capture.getHeight() + capture.getHeight() / 2, capture.getWidth(), CV_8UC1);
                    Mat rgbMat = new Mat(capture.getHeight(), capture.getHeight(), CvType.CV_8UC3);

                    ByteBuffer Y = capture.getPlanes()[0].getBuffer();
//                    ByteBuffer U = capture.getPlanes()[1].getBuffer();
//                    ByteBuffer V = capture.getPlanes()[2].getBuffer();

                    int Yb = Y.remaining();
//                    int Ub = U.remaining();
//                    int Vb = V.remaining();
                    System.out.println("Captured image size: " + capture.getWidth() + 'x' + capture.getHeight());
                    // Write the image out to the chosen file
//                    byte[] jpeg = new byte[buffer.remaining()];
//                    buffer.get(jpeg);
//                    ostream.write(jpeg);
//                    byte[] data = new byte[Yb + Ub + Vb ];
                    byte[] data = new byte[Yb];
                    Y.get(data, 0, Yb);
//                    U.get(data, Yb, Ub);
//                    V.get(data, Yb+ Ub, Vb);

                    System.out.println("RGB Mat data before"+rgbMat);
                    System.out.println("YUV Mat data before"+yuvMat);
                    yuvMat.put(0, 0, data);
                    System.out.println("sssssssssssssssssssssssssssssss");
                    Imgproc.cvtColor(yuvMat, rgbMat, Imgproc.COLOR_YUV420p2RGBA, 3);

                    yuvMat.release();
//                    mat1 = Imgcodecs.imdecode(new MatOfByte(data), Imgcodecs.CV_LOAD_IMAGE_UNCHANGED);
//                    String dump = rgbMat.dump();
                    Log.i(TAG,"-------------------------------------------");
                    System.out.println("RGB Mat data after"+rgbMat);
                    System.out.println("YUV Mat data after"+yuvMat);
                    //                    System.out.println(rgbMat);
//                    Log.d(TAG, dump);
                    Log.i(TAG,"-------------------------------------------");
//                    rgbMat.release();
//                    foriatry = new ASUforia(rgbMat);

//                    foriatry.startImageProcesing(ivBlurImage1);
//
                }
    };
    @Override
    public void surfaceCreated( SurfaceHolder holder) {
        Log.i(TAG, "------------------Surface created----------------");
        System.out.println("1. Surface created");
        mCameraId = null;
        mGotSecondCallback = false;
//        imgSurface = holder.getSurface();
//        Thread thread = new Thread(runnable);
//        thread.run();
        getFrame();
//        Draw();
        System.out.println("2. Surface created");
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        //frame_refresh( holder,  width,  height);
        //System.out.println("%%%%%%%%%%%%       Running in surface change");
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        Log.i(TAG, "Surface destroyed");
        holder.removeCallback(this);
    }

    private void getFrame() {
        System.out.println("Starting thread");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {

                mActivity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        System.out.println("Running thread");
                        while(true) {
                            frame_refresh(holder, deviceWidth, deviceHeight);
                            try {
                                Thread.sleep(2000);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }
                    }
                });
            }
        });

        thread.start();
        System.out.println("Done creating thread");
    }

    public void startImageProcesing(ImageView ivBlurImage) {
        Log.d(null, "Inside startImageProcesing: ");
        System.out.println("Inside startImageProcessing");
//        int mWidth = img1.getWidth();
//        int mHeight = img1.getHeight();

//        int[] mIntArray = new int[mWidth * mHeight];
//        byte[] data = new byte[16 * mHeight * mWidth];
        // Copy pixel data from the Bitmap into the 'intArray' array
//        img.getPixels(mIntArray, 0, mWidth, 0, 0, mWidth, mHeight);
        // Call to encoding function : convert intArray to Yuv Binary data
//        encodeYUV420SP(data, mIntArray, mWidth, mHeight);
//        Utils.bitmapToMat(img1, mat1);
        try {
            initializeOpenCVInstances(img1);

        }
        catch(IOException e){
            e.printStackTrace();
        }
        Mat release=new Mat();
        Mat rgb = new Mat();
//        Scalar color = new Scalar(0, 255, 0); // BGR
//        int flags = Features2d.DRAW_RICH_KEYPOINTS;

        Imgproc.cvtColor(img1, rgb, Imgproc.COLOR_RGBA2RGB);
        Features2d.drawKeypoints(rgb,keypoints1,rgb);
//        Highgui.imshow("Surf key pints");
        Imgproc.cvtColor(rgb, release, Imgproc.COLOR_RGB2RGBA);
        //Imgcodecs.imwrite("keypoints.jpg",release);

        //Utils.matToBitmap(release, outImg);
//        ivBlurImage.setImageBitmap(outImg);
        //displayImage(Mat2BufferedImage(release), "Feautures_"+detectorType);
        //match(img2);

    }

    static public Mat match(Bitmap img2)
    {
        Mat mat1_rgb = new Mat();
        Imgproc.cvtColor(mat1, mat1_rgb, Imgproc.COLOR_RGBA2RGB);
        Mat mat2=new Mat();
        Utils.bitmapToMat(img2, mat2);
        Imgproc.cvtColor(mat2, mat2, Imgproc.COLOR_RGBA2RGB);

        descriptors2 = new Mat();
        keypoints2 = new MatOfKeyPoint();
        detector.detect(mat2, keypoints2);
        detector.compute(mat2, keypoints2, descriptors2);


        MatOfDMatch matches = new MatOfDMatch();
        if (mat1_rgb.type() == mat2.type()) {
        DescriptorMatcher matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);
        matcher.match(descriptors1, descriptors2, matches);
        } else {
            return mat2;
        }
        List<DMatch> matchesList = matches.toList();
        Double max_dist = 0.0;
        Double min_dist = 100.0;

        for (int i = 0; i < matchesList.size(); i++) {
            Double dist = (double) matchesList.get(i).distance;
            if (dist < min_dist)
                min_dist = dist;
            if (dist > max_dist)
                max_dist = dist;
        }

        LinkedList<DMatch> good_matches = new LinkedList<DMatch>();
        for (int i = 0; i < matchesList.size(); i++) {
            if (matchesList.get(i).distance <= (1.5 * min_dist))
                good_matches.addLast(matchesList.get(i));
        }

        MatOfDMatch goodMatches = new MatOfDMatch();
        goodMatches.fromList(good_matches);
        Mat outputImg = new Mat();
        MatOfByte drawnMatches = new MatOfByte();
        if (mat2.empty() || mat2.cols() < 1 || mat2.rows() < 1) {
            return mat2;
        }
        Scalar color1 = new Scalar(0, 0, 255); // BGR
        Scalar color2 = new Scalar(0, 255, 0); // BGR
        System.out.println("Here");
        System.out.println(good_matches);
        Features2d.drawMatches(mat1_rgb, keypoints1, mat2, keypoints2, goodMatches, outputImg, color2, color1, drawnMatches, Features2d.NOT_DRAW_SINGLE_POINTS);
        Imgproc.resize(outputImg, outputImg, mat2.size());

        return outputImg;
    }


    public void initializeOpenCVInstances(Mat mat) throws IOException {
        Log.d(null, "Inside initializeOpenCVInstances: ");
        System.out.println("Inside initializeOpenCVInstances");
        detector=ORB.create(10000);
//            current_img=new Mat();
        keypoints1 = new MatOfKeyPoint();
        descriptors1=new Mat();
        detector.detect(mat,keypoints1);
        detector.compute(mat,keypoints1,descriptors1);
        List<KeyPoint> referenceKeypointsList =
                keypoints1.toList();
        System.out.println(referenceKeypointsList);
    }

    private void Draw()

    {

        Canvas canvas = holderTransparent.lockCanvas(null);

        Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);

        paint.setStyle(Paint.Style.STROKE);

        paint.setColor(Color.GREEN);

        paint.setStrokeWidth(3);

        RectLeft = 1;

        RectTop = 200 ;

        RectRight = RectLeft+ deviceWidth-100;

        RectBottom =RectTop+ 200;

        Rect rec=new Rect((int) RectLeft,(int)RectTop,(int)RectRight,(int)RectBottom);

        canvas.drawRect(rec,paint);

        holderTransparent.unlockCanvasAndPost(canvas);



    }

//    static public void encodeYUV420SP(byte[] yuv420sp, int[] rgba,
//                                      int width, int height) {
//        final int frameSize = width * height;
//
//        int[] U, V;
//        U = new int[frameSize];
//        V = new int[frameSize];
//
//        final int uvwidth = width / 2;
//
//        int r, g, b, y, u, v;
//        for (int j = 0; j < height; j++) {
//            int index = width * j;
//            for (int i = 0; i < width; i++) {
//
//                r = Color.red(rgba[index]);
//                g = Color.green(rgba[index]);
//                b = Color.blue(rgba[index]);
//
//                // rgb to yuv
//                y = (66 * r + 129 * g + 25 * b + 128) >> 8 + 16;
//                u = (-38 * r - 74 * g + 112 * b + 128) >> 8 + 128;
//                v = (112 * r - 94 * g - 18 * b + 128) >> 8 + 128;
//
//                // clip y
//                yuv420sp[index] = (byte) ((y < 0) ? 0 : ((y > 255) ? 255 : y));
//                U[index] = u;
//                V[index++] = v;
//            }
//        }
//
//    }

}
