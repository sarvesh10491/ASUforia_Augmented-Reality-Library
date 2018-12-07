/*

ASUforia.java - AR library to


This library will need to set up a Camera2 Device to capture frames and feed them to the pose
estimation. The library will also interact with the library user’s PoseListener, feeding coordinates and
images so the user.

 nativePoseEstimation(…) [Implemented as a java method]
o Uses OpenCV to determine pose.[Uses various opencv libraries ]
o Compare points against reference points [Stores reference image as a class variable and calculates scene image on the fly, also performs the comparison between them]
o Return rotation vector and translation vector[Stores both these parameters inside PNPRansac object]

 Asuforia Constructor Asuforia(PoseListener listener, Bitmap refImage, SurfaceView cameraSurface)
o Stores PoseListener argument to act as callback interface.
o Generates reference points from reference image.[stores sent bitmap into the referece image bitmap]
 Save the reference points for estimation usage [ defines Mat to calculate openCV calculations]

 startEstimation(…) : Starts the estimation process
o Start camera (Camera2) in preview mode, sending YUV420_888 to
onImageAvailable()

 onImageAvailable(…) implementation : Callback for Camera2 frames
o Call nativePoseEstimation() method
o Call the PoseListener’s function with the rotation and translation vector.

 endEstimation(…) : Ends the estimation process
o Stops camera

 */

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
import android.view.View;
import android.widget.ImageView;

import org.opencv.core.CvType;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.android.Utils;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfKeyPoint;
//import org.opencv.features2d.DescriptorExtractor;
//import org.opencv.features2d.DescriptorMatcher;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.Features2d;
import org.opencv.features2d.ORB;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;


import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.AbstractSequentialList;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import static android.content.Context.CAMERA_SERVICE;
//import static android.graphics.Color.GREEN;
//import static android.hardware.camera2.params.RggbChannelVector.RED;
import static android.os.Looper.getMainLooper;
//import static com.labs.leonardo.ar_test.MainActivity.TAG;
//import static com.labs.leonardo.ar_test.MainActivity.chooseBigEnoughSize;
//import static com.labs.leonardo.ar_test.MainActivity.getScreenHeight;
//import static com.labs.leonardo.ar_test.MainActivity.getScreenWidth;

import static com.labs.leonardo.ar_test.MainActivity.TAG;
import static com.labs.leonardo.ar_test.MainActivity.chooseBigEnoughSize;
import static com.labs.leonardo.ar_test.MainActivity.getScreenHeight;
import static com.labs.leonardo.ar_test.MainActivity.getScreenWidth;
import static org.opencv.calib3d.Calib3d.CV_ITERATIVE;
import static org.opencv.calib3d.Calib3d.Rodrigues;
import static org.opencv.calib3d.Calib3d.solvePnPRansac;
import static org.opencv.core.CvType.CV_32F;
import static org.opencv.core.CvType.CV_64FC1;
import static org.opencv.core.CvType.CV_8UC1;
import static org.opencv.core.Mat.zeros;


public class ASUforia implements SurfaceHolder.Callback{


    interface PoseListner {

        void callbackonPose();
    }

    PoseListner poseListner;

    void registerCallback(PoseListner PoseListnerClass) {
        poseListner = PoseListnerClass;
    }

    void onPose() {
        poseListner.callbackonPose();
    }


    List<KeyPoint> keypoint_ref_img_list;
    List<KeyPoint> keypoint_scene_img_list;
    static Bitmap ref_img_bitmap,outImg;
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

    ASUforia(Bitmap ref_img,SurfaceView cameraView) {

        ref_img_bitmap=ref_img;
        mSurfaceView=cameraView;
//        img2=lena_img2;
    }




    /*  Method      : startEstimation
        Arguments   : (Context _context) gets the context of Main Activity, (SurfaceView _transparentView) gets the
                       surface view of the canvas to draw the AR image on, (Activity _mActivity) gets the activity
                       object reference of the Main Activity
        Returns     : void

        The call to startEstimation initializes the camera manager by getting it from the camera Service and registers the
        callback for the surface view. The srface created is hence called from which a thread is started to capture the images
        from the camera stream. Thus the camera is started in the preview mode and the call back function send YUV420_888
        to onImageAvailabke callback.

    */
    public void startEstimation(Context _context, SurfaceView _transparentView, Activity _mActivity){

//        mSurfaceView = _SurfaceView;
        transparentView = _transparentView;
        mGotSecondCallback = false;
        context = _context;
        mActivity = _mActivity;

        deviceWidth=getScreenWidth();
        deviceHeight=getScreenHeight();

        getRefImageParams();
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

    /*  Method      : endEstimation
        Arguments   : (no arguments)
        Returns     : void

        The call to endEstimation sees to it the camera is properly closed by setting the camera device to NULL and
        setting the Capture Session to NULL and closing it.
        We also see to it the threads are exited properly by joining them.
    */
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
            Log.e(null, "Background worker thread was interrupted while joined", ex);
        }
        // Close the ImageReader now that the background thread has stopped
        if (mCaptureBuffer != null) mCaptureBuffer.close();

    }


    /*  Method      : frame_refresh
        Arguments   : (SurfaceHolder holder) gets the Surface holder of the camera , (int width) gets the width of the surface view,
                      (int height) gets the height of the surface view
        Returns     : void

        frame_refresh is called from the getframe() method and is called in the thread and is run to get the camera stream
        in real time. Sets up mImageCaptureListener which registers the onImageavailable callback. This method also sets the surface view layout and
        creates preview for the camera

    */
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
/*  Callback      :  CameraCaptureSession.StateCallback mCaptureSessionListener

    This is callback implementation to service the image capture. Once the Camera surfaceview is set , this callback provides mechanism to hold
    the surfaceview and captures frame after setting up the camera.
*/

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
                            System.out.println("Inside statecallback try1");
                            CaptureRequest.Builder requestBuilder =
                                    mCamera.createCaptureRequest(mCamera.TEMPLATE_PREVIEW);
                            requestBuilder.addTarget(holder.getSurface());
                            CaptureRequest previewRequest = requestBuilder.build();
                            // Start displaying preview images
                            try {
                                System.out.println("Inside statecallback try2");
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

/*  Callback      :  CameraDevice.StateCallback mCameraStateCallback

    Calledbacks invoked upon state changes in our  CameraDevice. These are run on
     using mBackgroundThread}>
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

    /*  Callback      :  ImageReader.OnImageAvailableListener mImageCaptureListener

  Converts the captured image into a Mat and sends it to the nativePoseEstimation.
  Call nativePoseEstimation() function,gets rotation and translation matrix in PNPRansac object.
*/

    final ImageReader.OnImageAvailableListener mImageCaptureListener =
            new ImageReader.OnImageAvailableListener() {

                @Override
                public void onImageAvailable(ImageReader reader) {
                    Image capture = reader.acquireNextImage();
                    System.out.println("**************** Captured Image ***************************");
                    Mat yuvMat = new Mat(capture.getHeight() + capture.getHeight() / 2, capture.getWidth(), CV_8UC1);
                    Mat rgbMat = new Mat(capture.getHeight(), capture.getHeight(), CvType.CV_8UC3);

                    ByteBuffer Y = capture.getPlanes()[0].getBuffer();


                    int Yb = Y.remaining();

                    System.out.println("Captured image size: " + capture.getWidth() + 'x' + capture.getHeight());
                    byte[] data = new byte[Yb];
                    Y.get(data, 0, Yb);

                    System.out.println("RGB Mat data before"+rgbMat);
                    System.out.println("YUV Mat data before"+yuvMat);
                    yuvMat.put(0, 0, data);
                    System.out.println("sssssssssssssssssssssssssssssss");
                    Imgproc.cvtColor(yuvMat, rgbMat, Imgproc.COLOR_YUV420p2GRAY, 3);

                    yuvMat.release();

                    Log.i(TAG,"-------------------------------------------");
                    System.out.println("RGB Mat data after"+rgbMat);
                    System.out.println("YUV Mat data after"+yuvMat);
                    //                    System.out.println(rgbMat);

                    Log.i(TAG,"-------------------------------------------");
                    nativePoseLister(rgbMat);
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
                                Thread.sleep(200);
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
    /*  Method      : getRefImageParams
        Arguments   : None
        Returns     : void

        This Method calculate the keypoints and descriptors of the reference image and stores it as a class variable.

    */
    public void getRefImageParams() {
        Log.d(null, "Inside startImageProcesing: ");
        System.out.println("Inside startImageProcessing");
        Utils.bitmapToMat(ref_img_bitmap, mat1);
        try {
            initializeOpenCVInstances(mat1);

        } catch (IOException e) {
            e.printStackTrace();
        }
        outImg=Bitmap.createBitmap(mat1.cols(), mat1.rows(), Bitmap.Config.ARGB_8888);
        Mat release = new Mat();
        release.convertTo(release, 0);
        Mat rgb = new Mat();
        Scalar color = new Scalar(0, 255, 0); // BGR
        System.out.println("Mat1 type as received:" + mat1.type());
        Imgproc.cvtColor(mat1, mat1, Imgproc.COLOR_RGBA2GRAY);
        System.out.println("Mat1 type afterRGBA2GRAY:" + mat1.type());
        Features2d.drawKeypoints(mat1, keypoints1, release);
        System.out.println("release mat type is:" + release.type());
        Utils.matToBitmap(release, outImg);
        System.out.println("Width is:" + outImg.getWidth() + " and height is:" + outImg.getHeight());
        mat1.convertTo(mat1, 0);// mat1 in camera type i.e. 0 format
//        nativePoseEstimation();

    }
/*  Method      : nativePoseLister
    Arguments   : Mat of scene image
    Returns     : void

    This method is called by onImageAvailable, receives Mat as the arguement of the scene image.
    Calculates keypoints and descriptors of the scene image.
    Draws matches between reference and scene image using FLANNBASED matching for fast similarity detection.
    Maintains the list of all the good matches as a list.
    Extracts 2D points from the scene and 3D from reference image.
    Using the extracted points calculates the rotation and translation matrix and stores it in the PNPansac class

*/

    public void nativePoseLister(Mat mat2) {

        mat2.convertTo(mat2, 0);

        descriptors2 = new Mat();
        keypoints2 = new MatOfKeyPoint();
        detector.detect(mat2, keypoints2);
        List<KeyPoint> keypoint_scene_img_list=keypoints2.toList();
        detector.compute(mat2, keypoints2, descriptors2);
        if(descriptors1.type()!=CV_32F) {
            descriptors1.convertTo(descriptors1, CV_32F);
        }

        if(descriptors2.type()!=CV_32F) {
            descriptors2.convertTo(descriptors2, CV_32F);
        }
        System.out.println("descriptor 2:" + descriptors2);
        List<MatOfDMatch> knnMatches;
//        MatOfDMatch matches = new MatOfDMatch();
//        if (mat1.type() == mat2.type()) {
        System.out.println("Here here !!");
        DescriptorMatcher matcher = DescriptorMatcher.create(DescriptorMatcher.FLANNBASED);
        knnMatches = new ArrayList<>();
        matcher.knnMatch(descriptors1, descriptors2, knnMatches, 2);
        float ratioThresh = 0.7f;
        List<DMatch> listOfGoodMatches = new ArrayList<>();
        for (int i = 0; i < knnMatches.size(); i++) {
            if (knnMatches.get(i).rows() > 1) {
                DMatch[] matches = knnMatches.get(i).toArray();
                if (matches[0].distance < ratioThresh * matches[1].distance) {
                    listOfGoodMatches.add(matches[0]);
                }
            }
        }
        MatOfDMatch goodMatches = new MatOfDMatch();
        goodMatches.fromList(listOfGoodMatches);
        Mat outputImg = new Mat();
        MatOfByte drawnMatches = new MatOfByte();
        Scalar color1 = new Scalar(0, 0, 255); // BGR
        Scalar color2 = new Scalar(0, 255, 0); // BGR
        System.out.println("Here");
//        System.out.println(good_matches);
        Features2d.drawMatches(mat1, keypoints1, mat2, keypoints2, goodMatches, outputImg, color2, color1, drawnMatches, Features2d.NOT_DRAW_SINGLE_POINTS);
        Imgproc.resize(outputImg, outputImg, mat2.size());

        System.out.println("Here Here Second Image!!");

        List<Point> list_points2d_scene_match = new ArrayList<Point>();
        List<Point3> list_points3d_model_match = new ArrayList<Point3>();

        System.out.println(keypoint_ref_img_list);
        System.out.println(keypoint_scene_img_list);
        for (int match_index = 0; match_index < listOfGoodMatches.size(); ++match_index) {
            Point point2d_ref = keypoint_ref_img_list.get(listOfGoodMatches.get(match_index).queryIdx).pt;        // 2D point from the scene
            Point point2d_frame = keypoint_scene_img_list.get(listOfGoodMatches.get(match_index).trainIdx).pt;    // 3D point from ref image part 1
            list_points3d_model_match.add(new Point3(point2d_ref.x, point2d_ref.y, 0.0f));            // 2D point from the scene
            list_points2d_scene_match.add(point2d_frame);                                            // 3D point from ref image part 1                                      // add 2D point
        }
        MatOfPoint2f points2d_scene_match=new MatOfPoint2f();
        MatOfPoint3f points3d_model_match=new MatOfPoint3f();
        Mat inliers_idx=new Mat();
        Mat rvec=new Mat();
        Mat tvec=new Mat();
        int flags;
        points3d_model_match.fromList(list_points3d_model_match);
        points2d_scene_match.fromList(list_points2d_scene_match);
        List<Point> list_points2d_inliers=new ArrayList<Point>();
        double f = 55;                           // focal length in mm
        double sx = 22.3, sy = 14.9;             // sensor size
        double width = 640, height = 480;        // image size
        double params_WEBCAM[] = {width * f / sx,   // fx
                height * f / sy,  // fy
                width / 2,      // cx
                height / 2};    // cy
        PNPRansac pnp_ref = new PNPRansac(params_WEBCAM); // instantiate PnPProblem class
        if (listOfGoodMatches.size()>0) {
            pnp_ref.estimatePoseRANSAC(points3d_model_match, points2d_scene_match, CV_ITERATIVE, inliers_idx);
        }
        System.out.println("Inside calling");
        System.out.println(pnp_ref.R_MAT.dump());
        System.out.println(pnp_ref.T_MAT.dump());

        System.out.println("Reached End !!");
    }
    /*  Method      : initialize opencv instances
        Arguments   : Mat
        Returns     :

        Just a helper function to instantiate and calculate reference image Mat and features

    */
    public void initializeOpenCVInstances(Mat mat) throws IOException {
        Log.d(null, "Inside initializeOpenCVInstances: ");
        System.out.println("Inside initializeOpenCVInstances");
        detector=ORB.create(10000);
        keypoints1 = new MatOfKeyPoint();
        descriptors1=new Mat();
        detector.detect(mat,keypoints1);
        detector.compute(mat,keypoints1,descriptors1);
        List<KeyPoint> referenceKeypointsList =
                keypoints1.toList();
        System.out.println(referenceKeypointsList);
    }

    /*  Method      : Draw
        Arguments   :
        Returns     : void

        Method to draw on the the Camera surface view, we have not implemented the rotation and translation based marker pose detection hence we are just drawing a rectnagle on the camera screen.

    */
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
    /*
        Private Class

        provides implemention to calculate the PNPRansac on the good matchers.

     */
    private class PNPRansac {
        Mat A_MAT, R_MAT, T_MAT, P_MAT;
        int iterationsCount = 500;        // number of Ransac iterations.
        float reprojectionError = 2.0f;    // maximum allowed distance to consider it an inlier.
        float confidence = 0.95f;          // RANSAC successful confidence.

        PNPRansac(double params[]) {
            A_MAT = zeros(3, 3, CV_64FC1);
            A_MAT.put(0, 0, params[0]);
            A_MAT.put(1, 1, params[1]);
            A_MAT.put(0, 2, params[2]);
            A_MAT.put(1, 2, params[3]);
            A_MAT.put(2, 2, 1);
            R_MAT = zeros(3, 3, CV_64FC1);   // rotation matrix
            T_MAT = zeros(3, 1, CV_64FC1);   // translation matrix
            P_MAT = zeros(3, 4, CV_64FC1);   // rotation-translation matrix
        }


        public void estimatePoseRANSAC(MatOfPoint3f list_points3d, MatOfPoint2f list_points2d, int flags, Mat inliers) {
            Mat dist = zeros(4, 1, CV_64FC1);    // vector of distortion coefficients
            MatOfDouble distCoeffs= new MatOfDouble(dist);
            Mat rvec = zeros(3, 1, CV_64FC1);          // output rotation vector
            Mat tvec = zeros(3, 1, CV_64FC1);          // output translation vector

            boolean useExtrinsicGuess = false;   // if true the function uses the provided rvec and tvec values as
            // initial approximations of the rotation and translation vectors
            System.out.println("Inside PNP ransac function");
            solvePnPRansac(list_points3d, list_points2d, A_MAT, distCoeffs, rvec, tvec,
                    useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                    inliers, flags);
            Rodrigues(rvec,R_MAT);                   // converts Rotation Vector to Matrix
            T_MAT = tvec;                            // set translation matrix

        }
    }

}
