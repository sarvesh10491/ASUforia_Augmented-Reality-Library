package com.example.nanda.open_cv_test_eee508;


import android.graphics.Bitmap;
import android.graphics.Color;
import android.util.Log;
import android.widget.ImageView;

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
import org.opencv.highgui.Highgui;


import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

import static android.graphics.Color.GREEN;
import static android.hardware.camera2.params.RggbChannelVector.RED;


public class ASUforia {
    static Bitmap img2,outImg;
    static Mat img1;
    private static Mat mat1 = new Mat();
//    FeatureDetector detector;
//    DescriptorExtractor descriptor;
    static ORB detector;
//    static DescriptorMatcher matcher;
    static Mat descriptors2,descriptors1;
    static MatOfKeyPoint keypoints1,keypoints2;

    ASUforia(Mat lena_img) {
        img1=lena_img;
//        img2=lena_img2;
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
