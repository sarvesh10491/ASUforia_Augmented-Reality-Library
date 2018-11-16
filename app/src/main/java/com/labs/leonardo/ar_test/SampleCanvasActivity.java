package com.labs.leonardo.ar_test;

import java.util.ArrayList;



import android.app.Activity;

import android.content.Context;

import android.graphics.Bitmap;

import android.graphics.Canvas;

import android.graphics.Color;

import android.graphics.Paint;

import android.graphics.Path;

import android.graphics.Rect;

import android.graphics.RectF;

import android.os.Bundle;

import android.view.MotionEvent;

import android.view.SurfaceHolder;

import android.view.SurfaceView;

import android.view.View;

import android.view.View.OnTouchListener;

import android.view.ViewGroup.LayoutParams;

import android.view.Window;

import android.view.WindowManager;

import android.widget.FrameLayout;



import android.content.Context;

import android.graphics.Canvas;

import android.graphics.Color;

import android.graphics.Paint;

import android.view.View;



public class SampleCanvasActivity extends View {

    Paint paint = new Paint();



    public SampleCanvasActivity(Context context) {

        super(context);

    }



    @Override

    public void onDraw(Canvas canvas) {

        paint.setColor(Color.BLACK);

        paint.setStrokeWidth(3);

        canvas.drawRect(130, 130, 180, 180, paint);

        paint.setStrokeWidth(0);

        paint.setColor(Color.CYAN);

        canvas.drawRect(133, 160, 177, 177, paint );

        paint.setColor(Color.YELLOW);

        canvas.drawRect(133, 133, 177, 160, paint );



    }

}
