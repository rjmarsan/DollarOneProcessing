/**
 * The $1 Unistroke Recognizer (Processing version)
 *
 *  RJ Marsan
 *  Based on $1 Unistroke Recognizer by:
 *
 *  Jacob O. Wobbrock, Ph.D.
 *   The Information School
 *  University of Washington
 *  Seattle, WA 98195-2840
 *  wobbrock@uw.edu
 *
 *  Andrew D. Wilson, Ph.D.
 *  Microsoft Research
 *  One Microsoft Way
 *  Redmond, WA 98052
 *  awilson@microsoft.com
 *
 *  Yang Li, Ph.D.
 *  Department of Computer Science and Engineering
 *   University of Washington
 *  Seattle, WA 98195-2840
 *   yangli@cs.washington.edu
 *
 * The academic publication for the $1 recognizer, and what should be 
 * used to cite it, is:
 *
 *  Wobbrock, J.O., Wilson, A.D. and Li, Y. (2007). Gestures without 
 *    libraries, toolkits or training: A $1 recognizer for user interface 
 *    prototypes. Proceedings of the ACM Symposium on User Interface 
 *    Software and Technology (UIST '07). Newport, Rhode Island (October 
 *    7-10, 2007). New York: ACM Press, pp. 159-168.
 *
 * The Protractor enhancement was separately published by Yang Li and programmed 
 * here by Jacob O. Wobbrock:
 *
 *  Li, Y. (2010). Protractor: A fast and accurate gesture
 *    recognizer. Proceedings of the ACM Conference on Human
 *    Factors in Computing Systems (CHI '10). Atlanta, Georgia
 *    (April 10-15, 2010). New York: ACM Press, pp. 2169-2172.
 *
 * This software is distributed under the "New BSD License" agreement:
 *
 * Copyright (C) 2007-2012, Jacob O. Wobbrock, Andrew D. Wilson and Yang Li.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the names of the University of Washington nor Microsoft,
 *      nor the names of its contributors may be used to endorse or promote
 *      products derived from this software without specific prior written
 *      permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Jacob O. Wobbrock OR Andrew D. Wilson
 * OR Yang Li BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/
 
//
// DollarRecognizer class constants
//
int NumUnistrokes = 16;
int NumPoints = 64;
float SquareSize = 250.0;
Point Origin = new Point(0, 0);
float Diagonal = sqrt(SquareSize * SquareSize + SquareSize * SquareSize);
float HalfDiagonal = 0.5 * Diagonal;
float AngleRange = Deg2Rad(45.0);
float AnglePrecision = Deg2Rad(2.0);
float Phi = 0.5 * (-1.0 + sqrt(5.0)); // Golden Ratio
 
 
//
// Unistroke class: a unistroke template
//
class Unistroke {
  public String Name;
  public Point[] Points;
  public float[] Vector;

  public Unistroke(String name, Point[] points) {
    this.Name = name;
    this.Points = Resample(points, NumPoints);
    float radians = IndicativeAngle(this.Points);
    this.Points = RotateBy(this.Points, -radians);
    this.Points = ScaleTo(this.Points, SquareSize);
    this.Points = TranslateTo(this.Points, Origin);
    this.Vector = Vectorize(this.Points); // for Protractor
  }
}

//
// Result class
//
class Result {
  String name;
  float score;

  public Result(String name, float score) {
    this.name = name;
    this.score = score;
  }
}


//
// DollarRecognizer class
//
class DollarRecognizer { 
  ArrayList<Unistroke> Unistrokes = new ArrayList<Unistroke>();

  public DollarRecognizer() {
    //
    // one built-in unistroke per gesture type
    //
  }

  //
  // The $1 Gesture Recognizer API begins here -- 3 methods: Recognize(), AddGesture(), and DeleteUserGestures()
  //
  public Result recognize(ArrayList<Point> points, boolean useProtractor) {
    return recognize(points.toArray(new Point[0]), useProtractor);
  }
  public Result recognize(Point[] points, boolean useProtractor) {
    points = Resample(points, NumPoints);
    float radians = IndicativeAngle(points);
    points = RotateBy(points, -radians);
    points = ScaleTo(points, SquareSize);
    points = TranslateTo(points, Origin);
    float[] vector = Vectorize(points); // for Protractor

    float b = Float.MAX_VALUE;
    int u = -1;
    for (int i = 0; i < this.Unistrokes.size(); i++) // for each unistroke
    {
      float d;
      if (useProtractor) // for Protractor
        d = OptimalCosineDistance(this.Unistrokes.get(i).Vector, vector);
      else // Golden Section Search (original $1)
        d = DistanceAtBestAngle(points, this.Unistrokes.get(i), -AngleRange, +AngleRange, AnglePrecision);
      if (d < b) {
        b = d; // best (least) distance
        u = i; // unistroke
      }
    }
    return (u == -1) ? new Result("No match.", 0.0) : new Result(this.Unistrokes.get(u).Name, useProtractor ? 1.0 / b : 1.0 - b / HalfDiagonal);
  }
  
  public int addGesture(String name, float[] floats) {
    Point[] points = new Point[floats.length/2];
    for (int i=0; i<floats.length; i+=2) {
      points[i/2] = new Point(floats[i], floats[i+1]); 
    }
    return addGesture(name, points);
  }
  
  public int addGesture(String name, Point[] points) {
    this.Unistrokes.add(new Unistroke(name, points)); // append new unistroke
    int num = 0;
    for (int i = 0; i < this.Unistrokes.size(); i++) {
      if (this.Unistrokes.get(i).Name.equals(name))
        num++;
    }
    return num;
  }
  
  
  public int deleteUserGestures() {
//        this.Unistrokes.length = NumUnistrokes; // clear any beyond the original set
//        return NumUnistrokes;
    return -1; //NOT IMPLEMENTED YET
  }
}


//
// Private helper functions from this point down
//
Point[] Resample(Point[] pointsx, int n) {
  ArrayList<Point> points = new ArrayList<Point>(Arrays.asList(pointsx));
  float I = PathLength(pointsx) / (n - 1); // interval length
  float D = 0.0;
  ArrayList<Point> newpoints = new ArrayList();
  newpoints.add(points.get(0));
  for (int i = 1; i < points.size(); i++) {
    float d = Distance(points.get(i - 1), points.get(i));
    if ((D + d) >= I) {
      float qx = points.get(i - 1).X + ((I - D) / d) * (points.get(i).X - points.get(i - 1).X);
      float qy = points.get(i - 1).Y + ((I - D) / d) * (points.get(i).Y - points.get(i - 1).Y);
      Point q = new Point(qx, qy);
      newpoints.add(q); // append new point 'q'
      points.add(i, q); // insert 'q' at position i in points s.t. 'q' will be the next i
      D = 0.0;
    }
    else D += d;
  }
  if (newpoints.size() == n - 1) // somtimes we fall a rounding-error short of adding the last point, so add it if so
  newpoints.add(new Point(points.get(points.size() - 1).X, points.get(points.size() - 1).Y));
  return newpoints.toArray(new Point[0]);
}

float IndicativeAngle(Point[] points) {
  Point c = Centroid(points);
  return atan2(c.Y - points[0].Y, c.X - points[0].X);
}

Point[] RotateBy(Point[] points, float radians) {
  Point c = Centroid(points);
  float cos = cos(radians);
  float sin = sin(radians);
  ArrayList<Point> newpoints = new ArrayList<Point>();
  for (int i = 0; i < points.length; i++) {
    float qx = (points[i].X - c.X) * cos - (points[i].Y - c.Y) * sin + c.X;
    float qy = (points[i].X - c.X) * sin + (points[i].Y - c.Y) * cos + c.Y;
    newpoints.add(new Point(qx, qy));
  }
  return newpoints.toArray(new Point[0]);
}

Point[] ScaleTo(Point[] points, float size) { // non-uniform scale; assumes 2D gestures (i.e., no lines)
  Rectangle B = BoundingBox(points);
  ArrayList<Point> newpoints = new ArrayList<Point>();
  for (int i = 0; i < points.length; i++) {
    float qx = points[i].X * (size / B.Width);
    float qy = points[i].Y * (size / B.Height);
    newpoints.add(new Point(qx, qy));
  }
  return newpoints.toArray(new Point[0]);
}

Point[] TranslateTo(Point[] points, Point pt) { // translates points' centroid
  Point c = Centroid(points);
  ArrayList<Point> newpoints = new ArrayList<Point>();
  for (int i = 0; i < points.length; i++) {
    float qx = points[i].X + pt.X - c.X;
    float qy = points[i].Y + pt.Y - c.Y;
    newpoints.add(new Point(qx, qy));
  }
  return newpoints.toArray(new Point[0]);
}

float[] Vectorize(Point[] points) { // for Protractor
  float sum = 0.0;
  float[] vector = new float[points.length*2];
  for (int i = 0; i < points.length; i++) {
    vector[i*2+0] = points[i].X;
    vector[i*2+1] = points[i].Y;
    sum += points[i].X * points[i].X + points[i].Y * points[i].Y;
  }
  float magnitude = sqrt(sum);
  for (int i = 0; i < vector.length; i++)
    vector[i] /= magnitude;
  return vector;
}

float OptimalCosineDistance(float[] v1, float[] v2) { // for Protractor
  float a = 0.0;
  float b = 0.0;
  for (int i = 0; i < v1.length; i += 2) {
    a += v1[i] * v2[i] + v1[i + 1] * v2[i + 1];
    b += v1[i] * v2[i + 1] - v1[i + 1] * v2[i];
  }
  float angle = atan(b / a);
  return acos(a * cos(angle) + b * sin(angle));
}

float DistanceAtBestAngle(Point[] points, Unistroke T, float a, float b, float threshold)
{
  float x1 = Phi * a + (1.0 - Phi) * b;
  float f1 = DistanceAtAngle(points, T, x1);
  float x2 = (1.0 - Phi) * a + Phi * b;
  float f2 = DistanceAtAngle(points, T, x2);
  while (abs (b - a) > threshold)  {
    if (f1 < f2) {
      b = x2;
      x2 = x1;
      f2 = f1;
      x1 = Phi * a + (1.0 - Phi) * b;
      f1 = DistanceAtAngle(points, T, x1);
    } else {
      a = x1;
      x1 = x2;
      f1 = f2;
      x2 = (1.0 - Phi) * a + Phi * b;
      f2 = DistanceAtAngle(points, T, x2);
    }
  }
  return min(f1, f2);
}
float DistanceAtAngle(Point[] points, Unistroke T, float radians) {
  Point[] newpoints = RotateBy(points, radians);
  return PathDistance(newpoints, T.Points);
}
Point Centroid(Point[] points) {
  float x = 0.0, y = 0.0;
  for (int i = 0; i < points.length; i++) {
    x += points[i].X;
    y += points[i].Y;
  }
  x /= points.length;
  y /= points.length;
  return new Point(x, y);
}
Rectangle BoundingBox(Point[] points) {
  float minX = Float.MAX_VALUE, maxX = Float.MIN_VALUE, minY = Float.MAX_VALUE, maxY = Float.MIN_VALUE;
  for (int i = 0; i < points.length; i++) {
    minX = min(minX, points[i].X);
    minY = min(minY, points[i].Y);
    maxX = max(maxX, points[i].X);
    maxY = max(maxY, points[i].Y);
  }
  return new Rectangle(minX, minY, maxX - minX, maxY - minY);
}
float PathDistance(Point[] pts1, Point[] pts2) {
  float d = 0.0;
  for (int i = 0; i < pts1.length; i++) // assumes pts1.length == pts2.length
    d += Distance(pts1[i], pts2[i]);
  return d / pts1.length;
}
float PathLength(Point[] points) {
  float d = 0.0;
  for (int i = 1; i < points.length; i++)
    d += Distance(points[i - 1], points[i]);
  return d;
}
float Distance(Point p1, Point p2) {
  float dx = p2.X - p1.X;
  float dy = p2.Y - p1.Y;
  return sqrt(dx * dx + dy * dy);
}

float Deg2Rad(float d) { 
  return (d * PI / 180.0);
}



//
// Point class
//
class Point {
  public float X;
  public float Y;
  public Point(float x, float y) {
    this.X = x;
    this.Y = y;
  }
}

//
// Rectangle class
//
class Rectangle {
  public float X;
  public float Y;
  public float Width;
  public float Height;

  public Rectangle (float x, float y, float width, float height) {
    this.X = x;
    this.Y = y;
    this.Width = width;
    this.Height = height;
  }
}



