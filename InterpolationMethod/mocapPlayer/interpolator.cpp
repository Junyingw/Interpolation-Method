#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include <iostream>
#include <cmath>

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

// matrix multiply
double** Interpolator::matrixMultiply( double Mat1[3][3], double Mat2[3][3])
{
  double** resMat = new double*[3];
  // define 3*3 matrix
  for(int t = 0; t < 3; t++)
  {
    resMat[t] = new double[3];
  }
  // matrix multiple calculation 
  for(int i = 0; i < 3; i ++)
  {
    for(int j = 0; j < 3; j++)
    {
      resMat[i][j]= Mat1[i][0] * Mat2[0][j] + Mat1[i][1] * Mat2[1][j] + Mat1[i][2] * Mat2[2][j];
    }
  }
  return resMat;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // change degree to randian 
  double theta1 = (angles[0]/180) * M_PI;
  double theta2 = (angles[1]/180) * M_PI;
  double theta3 = (angles[2]/180) * M_PI;
  double transMat[3][3];

  // define 3*3 matrix
  double** temptMat = new double*[3];
  double** resMat = new double*[3];
  for(int t = 0; t < 3; t++)
  {
		temptMat[t] = new double[3];
    	resMat[t] = new double[3];
  }

  // initialize rotation matrix 
  double rotMat1[3][3] = {
    {1, 0, 0},
    {0, cos(theta1), -sin(theta1)},
    {0, sin(theta1), cos(theta1)}
  };

  double rotMat2[3][3] = {
    {cos(theta2), 0, sin(theta2)},
    {0, 1, 0},
    {-sin(theta2), 0, cos(theta2)}
  };

  double rotMat3[3][3] = {
    {cos(theta3), -sin(theta3), 0},
    {sin(theta3), cos(theta3), 0},
    {0, 0, 1}
  };
  
  temptMat = matrixMultiply(rotMat3, rotMat2);
  // transfer double ** temptMat to double[3][3] transMat
  for(int i = 0; i < 3; i++)
  {
		for(int j =  0; j < 3; j++)
		{
			transMat[i][j] = temptMat[i][j];
		}
  }
  resMat = matrixMultiply(transMat, rotMat1);

  // write final matrix values to R array
  for(int i = 0; i < 3; i++)
  {
	  for(int j =  0; j < 3; j++)
	  {
		  R[j + 3 * i] = resMat[i][j];
	  }
  }
}


void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	int startKeyframe = 0;
	
	// define control point
	vector p0, p1, p2, p3;
	vector a, b;

	while(startKeyframe + N + 1 < inputLength)
	{
		int endKeyframe = startKeyframe + N + 1;
		int preKeyframe = startKeyframe - N - 1;
		int nextKeyframe = endKeyframe + N + 1;

		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
		Posture * prePosture;
		Posture * nextPosture;

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for(int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			p1 = startPosture->root_pos;
			p2 = endPosture->root_pos;

			// interpolate root position
			// linear euler for the root 
			// if the start key frame is the first frame 
			if(startKeyframe == 0) 
			{
				nextPosture = pInputMotion->GetPosture(nextKeyframe);
				p3 = nextPosture->root_pos;
				a = p1 + ((p2 + p2 - p3) - p1) / 3.0;
				b = p2 - (((p2 + p2 - p1) + p3) * 0.5 - p2) / 3.0;
			}

			if(startKeyframe != 0)
			{
				prePosture = pInputMotion->GetPosture(preKeyframe);
				p0 = prePosture->root_pos;

				// if the next key frame is the last frame
				if(nextKeyframe >= inputLength-1)
				{
					a = p1 + (((p1 + p1 - p0) + p2) * 0.5 - p1) / 3.0;
					b = p2 + ((p1 + p1 - p0) - p2) / 3.0;
				}
				// if next key frame is the normal frame
				if(nextKeyframe < inputLength-1)
				{
					nextPosture = pInputMotion->GetPosture(nextKeyframe);
					p3 = nextPosture->root_pos;
					a = p1 + (((p1 + p1 - p0) + p2) * 0.5 - p1) / 3.0;
					b = p2 - (((p2 + p2 - p1) + p3) * 0.5 - p2) / 3.0;
				}
				
			}
			// implement DeCasteljau construction to evaluate a spline at any t
			interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a, b, p2);

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				p1 = startPosture->bone_rotation[bone];
				p2 = endPosture->bone_rotation[bone];

				// if the start key frame is the first frame 
				if(startKeyframe == 0) 
				{
					p3 = nextPosture->bone_rotation[bone];
					a = p1 + ((p2 + p2 - p3) - p1) / 3.0;
					b = p2 - (((p2 + p2 - p1) + p3) * 0.5 - p2) / 3.0;
				}

				if (startKeyframe != 0)
				{
					p0 = prePosture->bone_rotation[bone];

					// if the next key frame is the last frame
					if(nextKeyframe >= inputLength-1)
					{
						a = p1 + (((p1 + p1 - p0) + p2) / 2 - p1) / 3;
						b = p2 + ((p1 + p1 - p0) - p2) / 3;
					}
					// if next key frame is the normal frame
					if (nextKeyframe < inputLength-1)
					{
						p3 = nextPosture->bone_rotation[bone];
						a = p1 + (((p1 + p1 - p0) + p2) * 0.5 - p1) / 3.0;
						b = p2 - (((p2 + p2 - p1) + p3) * 0.5 - p2) / 3.0;
					}	
				}
				
				// implement DeCasteljau construction to evaluate a spline at any t
				interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, p1, a, b, p2);
			}

			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}
		startKeyframe = endKeyframe;
	}
	for (int frame = startKeyframe + 1; frame<inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}


//using SLERP
void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
	int startKeyframe = 0;
  
	while (startKeyframe + N + 1 < inputLength)
	{
		int endKeyframe = startKeyframe + N + 1;

		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			// define the start and end of oritation on every bone
			Quaternion<double> boneStart, boneEnd, boneInterpolate;

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				Euler2Quaternion(startPosture->bone_rotation[bone].p, boneStart);
				Euler2Quaternion(endPosture->bone_rotation[bone].p, boneEnd);

				boneInterpolate = Slerp(t, boneStart, boneEnd);

				Quaternion2Euler(boneInterpolate, interpolatedPosture.bone_rotation[bone].p);
			}
			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}
		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
	{
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
	}
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
    
    int startKeyframe = 0;
    
	  // define the control points and key points 
    vector p0, p1, p2, p3;
    vector a, b;
    
    // key points used for bone rotation
    Quaternion<double> pp0;
    Quaternion<double> pp1;
    Quaternion<double> pp2;
    Quaternion<double> pp3;

    // control points used for bone rotation
    Quaternion<double> ra;
    Quaternion<double> rb;

    
    while (startKeyframe + N + 1 < inputLength)
    {
      int endKeyframe = startKeyframe + N + 1;
      int preKeyframe = startKeyframe - N - 1;
      int nextKeyframe = endKeyframe + N + 1;
        
      Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
      Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
      Posture * prePosture = NULL;
      Posture * nextPosture = NULL;
        
      // copy start and end keyframe
      pOutputMotion->SetPosture(startKeyframe, *startPosture);
      pOutputMotion->SetPosture(endKeyframe, *endPosture);
        
      // interpolate in between
      for (int frame = 1; frame <= N; frame++)
      {
        Posture interpolatedPosture;
        double t = 1.0 * frame / (N + 1);
            
        p1 = startPosture->root_pos;
        p2 = endPosture->root_pos;

			  // interpolate root position
        // if the start frame is the first frame 
        if(startKeyframe == 0)      
        {
          // the first frame only has next posture
          nextPosture = pInputMotion->GetPosture(nextKeyframe);
          p3 = nextPosture->root_pos;
          
          // set the control point a, b
          a = p1 + ((p2 + p2 - p3) - p1) / 3.0;
          b = p2 - (((p2 + p2 - p1) + p3) * 0.5 - p2) / 3.0;
        }

        // if the frame is the normal frame
        if(startKeyframe != 0)
        {
          prePosture = pInputMotion->GetPosture(preKeyframe);
          p0 = prePosture->root_pos;
				  // if next key frame is the normal frame
          if (nextKeyframe < inputLength-1)  
          {
            nextPosture = pInputMotion->GetPosture(nextKeyframe); 
            
            p3 = nextPosture->root_pos;
            a = p1 + (((p1 + p1 - p0) + p2) * 0.5 - p1) / 3.0;
            b = p2 - (((p2 + p2 - p1) + p3) * 0.5 - p2) / 3.0;
          }  
          
          // if next key frame is the last frame
          if(nextKeyframe >= inputLength-1)
          {
            a = p1 + (((p1 + p1 - p0) + p2) * 0.5 - p1) / 3.0;
            b = p2 + ((p1 + p1 - p0) - p2) / 3.0;
          }

        }
			  // implement DeCasteljau construction to evaluate a spline at any t
        interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a, b, p2);
          
        // interpolate bone rotations
       for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        {

          Euler2Quaternion(startPosture->bone_rotation[bone].p, pp1);
          Euler2Quaternion(endPosture->bone_rotation[bone].p, pp2);
            
				  // if the start frame is the first frame 
          if(startKeyframe == 0)
          {
            Euler2Quaternion(nextPosture->bone_rotation[bone].p, pp3);

            // temporary variables used to calculate control points
					  Quaternion<double> tempt0 = Double(pp3, pp2);
					  Quaternion<double> tempt1 = Double(pp1, pp2);
					  Quaternion<double> tempt2 = Slerp(0.5, tempt1, pp3);

            ra = Slerp((1.0 / 3.0), pp1, tempt0);
            rb = Slerp((-1.0 / 3.0), pp2, tempt2);
          }
          // if the start frame is the normal frame
          if(startKeyframe != 0)
          {
            Euler2Quaternion(prePosture->bone_rotation[bone].p, pp0);

					  // if next key frame is the last frame 
            if(nextKeyframe >= inputLength-1)   
            {
              // temporary variables used to calculate control points
						  Quaternion<double> tempt0 = Double(pp0, pp1);
						  Quaternion<double> tempt1 = Slerp(0.5, tempt0, pp2);
						
              ra = Slerp((1.0 / 3.0), pp1, tempt1);
              rb = Slerp((1.0 / 3.0), pp2, tempt0);
            }
					  // if next key frame is the normal frame
            if(nextKeyframe < inputLength-1)   
            {
              Euler2Quaternion(nextPosture->bone_rotation[bone].p, pp3);
              
              // temporary variables used to calculate control points
						  Quaternion<double> tempt0 = Double(pp0, pp1);
						  Quaternion<double> tempt1 = Double(pp1, pp2);
						  Quaternion<double> tempt2 = Slerp(0.5, tempt0, pp2);
						  Quaternion<double> tempt3 = Slerp(0.5, tempt1, pp3);
					
              ra = Slerp((1.0 / 3.0), pp1, tempt2);
              rb = Slerp((-1.0 / 3.0), pp2, tempt3);
             }
          }
				  // implement DeCasteljau construction to evaluate a spline at any t
				  Quaternion<double> boneInterpolation = DeCasteljauQuaternion(t, pp1, ra, rb, pp2);
          Quaternion2Euler(boneInterpolation, interpolatedPosture.bone_rotation[bone].p);
        }    
        pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
      }

      startKeyframe = endKeyframe;
    }
    
    for(int frame = startKeyframe + 1; frame<inputLength; frame++)
      pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}


void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
	double R[9]; 
  // Euler angles to rotation matrix
	Euler2Rotation(angles, R);
	// rotation matrix to quaternion
	q = Quaternion<double>::Matrix2Quaternion(R);
	q.Normalize();
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  double R[9];
  // quaternion to matirx
  q.Quaternion2Matrix(R);
  // matrix to Euler
  Rotation2Euler(R, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd)
{  
  Quaternion<double> result; 
  double costheta, sintheta, angle;

  // pick the shortest path
  costheta = qStart.Gets() * qEnd.Gets() + qStart.Getx() * qEnd.Getx() + qStart.Gety() * qEnd.Gety() + qStart.Getz() * qEnd.Getz();
  if(costheta < 0)
  {
    costheta = abs(costheta);
    qEnd = -1*qEnd;
  }

  //in the interval [0,pi] radians.
  angle = acos(costheta);
  sintheta = sin(angle);

  // if the angle is 0, return qStart.
  if(fabs(sintheta) == 0.0)
	{
		result = qStart;
		return result;
	}

  // implement SLEPR formulation
  result = (sin((1 - t) * angle) / sin(angle)) * qStart + (sin(t * angle) / sin(angle)) * qEnd;
	result.Normalize();
	return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  Quaternion<double> result;

  result = Slerp(2.0, p, q);

  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // define control points
	vector result;
	vector q0, q1, q2;
	vector r0, r1;

	// implement DeCasteljau using euler angles
	q0 = p0 * (1 - t) + p1 * t;
	q1 = p1 * (1 - t) + p2 * t;
	q2 = p2 * (1 - t) + p3 * t;
	r0 = q0 * (1 - t) + q1 * t;
	r1 = q1 * (1 - t) + q2 * t;
	result = r0 * (1 - t) + r1 * t;

	return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // define control points
	Quaternion<double> result;
	Quaternion<double> q0, q1, q2;
	Quaternion<double> r0, r1;

	// implement DeCasteljau using quaternion
	q0 = Slerp(t, p0, p1);
	q1 = Slerp(t, p1, p2);
	q2 = Slerp(t, p2, p3);
	r0 = Slerp(t, q0, q1);
	r1 = Slerp(t, q1, q2);
	result = Slerp(t, r0, r1);

	return result;
}

