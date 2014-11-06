#include "Kinect_Thread.h"

Mat Kinect_Thread::equalizeIntensity(const Mat& inputImage)
{
	if (inputImage.channels() >= 3)
	{
		Mat ycrcb;

		cvtColor(inputImage, ycrcb, CV_BGR2YCrCb);

		vector<Mat> channels;
		split(ycrcb, channels);

		equalizeHist(channels[0], channels[0]);

		Mat result;
		merge(channels, ycrcb);

		cvtColor(ycrcb, result, CV_YCrCb2RGB);
		


		return result;
	}
	return Mat();
}
float Kinect_Thread::d2r(float d)
{
	return d * M_PI / 180;
}
float Kinect_Thread::r2d(float r)
{
	return r * 180 / M_PI;
}

Kinect_Thread::Kinect_Thread(PipelineUtilities::PipelineGovernor &governor, ITarget<shared_ptr<Kinect_Data>> &target):
	m_Governor(governor),
	m_KinectData(target)
{
	InitializeDefaultSensor();
	m_pDepthRGBX=new RGBQUAD[cDepthWidth*cDepthHeight];
	depthImageBuffer=new BYTE[cDepthWidth*cDepthHeight];
	//m_CurrentFrame=Kinect_Data();
	//m_CurrentFrame.locations=vector<HeadLoc>();
	

	
	realPoints=new CameraSpacePoint[cDepthWidth*cDepthWidth];
	m_pColorCoordinates= new ColorSpacePoint[cDepthWidth*cDepthWidth];
	test=Mat(cDepthHeight,cDepthHeight,CV_8UC4);
	// create heap storage for composite image pixel data in RGBX format
	m_pOutputRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];

	// create heap storage for background image pixel data in RGBX format
	m_pBackgroundRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];

	// create heap storage for color pixel data in RGBX format
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
	colourScheme = COLORMAP_JET;
	// create heap storage for the coordinate mapping from depth to color
	m_pColorCoordinates = new ColorSpacePoint[cDepthWidth * cDepthHeight];
	m_ComputeHeadPose = false;
	m_continue = true;
	m_Colour_Properties = Colour_Display_Properties::Display_Colour;
	
	InitializeMLP("./mlp_tuned.json");
	
	p1=pow(-1.3835,-9);
	p2=pow(1.8435,-5);
	p3=-0.091403;
	p4=189.38;
	kappa = 8;
	record_data = false;
}



Kinect_Thread::~Kinect_Thread(void)
{
}

//////////////////////////////////////////////////////////////////////////
//REFACTOR FUNCTION AND DIVIDE WITH	ADDITIONAL PIPELINE STAGES
//////////////////////////////////////////////////////////////////////////
void Kinect_Thread::ProcessFrame(INT64 nTime, UINT16* pDepthBuffer, int nDepthHeight, int nDepthWidth, 
								 RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight, BYTE* pBodyIndexBuffer, 
								 int nBodyIndexWidth, int nBodyIndexHeight, int BodyCount,IBody** ppBodies, Vector4 floorPlane)

{
	shared_ptr<Kinect_Data> 						m_CurrentFrame(new Kinect_Data());

	if (m_pCoordinateMapper && m_pColorCoordinates && m_pOutputRGBX && 
		pDepthBuffer && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight) && 
		pColorBuffer && (nColorWidth == cColorWidth) && (nColorHeight == cColorHeight) &&
		pBodyIndexBuffer && (nBodyIndexWidth == cDepthWidth) && (nBodyIndexHeight == cDepthHeight))
	{
		HRESULT hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(nDepthWidth * nDepthHeight, (UINT16*)pDepthBuffer,nDepthWidth * nDepthHeight, m_pColorCoordinates); 
		
		m_CurrentFrame->new_cloud=pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>());

		m_CurrentFrame->new_cloud->width = cDepthWidth;
		m_CurrentFrame->new_cloud->height = cDepthHeight;
		m_CurrentFrame->new_cloud->resize(cDepthWidth*cDepthHeight);

		m_pCoordinateMapper->MapDepthFrameToCameraSpace(nLength,pDepthBuffer,nLength,realPoints);
		
		m_ColorImage=Mat(cColorHeight,cColorWidth,CV_8UC4,reinterpret_cast<BYTE *> ( pColorBuffer),Mat::AUTO_STEP);
		m_DepthImage = Mat(cDepthHeight, cDepthWidth, CV_16UC1, reinterpret_cast<BYTE *>(pDepthBuffer), Mat::AUTO_STEP);
		m_BodyImage = Mat(cDepthHeight, cDepthWidth, CV_8UC4, reinterpret_cast<BYTE *> (m_pOutputRGBX), Mat::AUTO_STEP);
		Mat depthCol, temp1;
		Mat colourSchemeMat;
		vector<BYTE> playerIDs=vector<BYTE>();
		test.release();
		test=Mat(cDepthHeight,cDepthWidth,CV_8UC4);

		m_CurrentFrame->num_heads = 0;
		m_CurrentFrame->locations.clear();
		heads.clear();

		//Process body frames in parallel
		for (int i = 0; i < BodyCount;i++)
		//parallel_for(0, BodyCount, [&](int i)
		{
			IBody* pBody = ppBodies[i];
			CameraSpacePoint pt;
			DepthSpacePoint dpt;
			

			if (pBody)
			{
				BOOLEAN bTracked = false;
				
				hr = pBody->get_IsTracked(&bTracked);

				if (SUCCEEDED(hr) && bTracked)
				{
					Joint joints[JointType_Count];

					
					hr = pBody->GetJoints(_countof(joints), joints);
					if (SUCCEEDED(hr))
					{
						ColorSpacePoint headPoint = { 0 };
						ColorSpacePoint neckPoint = { 0 };
						CameraSpacePoint hdPT;
						bool hed = false, nk = false;

						std::vector<JointLoc> body;

						for (int j = 0; j < _countof(joints); ++j)
						{
							//jointPoints[j] = BodyToScreen(joints[j].Position, width, height);
							DepthSpacePoint depthPoint = { 0 };
							m_pCoordinateMapper->MapCameraPointToDepthSpace(joints[j].Position, &depthPoint);
							dpt.X = floor(depthPoint.X);
							dpt.Y = floor(depthPoint.Y);
							HeadLoc hd = HeadLoc();
							
							JointLoc jl = JointLoc();
							m_pCoordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &headPoint);

							jl.Loc3D.x = joints[j].Position.X;
							jl.Loc3D.y = joints[j].Position.Y;
							jl.Loc3D.z = joints[j].Position.Z;
							if (joints[j].TrackingState == TrackingState_Tracked)
								jl.tracked = true;
							else
								jl.tracked = false;

							jl.Loc2D.X = headPoint.X;
							jl.Loc2D.Y = headPoint.Y;
							jl.type = joints[j].JointType;
							body.push_back(jl);


							if (joints[j].JointType == JointType::JointType_Head)
							{
								//HeadLocation hd = HeadLocation();
								m_pCoordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &headPoint);
								//hd.pos.x = headPoint.X;
								//hd.pos.y = headPoint.Y;
								if (headPoint.X != 0 && headPoint.Y != 0)
								{
									hed = true;
									//cv::circle(m_DrawnColorImage,cv::Point(headPoint.X,headPoint.Y),5,cv::Scalar(0,0,255),5);
								}

								hd.Loc.x = joints[j].Position.X;
								hd.Loc.y = joints[j].Position.Y;
								hd.Loc.z = joints[j].Position.Z;
								double depth = joints[j].Position.Z;
								double h = p1*pow(depth, 3) + p2*pow(depth, 2) + p3*depth + p4;
								hd.Radius = 1.33*h / 2000.0;
								heads.push_back(hd);

								cv::Point left = Point(depthPoint.X - hd.Radius, depthPoint.Y - hd.Radius);
								cv::Point right = Point(depthPoint.X + hd.Radius, depthPoint.Y + hd.Radius);
								//cv::circle(test,cv::Point(depthPoint.X,depthPoint.Y),hd.Radius,cv::Scalar(0,0,255),5);
								//cv::rectangle(test, left, right, Scalar(0, 0, 255), 4);
							}

							if (joints[j].JointType == JointType::JointType_Neck)
							{
								m_pCoordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &neckPoint);
								if (neckPoint.X != 0 && neckPoint.Y != 0)
								{
									nk = true;
									//cv::circle(m_DrawnColorImage,cv::Point(arbitPoint.X,arbitPoint.Y),5,cv::Scalar(0,0,255),5);
								}

								//cout<<arbitPoint.X<<","<<arbitPoint.Y<<",";
							}
							if (hed && nk 
								&& !isnan(headPoint.X) && !isinf(headPoint.X)
								&& !isnan(headPoint.Y) && !isinf(headPoint.Y)
								&& !isnan(neckPoint.X) && !isinf(neckPoint.X)
								&& !isnan(neckPoint.Y) && !isinf(neckPoint.Y))
							{
								if (m_ComputeHeadPose)
								{
									auto val1 = pow(headPoint.X - neckPoint.X, 2);
									auto val2 = pow(headPoint.Y - neckPoint.Y, 2);

									float distance = sqrt(val1 + val2);
									//cout << distance << endl;
									float radius = distance;
									//Get Bounding Box
									float leftX = (headPoint.X - radius);
									float leftY = (headPoint.Y - radius);
									float rightX = (headPoint.X + radius);
									float rightY = (headPoint.Y + radius);
									if (leftX < 0)
										leftX = 0;
									if (leftY < 0)
										leftY = 0;
									if (rightX >= 1920)
										rightX = 1919;
									if (rightY >= 1080)
										rightY = 1079;
									cv::Point left = Point(leftX, leftY);
									cv::Point right = Point(rightX, rightY);
									Rect crop = Rect(left, right);
									Mat tmp(m_ColorImage, crop);//m_ColorImage(crop).clone();
									Mat tmp1;
									//Create Image for processing
									resize(tmp, tmp, Size(32, 32), INTER_CUBIC);
									
									cvtColor(tmp, tmp1, CV_BGRA2BGR);
									Mat jt = equalizeIntensity(tmp1);
									//imshow("hd", jt);
									//cv::waitKey(5);
									jt.convertTo(tmp, CV_32F);
									transpose(tmp, tmp);
									Mat hdImage = tmp.clone();
									//Mat headImage
									normalize(hdImage, hdImage, 0, 1, NORM_MINMAX, CV_32FC3);
									//Allocate SSE aligned memory to feed through MLP
									float* visible_buffer = (float*) OMLT::AlignedMalloc(sizeof(float) * 3072, 16);
									float* hidden_buffer = (float*) OMLT::AlignedMalloc(sizeof(float) * 8, 16);
									//Copy data
									if (hdImage.isContinuous())
									{
										//copy(reinterpret_cast<float *>(hdImage.data))
										memcpy(visible_buffer, reinterpret_cast<float *>(hdImage.data), sizeof(float) * 3072);

									}
									else
									{

									}
									lockGuard.lock();
									model.mlp->FeedForward(visible_buffer, hidden_buffer);
									lockGuard.unlock();
									if (!jt.empty()) jt.release();
									if (!tmp1.empty()) tmp1.release();
									float temp = 0;
									int pos = 0;
									for (int k = 0; k<8; k++)
									{
										if (hidden_buffer[k]>temp)
										{
											hd.probabilities[k] = hidden_buffer[k];
											temp = hidden_buffer[k];
											pos = k;
										}

									}
									hd.HeadPose = pos * 45;
									hd.HeadPoseAvailable = true;
								}
								else
								{
									hd.HeadPose = -1;
									hd.HeadPoseAvailable = false;
								}
								m_CurrentFrame->locations.push_back(hd);
								m_CurrentFrame->num_heads++;
								m_CurrentFrame->floorPlane = floorPlane;
								hed = nk = false;
								BYTE indx = pBodyIndexBuffer[static_cast<int>(dpt.Y*nDepthWidth + dpt.X)];
								playerIDs.push_back(indx);
							}


						}

						m_CurrentFrame->bodies.push_back(body);
					}
				}
			}
		//});
		}

		/*for (int i = 0; i < BodyCount; ++i)
		{
			
		}*/
		
		if (SUCCEEDED(hr))
		{
			RGBQUAD c_green = {0, 255, 0}; 
			Colour_Display_Properties C_option; 
			lockGuard.lock();
			C_option = m_Colour_Properties;
			lockGuard.unlock();
			
			//Create Colour Scheme
			switch (C_option)
			{
			case Colour_Display_Properties::Display_Colour:
				//Fill image with mapped colour data
				colourSchemeMat=Mat(nDepthHeight, nDepthWidth, CV_8UC3);
				parallel_for(0, nDepthWidth * nDepthHeight, [&](int depthIndex)
				{
					int depthY = (int) (floor((double) depthIndex / (double) nDepthWidth));
					int depthX = depthIndex - (nDepthWidth*depthY);
					
					ColorSpacePoint colorPoint = m_pColorCoordinates[depthIndex];
					int colorX = (int) (floor(colorPoint.X + 0.5));
					int colorY = (int) (floor(colorPoint.Y + 0.5));
					if ((colorX >= 0) && (colorX < nColorWidth) && (colorY >= 0) && (colorY < nColorHeight) && (!m_ColorImage.empty()))
					{

						if ((depthX >= 0) && (depthX < nDepthWidth) && (depthY >= 0) && (depthY < nDepthHeight))
						{
							if (C_option == Colour_Display_Properties::Display_Colour)
							{
								colourSchemeMat.at<Vec3b>(depthY, depthX)[0] = m_ColorImage.at<Vec4b>(colorY, colorX)[0];
								colourSchemeMat.at<Vec3b>(depthY, depthX)[1] = m_ColorImage.at<Vec4b>(colorY, colorX)[1];
								colourSchemeMat.at<Vec3b>(depthY, depthX)[2] = m_ColorImage.at<Vec4b>(colorY, colorX)[2];
							}
						
						}
					}
				});
				break;

			case Colour_Display_Properties::Display_Depth:
				//Fill image with chosen colour scheme colour
				cv::normalize(m_DepthImage, temp1, 0, 255, CV_MINMAX);
				temp1.convertTo(depthCol, CV_8UC1);
				applyColorMap(depthCol, colourSchemeMat, colourScheme);
				break;

			case Colour_Display_Properties::Display_Attention:
			
				if (playerIDs.size()>0)
				{
					float* pixels = new float[nDepthWidth*nDepthHeight]();
					float kappa1=kappa;
					//for (int depthIndex = 0; depthIndex < nDepthWidth * nDepthHeight; depthIndex++)
					parallel_for(0, nDepthWidth * nDepthHeight, [&](int depthIndex)
					{
						int depthY = (int) (floor((double) depthIndex / (double) nDepthWidth));
						int depthX = depthIndex - (nDepthWidth*depthY);

						//Check every point and for every human and colour it and put in the corresponding position
						for (int person = 0; person < playerIDs.size(); person++)
						{
							if (pBodyIndexBuffer[depthIndex] != playerIDs[person])
							{
								//Translate the coordinate of every point so that the current head is at (0,0,0) 
								float X = realPoints[depthIndex].X - m_CurrentFrame->locations[person].Loc.x;
								float Y = realPoints[depthIndex].Y - m_CurrentFrame->locations[person].Loc.y;
								float Z = realPoints[depthIndex].Z - m_CurrentFrame->locations[person].Loc.z;
								float Norm = sqrt(X*X + Y*Y + Z*Z);
								X /= Norm;
								Y /= Norm;
								Z /= Norm;
								//Unit vector in the direction of gaze
								float X_G = cos(d2r(m_CurrentFrame->locations[person].HeadPose));
								float Y_G = 0;
								float Z_G = sin(d2r(m_CurrentFrame->locations[person].HeadPose));
								float dotProduct = X*X_G + Z*Z_G;

								float C;
								C = kappa1 / (4.0 * M_PI*sinh(kappa1));
								float val;
								val = C*exp(kappa1*dotProduct);
								if (isinf(val) || isnan(val))
								{
									val = 0;
								}
								pixels[depthIndex] += val;
								//Calculate Von Mises Fisher distribution for that point and change the value at the corresponding matrix

							}
						}

					});
					//}

					Mat accumulatedImage = Mat(nDepthHeight, nDepthWidth, CV_32FC1, reinterpret_cast<BYTE *>(pixels), Mat::AUTO_STEP);
					medianBlur(accumulatedImage, accumulatedImage, 5);
					//delete [] pixels;
					
					double min, max;
					cv::Point min_loc, max_loc;
					minMaxLoc(accumulatedImage, &min, &max,&min_loc,&max_loc);
					

					cout << min <<"," <<max << endl;
					//Normalize and apply Colourmap
					Mat temporaryImage1, temporaryImage2;
					cv::normalize(accumulatedImage, temporaryImage1, 0, 255, CV_MINMAX);
					
					temporaryImage1.convertTo(temporaryImage2, CV_8UC1);
					applyColorMap(temporaryImage2, colourSchemeMat, colourScheme);
					if (!temporaryImage1.empty()) temporaryImage1.release();
					if (!temporaryImage2.empty()) temporaryImage2.release();
					delete [] pixels;
				}
				else
				{
					//Paint white
					colourSchemeMat = Mat(nDepthHeight, nDepthWidth, CV_8UC3);
					parallel_for(0, nDepthWidth * nDepthHeight, [&](int depthIndex)
					{
						int depthY = (int) (floor((double) depthIndex / (double) nDepthWidth));
						int depthX = depthIndex - (nDepthWidth*depthY);


						if ((depthX >= 0) && (depthX < nDepthWidth) && (depthY >= 0) && (depthY < nDepthHeight))
						{
								
								colourSchemeMat.at<Vec3b>(depthY, depthX)[0] = 255;
								colourSchemeMat.at<Vec3b>(depthY, depthX)[1] = 255;
								colourSchemeMat.at<Vec3b>(depthY, depthX)[2] = 255;
								

						}
						
					});

				}
				break;

			case Colour_Display_Properties::Display_Custom:
				//colourSchemeMat = Mat(nDepthHeight, nDepthWidth, CV_8UC3);
				//Create Colour  Mat
				Mat partialColourMat = Mat(nDepthHeight, nDepthWidth, CV_8UC3);
				parallel_for(0, nDepthWidth * nDepthHeight, [&](int depthIndex)
				{
					int depthY = (int) (floor((double) depthIndex / (double) nDepthWidth));
					int depthX = depthIndex - (nDepthWidth*depthY);

					ColorSpacePoint colorPoint = m_pColorCoordinates[depthIndex];
					int colorX = (int) (floor(colorPoint.X + 0.5));
					int colorY = (int) (floor(colorPoint.Y + 0.5));
					if ((colorX >= 0) && (colorX < nColorWidth) && (colorY >= 0) && (colorY < nColorHeight) && (!m_ColorImage.empty()))
					{

						if ((depthX >= 0) && (depthX < nDepthWidth) && (depthY >= 0) && (depthY < nDepthHeight))
						{
							
							partialColourMat.at<Vec3b>(depthY, depthX)[0] = m_ColorImage.at<Vec4b>(colorY, colorX)[0];
							partialColourMat.at<Vec3b>(depthY, depthX)[1] = m_ColorImage.at<Vec4b>(colorY, colorX)[1];
							partialColourMat.at<Vec3b>(depthY, depthX)[2] = m_ColorImage.at<Vec4b>(colorY, colorX)[2];
						

						}
					}
				});

				//Create attention mat
				if (playerIDs.size() > 0)
				{
					//Leak
					float* pixels = new float[nDepthWidth*nDepthHeight]();
					float kappa1 = kappa;
					//for (int depthIndex = 0; depthIndex < nDepthWidth * nDepthHeight; depthIndex++)
					parallel_for(0, nDepthWidth * nDepthHeight, [&](int depthIndex)
					{
						int depthY = (int) (floor((double) depthIndex / (double) nDepthWidth));
						int depthX = depthIndex - (nDepthWidth*depthY);

						//Check every point and for every human and colour it and put in the corresponding position
						for (int person = 0; person < playerIDs.size(); person++)
						{
							if (pBodyIndexBuffer[depthIndex] != playerIDs[person])
							{
								//Translate the coordinate of every point so that the current head is at (0,0,0) 
								float X = realPoints[depthIndex].X - m_CurrentFrame->locations[person].Loc.x;
								float Y = realPoints[depthIndex].Y - m_CurrentFrame->locations[person].Loc.y;
								float Z = realPoints[depthIndex].Z - m_CurrentFrame->locations[person].Loc.z;
								float Norm = sqrt(X*X + Y*Y + Z*Z);
								X /= Norm;
								Y /= Norm;
								Z /= Norm;
								//Unit vector in the direction of gaze
								float X_G = cos(d2r(m_CurrentFrame->locations[person].HeadPose));
								float Y_G = 0;
								float Z_G = sin(d2r(m_CurrentFrame->locations[person].HeadPose));
								float dotProduct = X*X_G + Z*Z_G;

								float C;
								C = kappa1 / (4.0 * M_PI*sinh(kappa1));
								float val;
								val = C*exp(kappa1*dotProduct);
								if (isinf(val) || isnan(val))
								{
									val = 0;
								}
								pixels[depthIndex] += val;
								//Calculate Von Mises Fisher distribution for that point and change the value at the corresponding matrix

							}
						}

					});
					//}

					Mat accumulatedImage = Mat(nDepthHeight, nDepthWidth, CV_32FC1, reinterpret_cast<BYTE *>(pixels), Mat::AUTO_STEP);
					medianBlur(accumulatedImage, accumulatedImage, 5);
					//delete [] pixels;

					
					//Normalize and apply Colourmap
					Mat temporaryImage1, temporaryImage2, temporaryImage3;
					cv::normalize(accumulatedImage, temporaryImage1, 0, 255, CV_MINMAX);

					temporaryImage1.convertTo(temporaryImage2, CV_8UC1);
					applyColorMap(temporaryImage2, temporaryImage3, colourScheme);
					addWeighted(partialColourMat, 0.8, temporaryImage3, 0.2, 0.0, colourSchemeMat);
					if (!temporaryImage1.empty()) temporaryImage1.release();
					if (!temporaryImage2.empty()) temporaryImage2.release();
					if (!temporaryImage3.empty()) temporaryImage2.release();
					delete [] pixels;
				}
				else
				{
					colourSchemeMat = partialColourMat.clone();
				}
				
				if (!partialColourMat.empty()) partialColourMat.release();
				break;
			}
			

			parallel_for(0,nDepthWidth * nDepthHeight,[&](int depthIndex)
			{
				int depthY=(int)(floor((double)depthIndex / (double)nDepthWidth));
				int depthX=depthIndex-(nDepthWidth*depthY);
				PointXYZRGB point;
				point.x=0;
				point.y=0;
				point.z=0;
				point.r=0;
				point.g=0;
				point.b=0;
				m_CurrentFrame->new_cloud->at(depthX, depthY) = point;
				//m_Kinect2DepthCloud->at(depthX,depthY)=point;
				ColorSpacePoint colorPoint = m_pColorCoordinates[depthIndex];

				
				// make sure the depth pixel maps to a valid point in color space
				int colorX = (int)(floor(colorPoint.X + 0.5));
				int colorY = (int)(floor(colorPoint.Y + 0.5));
				if ((colorX >= 0) && (colorX < nColorWidth) && (colorY >= 0) && (colorY < nColorHeight)&& (!m_ColorImage.empty()))
				{

					// calculate index into color array
					//int colorIndex = colorX + (colorY * nColorWidth);


					if ((depthX >= 0) && (depthX < nDepthWidth) && (depthY >= 0) && (depthY < nDepthHeight))
					{
						//Choose colour scheme based on parameters
						BYTE B, G, R;
						
						B = colourSchemeMat.at<Vec3b>(depthY, depthX)[0];
						G = colourSchemeMat.at<Vec3b>(depthY, depthX)[1];
						R = colourSchemeMat.at<Vec3b>(depthY, depthX)[2];
						
						PointXYZRGB pt;
						pt.x=realPoints[depthIndex].X;
						pt.y=realPoints[depthIndex].Y;
						pt.z=realPoints[depthIndex].Z;
						pt.r = R;//R;
						pt.g = G;// G;
						pt.b = B;// B;
						m_CurrentFrame->new_cloud->at(depthX, depthY) = pt;

					}
				}

				
			});
			

		}

		m_Governor.WaitForAvailablePipelineSlot();
		asend(m_KinectData,m_CurrentFrame);
		emit Kinect_Frame_Available();
		//cout << "send" << endl;

		//Fork saving code

		//Release OpenCV matrices
		if (!m_ColorImage.empty())	m_ColorImage.release();
		if (!m_DepthImage.empty())	m_DepthImage.release();
		if (!m_BodyImage.empty())	m_BodyImage.release();
		if (!colourSchemeMat.empty()) colourSchemeMat.release();
		//Reset smart pointers
		//m_CurrentFrame->new_cloud.reset();
		//m_CurrentFrame.reset();
		
	}
	//m_Governor.WaitForAvailablePipelineSlot();
	
}

void Kinect_Thread::Update()
{
	if (!m_pMultiSourceFrameReader)
	{
		return;
	}

	IMultiSourceFrame* pMultiSourceFrame = NULL;
	IDepthFrame* pDepthFrame = NULL;
	IColorFrame* pColorFrame = NULL;
	IBodyIndexFrame* pBodyIndexFrame = NULL;
	IBodyFrame* pBodyFrame=NULL;
	hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

	if (SUCCEEDED(hr))
	{
		IDepthFrameReference* pDepthFrameReference = NULL;

		hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
		}

		SafeRelease(pDepthFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IColorFrameReference* pColorFrameReference = NULL;

		hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pColorFrameReference->AcquireFrame(&pColorFrame);
		}

		SafeRelease(pColorFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;

		hr = pMultiSourceFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);
		}

		SafeRelease(pBodyIndexFrameReference);
	}
	if (SUCCEEDED(hr))
	{
		IBodyFrameReference* pBodyFrameReference = NULL;

		hr = pMultiSourceFrame->get_BodyFrameReference(&pBodyFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameReference->AcquireFrame(&pBodyFrame);
		}

		SafeRelease(pBodyFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		INT64 nDepthTime = 0;
		IFrameDescription* pDepthFrameDescription = NULL;
		int nDepthWidth = 0;
		int nDepthHeight = 0;
		UINT nDepthBufferSize = 0;
		UINT16 *pDepthBuffer = NULL;

		IFrameDescription* pColorFrameDescription = NULL;
		int nColorWidth = 0;
		int nColorHeight = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nColorBufferSize = 0;
		RGBQUAD *pColorBuffer = NULL;

		IFrameDescription* pBodyIndexFrameDescription = NULL;
		int nBodyIndexWidth = 0;
		int nBodyIndexHeight = 0;
		UINT nBodyIndexBufferSize = 0;
		BYTE *pBodyIndexBuffer = NULL;

		// get depth frame data

		hr = pDepthFrame->get_RelativeTime(&nDepthTime);

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameDescription->get_Width(&nDepthWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameDescription->get_Height(&nDepthHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);            
		}

		// get color frame data

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameDescription->get_Width(&nColorWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameDescription->get_Height(&nColorHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}

		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
			}
			else if (m_pColorRGBX)
			{
				pColorBuffer = m_pColorRGBX;
				nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
			}
		}

		// get body index frame data

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrame->get_FrameDescription(&pBodyIndexFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameDescription->get_Width(&nBodyIndexWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameDescription->get_Height(&nBodyIndexHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyIndexBufferSize, &pBodyIndexBuffer);            
		}

		//get body frame

		IBody* ppBodies[BODY_COUNT] = {0};
		//hr = pBodyFrame->get_RelativeTime(&nTime);
		hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
		Vector4 floordata;
		pBodyFrame->get_FloorClipPlane(&floordata);
		if (SUCCEEDED(hr))
		{
			//Fork two tasks. Save to disk the raw data and process frame


			ProcessFrame(nDepthTime, pDepthBuffer, nDepthHeight, nDepthWidth, 
				pColorBuffer, nColorWidth, nColorHeight,
				pBodyIndexBuffer, nBodyIndexWidth, nBodyIndexHeight,
				BODY_COUNT,ppBodies,floordata);
		}



		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);
		}
		SafeRelease(pDepthFrameDescription);
		SafeRelease(pColorFrameDescription);
		SafeRelease(pBodyIndexFrameDescription);
		
	}

	SafeRelease(pDepthFrame);
	SafeRelease(pColorFrame);
	SafeRelease(pBodyIndexFrame);
	SafeRelease(pMultiSourceFrame);
	SafeRelease(pBodyFrame);
}

HRESULT Kinect_Thread::InitializeDefaultSensor()
{
	

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get the depth reader
		IDepthFrameSource* pDepthFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			//m_pKinectSensor-
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_BodyIndex | FrameSourceTypes::FrameSourceTypes_Body,
				&m_pMultiSourceFrameReader);
		}
	}


	if (!m_pKinectSensor || FAILED(hr))
	{
		cout<<"Could not initialize sensor"<<endl;
		return E_FAIL;
	}
	

	return hr;
}

void Kinect_Thread::run()
{
	while(m_continue)
	{
		Update();
	}
}

void Kinect_Thread::ProcessFrameAMP(INT64 nTime, UINT16* pDepthBuffer, int nDepthHeight, int nDepthWidth, RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight, BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight, int BodyCount,IBody** ppBodies)
{

}

void Kinect_Thread::endThread(void)
{

}

void Kinect_Thread::changeColor(int opt)
{
	lockGuard.lock();

	switch (opt)
	{
	case 0:
		m_Colour_Properties = Colour_Display_Properties::Display_Colour;
		break;
	case 1:
		m_Colour_Properties = Colour_Display_Properties::Display_Depth;
		break;
	case 2:
		m_Colour_Properties = Colour_Display_Properties::Display_Attention;
		break;
	case 3:
		m_Colour_Properties = Colour_Display_Properties::Display_Custom;
		break;
	default:
		m_Colour_Properties = Colour_Display_Properties::Display_Colour;
		break;
	}
	lockGuard.unlock();
}

void Kinect_Thread::changeHP(int opt)
{
	//lockGuard.lock();
	switch (opt)
	{
	case 0:
		m_HP_Properties = HeadPose_Display_Properties::Display_Single;

		break;
	case 1:
		m_HP_Properties = HeadPose_Display_Properties::Display_All;
		break;

	default:
		m_HP_Properties = HeadPose_Display_Properties::Display_All;
		//m_Colour_Properties = Colour_Display_Properties::Display_Colour;
		break;
	}
	//lockGuard.unlock();
}

void Kinect_Thread::hpComputeState(int opt)
{
	
	if (opt == 2)
	{
		m_ComputeHeadPose = true;
	}
	else
	{
		m_ComputeHeadPose = false;
	}
}

HRESULT Kinect_Thread::InitializeMLP(string jsonFile)
{
	//result = -1;
	input_count = 3072;
	output_count = 8;
	//jsonFile="E:/visual-rbm/Tools/RealTime/mlp_tuned.json";
	String in_json;
	ReadTextFile(jsonFile, in_json);
	cJSON* cj_root = cJSON_Parse(in_json.c_str());
	//MultilayerPerceptron *mlp= MultilayerPerceptron::FromJSON();
	//input = IDX::Load("E:/visual-rbm/Tools/RealTime/x1.idx");
	if (!Model::FromJSON(in_json, model))
	{
		printf("Could not parse model json from \"%s\"\n", in_json);
		goto CLEANUP;
	}
	return S_OK;


CLEANUP:


	switch (model.type)
	{

	case ModelType::MLP:
		delete model.mlp;
		break;
	}
	return E_FAIL;
}

void Kinect_Thread::changeColorScheme(int opt)
{
	
	colourScheme = opt;
}

void Kinect_Thread::changeKappa(int opt)
{
	kappa = opt;
}

void Kinect_Thread::record(void)
{
	record_data = !record_data;
}
