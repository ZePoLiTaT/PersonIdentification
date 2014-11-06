#include "RenderWindowUISingleInheritance.h"

// This is included here because it is forward declared in
// RenderWindowUISingleInheritance.h
#include "ui_form.h"
#include <Global_def.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>
#include <vtkSmartPointer.h>
#include <vtkWindowToImageFilter.h>
#include <vtkImageData.h>
#include <boost\lexical_cast.hpp>
#include <ppl.h>
#include <ppltasks.h>
#include "GeodesicFeatures.h"
#include "SkeletonFeatures.h"


// Constructor

using namespace concurrency;

void savefeatures(ofstream &fout, vector<float> skfeat, vector<float> gdfeat)
{
	vector <float>::iterator featIterator;

	for (featIterator = skfeat.begin(); featIterator != skfeat.end(); featIterator++)
	{
		fout << *featIterator << ", ";
	}

	for (featIterator = gdfeat.begin(); featIterator != gdfeat.end(); featIterator++)
	{
		fout << *featIterator << ", ";
	}
	fout << endl;	
}

double d2r(double d)
{
	return d * M_PI / 180;
}
double r2d(double r)
{
	return r * 180 / M_PI;
}
//////////////////////////////////////////////////////////////////////////
// 
//////////////////////////////////////////////////////////////////////////
void getQuat(double heading, double attitude, double bank, Eigen::Quaternionf & q)
{
	// Assuming the angles are in radians.
	float c1 = cos(heading);
	float s1 = sin(heading);
	float c2 = cos(attitude);
	float s2 = sin(attitude);
	float c3 = cos(bank);
	float s3 = sin(bank);
	float w = sqrt(1.0 + c1 * c2 + c1*c3 - s1 * s2 * s3 + c2*c3) / 2.0;
	float w4 = (4.0 * w);
	float x = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / w4;
	float y = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / w4;
	float z = (-s1 * s3 + c1 * s2 * c3 + s2) / w4;
	q = Eigen::Quaternionf(w, x, y, z);
}

void getAngles(Eigen::Quaternionf &q1, float & heading, float & attitude, float & bank)
{
	double test = q1.x()*q1.y() + q1.z()*q1.w();

	if (test > 0.499) { // singularity at north pole
		heading = 2 * atan2(q1.x(), q1.w());
		attitude = M_PI_2;
		bank = 0;
		return;
	}
	if (test < -0.499) { // singularity at south pole
		heading = -2 * atan2(q1.x(), q1.w());
		attitude = -M_PI_2;
		bank = 0;
		return;
	}
	double sqx = q1.x()*q1.x();
	double sqy = q1.y()*q1.y();
	double sqz = q1.z()*q1.z();
	heading = atan2(2 * q1.y()*q1.w() - 2 * q1.x()*q1.z(), 1 - 2 * sqy - 2 * sqz);
	attitude = asin(2 * test);
	bank = atan2(2 * q1.x()*q1.w() - 2 * q1.y()*q1.z(), 1 - 2 * sqx - 2 * sqz);
}


RenderWindowUISingleInheritance::RenderWindowUISingleInheritance() 
{
	this->geodist = 0;
	this->ui = new Ui_MainWindow;
	this->ui->setupUi(this);
	currentDir=QDir("C:/Rick/Data/HeadPose/");
	currentDir.setFilter(QDir::Dirs| QDir::NoDotAndDotDot);
	QStringList entries = currentDir.entryList();
	QStringList::ConstIterator entry=entries.begin();
	fout.open("C:/local/data/test2.csv", ios::out);
	int maximum=0,current;
	for(entry=entries.begin();entry !=entries.end();entry++)
	{
		int len=entry->length();
		current=entry->mid(6).toInt();
		if(current>maximum)
			maximum=current;
		
	}
	personNumber=maximum;
	current_Dir=QString("C:/Rick/Data/HeadPose/Person%1/").arg(++personNumber);
	new_dir=QDir("C:/Rick/Data/HeadPose");
	if(!new_dir.exists(current_Dir))
	{
		new_dir.mkdir(current_Dir);
		
	}
	
	this->ui->dial_Pitch->setNotchesVisible(true);
	this->ui->dial_Roll->setNotchesVisible(true);
	this->ui->dial_Yaw->setNotchesVisible(true);
	frameNumber=0;
	record_data=false;
	writer=pcl::PCDWriter();
	govt=new PipelineUtilities::PipelineGovernor(5);
	frames=new concurrency::unbounded_buffer<shared_ptr<Kinect_Data>>();
	sensor_Data=new overwrite_buffer<tuple<QuaternionValue,EulerAnglesStruct>>();
	kinectThread=new Kinect_Thread(*govt,*frames);
	//imuThread=new IMU_THREAD(*sensor_Data);
	
	//Create connections 
	//connect(this,SIGNAL(cont(void)),imuThread,SLOT(getNext(void)),Qt::QueuedConnection);
	connect(kinectThread,SIGNAL(Kinect_Frame_Available()),this,SLOT(frameReceived()),Qt::QueuedConnection);
	//connect(imuThread,SIGNAL(Sensor_Data_Available(QuaternionValue,EulerAnglesStruct)),this,SLOT(dataReceived(QuaternionValue,EulerAnglesStruct)),Qt::QueuedConnection);
	connect(this->ui->btn_Record,SIGNAL(clicked()),this,SLOT(record()));
	connect(this->ui->btn_new,SIGNAL(clicked()),this,SLOT(new_person()));
	connect(this->ui->cb_ColourOptions, SIGNAL(currentIndexChanged(int)), this, SLOT(colOPTChange(int)));
	connect(this->ui->cb_HPOptions, SIGNAL(currentIndexChanged(int)), this, SLOT(hpOPTChange(int)));
	connect(this, SIGNAL(colourDisplayParameterChange(int)), kinectThread, SLOT(changeColor(int)), Qt::QueuedConnection);
	connect(this, SIGNAL(hpDisplayParameterChange(int)), kinectThread, SLOT(changeHP(int)), Qt::QueuedConnection);
	connect(this->ui->chb_HP, SIGNAL(stateChanged(int)), kinectThread, SLOT(hpComputeState(int)),Qt::QueuedConnection);
	connect(this->ui->cb_ColourScheme, SIGNAL(currentIndexChanged(int)), kinectThread, SLOT(changeColorScheme(int)), Qt::QueuedConnection);
	connect(this->ui->sliderVMF, SIGNAL(valueChanged(int)), kinectThread, SLOT(changeKappa(int)), Qt::QueuedConnection);
	connect(this->ui->chb_VirtualCanvas, SIGNAL(stateChanged(int)), this, SLOT(broadcastOPTChange(int)));
	connect(this->ui->btn_Record, SIGNAL(clicked()), kinectThread, SLOT(record()), Qt::QueuedConnection);
	cloud=pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	
	//pcl::io::loadPCDFile<pcl::PointXYZRGB> ("C:/rick/pcldata/file52.pcd", *cloud);
	viewer=boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->setBackgroundColor(0, 0, 0);
	
	viewer->initCameraParameters ();
	// VTK/Qt wedded
	//this->ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());

	//viewer->setupInteractor(this->ui->qvtkWidget->GetInteractor(), this->ui->qvtkWidget->GetRenderWindow());
	//viewer->setupInteractor(
	//viewer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
	this->ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->getRenderWindow()->Render(); 	
	//Get RGB Image
	
	renderWindow = viewer->getRenderWindow();
	windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	windowToImageFilter->SetInput(renderWindow);
	//get rgb-image
	windowToImageFilter->SetInputBufferTypeToRGB();
	//imuThread->start();
	kinectThread->start();
	frameNumber = 0;
	broadcast = false;
}
 
void RenderWindowUISingleInheritance::slotExit() 
{
  qApp->exit();
}

//void RenderWindowUISingleInheritance::
void RenderWindowUISingleInheritance::dataReceived(QuaternionValue data,EulerAnglesStruct angles)
{
	//this->ui->lNum_QW->display(angles.yaw);
	this->ui->lNum_QX->display(angles.yaw);
	this->ui->lNum_QY->display(angles.pitch);
	this->ui->lNum_QZ->display(angles.roll);
	viewer->removeAllShapes();
	Eigen::Vector3f trans=Eigen::Vector3f(0,0,0);
	Eigen::Quaternionf quat=Eigen::Quaternionf(data.Qw,data.Qx,data.Qy,data.Qz);
	quat.normalize();
	viewer->removeAllShapes();
	viewer->addCube(trans,quat,1,1,1);
	//viewer->spinOnce(1);
	this->ui->qvtkWidget->update();
	emit cont();
	//viewer->spinOnce(10);
}

void RenderWindowUISingleInheritance::frameReceived()
{
	//cout << "received"<<endl;
	shared_ptr<Kinect_Data> dat;
	dat=receive(*frames);
	frameNumber++;
	//std::tuple<QuaternionValue,EulerAnglesStruct> sens_dat=	receive(*sensor_Data,10);
	
	viewer->removeAllShapes();
	//dat->locations[0].Loc.x
	if(dat->num_heads>0)
	{
		

		if (frameNumber % 5 == 0)
		{

		}

		stringstream names=stringstream("");
		stringstream arrownames = stringstream("");
		//cout<<dat->num_heads<<",";
		if (dat->num_heads==1)
		{
			//Check whether we are computing headpose
			if (this->ui->chb_HP->checkState() == 2 && dat->locations[0].HeadPoseAvailable)
			{
				Eigen::Quaternionf quat=Eigen::Quaternionf();
				getQuat(-dat->locations[0].HeadPose, 0, 0, quat);
				quat.normalize();
				Eigen::Vector3f pt = Eigen::Vector3f(0, 0, -0.25);
				Eigen::Vector3f new_point = quat._transformVector(pt);

				Eigen::Vector3f trans = Eigen::Vector3f(dat->locations[0].Loc.x, dat->locations[0].Loc.y, dat->locations[0].Loc.z);
				//cout << dat->locations[0].Loc.x << "," << dat->locations[0].Loc.y << "," << dat->locations[0].Loc.z << endl;
				PointXYZRGB pt1, pt2;
				pt1.x = trans[0];
				pt1.y = trans[1];
				pt1.z = trans[2];
				pt1.r = 255;
				pt1.g = 255;
				pt1.b = 255;
				pt2.x = trans[0] + new_point[0];
				pt2.y = trans[1] + new_point[1];
				pt2.z = trans[2] + new_point[2];
				pt2.r = pt2.g = pt2.b = 255;
				names << "box1";
				arrownames << "arrow1";
				float head_size = 2 * dat->locations[0].Radius;
				viewer->addCube(trans, quat, head_size, head_size, head_size, names.str());
				this->ui->dial_Yaw->setValue((int) floor(dat->locations[0].HeadPose));
				this->ui->txt_Yaw->setText(QString::number(dat->locations[0].HeadPose));
				//viewer->addArrow(pt1, pt2, 255, 255, 0);
				viewer->addLine(pt1, pt2, 255, 255, 0, arrownames.str());
				//ui->l
				this->ui->lbl_Temp->setText(QString::number(dat->locations[0].HeadPose/45.0f));
			}
			else //try to get it from sensor
			{
				
				//Eigen::Quaternionf quat = Eigen::Quaternionf(get<0>(sens_dat).Qw, -get<0>(sens_dat).Qx, get<0>(sens_dat).Qy, get<0>(sens_dat).Qz);
				Eigen::Quaternionf quat = Eigen::Quaternionf(1, 1, 1, 1);
				quat.normalize();
				Eigen::Vector3f pt = Eigen::Vector3f(0, 0, -0.25);
				Eigen::Vector3f new_point = quat._transformVector(pt);

				Eigen::Vector3f trans = Eigen::Vector3f(dat->locations[0].Loc.x, dat->locations[0].Loc.y, dat->locations[0].Loc.z);
				//cout << dat->locations[0].Loc.x << "," << dat->locations[0].Loc.y << "," << dat->locations[0].Loc.z << endl;
				PointXYZRGB pt1, pt2;
				pt1.x = trans[0];
				pt1.y = trans[1];
				pt1.z = trans[2];
				pt1.r = 255;
				pt1.g = 255;
				pt1.b = 255;
				pt2.x = trans[0] + new_point[0];
				pt2.y = trans[1] + new_point[1];
				pt2.z = trans[2] + new_point[2];
				pt2.r = pt2.g = pt2.b = 255;

				names << "box1";
				arrownames << "arrow1";
				float head_size = 2 * dat->locations[0].Radius;
				viewer->addCube(trans, quat, head_size, head_size, head_size, names.str());
				/*this->ui->dial_Pitch->setValue((int) floor(get<1>(sens_dat).pitch));
				this->ui->dial_Roll->setValue((int) floor(get<1>(sens_dat).roll));
				this->ui->dial_Yaw->setValue((int) floor(get<1>(sens_dat).yaw));
				this->ui->txt_Pitch->setText(QString::number(get<1>(sens_dat).pitch));
				this->ui->txt_Roll->setText(QString::number(get<1>(sens_dat).roll));
				this->ui->txt_Yaw->setText(QString::number(get<1>(sens_dat).yaw));*/
				//viewer->addArrow(pt1,pt2,255,255,0);
				viewer->addLine(pt1, pt2, 255, 255, 0, arrownames.str());
			}
		}
		else		
		{
			//Iterate through all heads
			for (int i = 0; i < dat->num_heads; i++)
			{
				
				names << "box" << i;
				arrownames << "arrow" << i;
				Eigen::Quaternionf quat = Eigen::Quaternionf();
				bool drawarrow = false;
				if (this->ui->chb_HP->checkState() == 2 && dat->locations[0].HeadPoseAvailable)
				{
					
					getQuat(dat->locations[i].HeadPose, 0, 0, quat);
					quat.normalize();
					drawarrow = true;
				}
				else
				{
					getQuat(0, 0, 0, quat);
					quat.normalize();
					drawarrow = false;
				}

				Eigen::Vector3f pt = Eigen::Vector3f(0, 0, -0.25);
				Eigen::Vector3f new_point = quat._transformVector(pt);

				Eigen::Vector3f trans = Eigen::Vector3f(dat->locations[i].Loc.x, dat->locations[i].Loc.y, dat->locations[i].Loc.z);
				//cout << dat->locations[i].Loc.x << "," << dat->locations[i].Loc.y << "," << dat->locations[i].Loc.z << endl;
				PointXYZRGB pt1, pt2;
				pt1.x = trans[0];
				pt1.y = trans[1];
				pt1.z = trans[2];
				pt1.r = 255;
				pt1.g = 255;
				pt1.b = 255;
				pt2.x = trans[0] + new_point[0];
				pt2.y = trans[1] + new_point[1];
				pt2.z = trans[2] + new_point[2];
				pt2.r = pt2.g = pt2.b = 255;
				float head_size = 2 * dat->locations[i].Radius;
				viewer->addCube(trans, quat, head_size, head_size, head_size, names.str());
				//viewer->addArrow(pt1, pt2, 255, 255, 0,arrownames.str());
				if (drawarrow)
				{
					viewer->addLine(pt1, pt2, 255, 255, 0, arrownames.str());
					//viewer->addArrow(pt1, pt2, 255, 255, 0, arrownames.str());
				}
				
				
			}
		}
	}
	
	viewer->removeAllPointClouds();
	
	//pcl::PointCloud<PointXYZRGBNormal>::ConstPtr pointr=dat.cloud->ConstPtr;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(dat->new_cloud);

	viewer->addPointCloud<PointXYZRGB>(dat->new_cloud,rgb);
	this->ui->qvtkWidget->update();

	if (dat->num_heads > 0 )
	{
		this->geodist++;

		GeodesicFeatures geo;
		SkeletonFeatures ske;

//		cout << "Floor Plane " << dat->floorPlane;

		// Triangulate the mesh
		geo.processCloud(dat->new_cloud);
		
		for (int i = 0; i < dat->num_heads; i++)
		{
			cout << endl << "============== PERSON [" << i << "] =============";

			//Eigen::Vector3f trans = Eigen::Vector3f(dat->bodies[i].at(JointType::JointType_SpineMid).Loc3D.x, 
			//										dat->bodies[i].at(JointType::JointType_SpineMid).Loc3D.y, 
			//										dat->bodies[i].at(JointType::JointType_SpineMid).Loc3D.z - 0.2);

			//Eigen::Quaternionf quat = Eigen::Quaternionf();
			//getQuat(0, 0, 0, quat);
			//quat.normalize();
			//viewer->addCube(trans, quat, 0.05, 0.05, 0.05, "x");


			vector<float> feat_gd = geo.extract(dat->bodies[i]);
			vector<float> feat_sk = ske.extract(dat->bodies[i],dat->floorPlane);

			for (int i = 0; i < feat_sk.size(); i++)
			{
				cout << "sd(" << (i + 1) << ") = " << feat_sk.at(i)<<endl;
			}
			
			for (int i = 0; i < feat_gd.size(); i++)
			{
				cout << "gd(" << (i + 1) << ") = " << feat_gd.at(i) << endl;
			}

			savefeatures(fout, feat_sk, feat_gd);
		}
		
	}
	
	//windowToImageFilter->SetInput(renderWindow);
	if (broadcast)
	{
		windowToImageFilter->Update();
		vtkRGBimage = windowToImageFilter->GetOutput();
		int dimsRGBImage[3];
		vtkRGBimage->GetDimensions(dimsRGBImage);
		cv::Mat cvImageRGB(dimsRGBImage[1], dimsRGBImage[0], CV_8UC3, vtkRGBimage->GetScalarPointer());
		cv::cvtColor(cvImageRGB, cvImageRGB, CV_BGR2RGB); //convert color
		cv::flip(cvImageRGB, cvImageRGB, 0); //align axis with visualizer
		imshow("aa",cvImageRGB);
		//send it to virtual canvas. 


		//Currently save it
		/*stringstream nm = stringstream("");
		nm << "d:/data/OutImages/" << frameNumber << ".jpg";
		imwrite(nm.str(), cvImageRGB);*/
	}
	// SAVE DATA
	if (record_data)
	{
		
		std::stringstream pcd_file = std::stringstream("");
		std::stringstream ground_truth_file = std::stringstream("");

		pcd_file.flush();
		ground_truth_file.flush();
		ground_truth_file << current_Dir.toStdString() << "Ground_Truth" << frameNumber << ".dat";

		pcd_file << current_Dir.toStdString() << "Data" << frameNumber << ".pcd";
		fileWriter = ofstream(ground_truth_file.str());
		writer = pcl::PCDWriter();
		writer.writeBinary(pcd_file.str(), *dat->new_cloud);
		listFileWriter << pcd_file.str() << "," << ground_truth_file.str() << endl;
		fileWriter << dat->num_heads << endl;
		for (int i = 0; i < dat->num_heads; i++)
		{
			fileWriter << dat->locations[i].Loc.x
				<< "," << dat->locations[i].Loc.y
				<< "," << dat->locations[i].Loc.z
				<< "," << dat->locations[i].Radius
				<< "," << dat->locations[i].HeadPose
				<< "," << dat->locations[i].probabilities[0]
				<< "," << dat->locations[i].probabilities[1]
				<< "," << dat->locations[i].probabilities[2]
				<< "," << dat->locations[i].probabilities[3]
				<< "," << dat->locations[i].probabilities[4]
				<< "," << dat->locations[i].probabilities[5]
				<< "," << dat->locations[i].probabilities[6]
				<< "," << dat->locations[i].probabilities[7]
				<< endl;

			//fileWriter << get<0>(sens_dat).Qw << endl << get<0>(sens_dat).Qx << endl << get<0>(sens_dat).Qy << endl << get<0>(sens_dat).Qz << endl;
		}
		fileWriter.close();

	}

	//imshow("aa", cvImageRGB);
	//waitKey(0);
	//cleanup
	dat->new_cloud->clear();
	//delete dat;
	dat->new_cloud.reset();
	dat.reset();
	//if (!cvImageRGB.empty()) cvImageRGB.release();
	govt->FreePipelineSlot();
}

void RenderWindowUISingleInheritance::record()
{
	if(record_data==true)
	{
		record_data=false;
		listFileWriter.close();
		this->ui->btn_Record->setText(QString(L'►'));
		this->ui->cb_ColourOptions->setDisabled(false);
		current_Dir = QString("C:/Rick/Data/HeadPose/Person%1/").arg(++personNumber);
		//new_dir = QDir("C:/Rick/Data/HeadPose");
		if (!new_dir.exists(current_Dir))
		{
			new_dir.mkdir(current_Dir);

		}
	}
	else
	{
		std::stringstream list_file = std::stringstream("");
		list_file << current_Dir.toStdString() << "list" << ".csv";
		listFileWriter = ofstream(list_file.str());
		record_data=true;
		this->ui->cb_ColourOptions->setCurrentIndex(0);
		this->ui->cb_ColourOptions->setDisabled(true);
		this->ui->btn_Record->setText(QString(L'■'));
	}
}

void RenderWindowUISingleInheritance::pause()
{

}

void RenderWindowUISingleInheritance::new_person()
{
	
	frameNumber=0;
	personNumber++;
	dirName=QString("C:/Rick/Data/HeadPose/Person%1/").arg(personNumber);
	new_dir=QDir("C:/Rick/Data/HeadPose");
	if(!new_dir.exists(dirName))
	{
		new_dir.mkdir(dirName);
		current_Dir=dirName;

	}
	else
	{
		//Handle case where directories exist
	}
}

void RenderWindowUISingleInheritance::colOPTChange(int opt)
{
	emit colourDisplayParameterChange(opt);
}

void RenderWindowUISingleInheritance::hpOPTChange(int opt)
{
	emit hpDisplayParameterChange(opt);
}

void RenderWindowUISingleInheritance::broadcastOPTChange(int opt)
{
	if (opt == 2)
	{
		broadcast = true;
	}
	else
	{
		broadcast = false;
	}
}

RenderWindowUISingleInheritance::~RenderWindowUISingleInheritance()
{
	
	fout.close();
}
