#ifndef _GLOBAL_DEF_ 
#define _GLOBAL_DEF_
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	A cloud points. </summary>
///
/// <remarks>	Rick, 27/02/2014. </remarks>
////////////////////////////////////////////////////////////////////////////////////////////////////

struct CloudPoints
{
	float X;
	float Y;
	float Z;

};

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Defines an alias representing the location 3D. </summary>
///
/// <remarks>	Rick, 27/02/2014. </remarks>
////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _Loc3d
{
	float x;
	float y;
	float z;
}Location;

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Defines an alias representing the head location. </summary>
///
/// <remarks>	Rick, 27/02/2014. Write Destructor </remarks>
////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _headLoc
{
	bool		HeadPoseAvailable=false;
	Location	Loc;
	float		HeadPose;
	float		probabilities [8];
	float		Radius;
}HeadLoc;

typedef struct _jointLoc
{
	Location Loc3D;
	ColorSpacePoint Loc2D;
	JointType type;
	bool tracked;
}JointLoc;

class Kinect_Data
{
public:

	int num_heads;
	std::vector<HeadLoc> locations;
	std::vector< std::vector<JointLoc> > bodies;
	Vector4 floorPlane;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud;
};

enum class Colour_Display_Properties
{
	Display_Colour,
	Display_Depth,
	Display_Attention,
	Display_Custom
};

enum class HeadPose_Display_Properties
{
	Display_Single,
	Display_All,
	
};


#endif