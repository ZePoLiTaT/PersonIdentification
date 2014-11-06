#include "SkeletonFeatures.h"
#include <math.h>
#include <cstdarg>
#include <iostream>

#include "pcl/common/distances.h"
#include "Global_def.h"


SkeletonFeatures::SkeletonFeatures()
{
}


SkeletonFeatures::~SkeletonFeatures()
{
}



float SkeletonFeatures::compute(const Location &ji, const Location &je) const
{
	float eucdist = 0;

	float xdif = ( je.x - ji.x );
	float ydif = ( je.y - ji.y );
	float zdif = ( je.z - ji.z );

	eucdist = sqrt(xdif*xdif + ydif *ydif + zdif *zdif);
	return eucdist;
}

float SkeletonFeatures::compute(const vector<Location> &locations) const
{
	float totalDistance = 0;
	for (int i = 0; i < locations.size() - 1 ; i++)
	{
		totalDistance += this->compute(locations[i], locations[i + 1]);
	}
	return totalDistance;
	
}

vector <float> SkeletonFeatures::extract(std::vector<JointLoc> const &body, Vector4 const & floorPlane) const
{
	cout << endl << "--- Euclidean:";

	vector <float> euclidean;

	// torso
	float torso = getDistanceTorso(body);

	// torso to head
	float torso2head = getDistanceTorso2Head(body,torso);

	// foot to hip
	float foot2hip = getDistanceFoot2Hip(body);

	// torso to hip
	float torso2hip = getDistanceTorso2Hip(body);

	////////////////////////////////////////////////////////////////////////
	// SKELETON-BASED FEATURES
	///////////////////////////////////////////////////////////////////////

	// floor to head
	float d1 = getDistancePoint2Plane(floorPlane, body.at(JointType::JointType_Head).Loc3D);
	euclidean.push_back(d1);

	// radio between torso and legs
	float d2 = torso / foot2hip;
	euclidean.push_back(d2);

	// height estimate
	float d3 = torso2head  + foot2hip;
	euclidean.push_back(d3);
	
	// floor to neck
	float d4 = getDistancePoint2Plane(floorPlane, body.at(JointType::JointType_Neck).Loc3D);
	euclidean.push_back(d4);
	
	// neck to left shoulder
	float d5 = getDistanceNeck2Shoulder(body.at(JointType::JointType_Neck), body.at(JointType::JointType_ShoulderLeft));
	euclidean.push_back(d5);
	
	// neck to right shoulder
	float d6 = getDistanceNeck2Shoulder(body.at(JointType::JointType_Neck), body.at(JointType::JointType_ShoulderRight));
	euclidean.push_back(d6);

	// torso to right shoulder
	float d7 = getDistanceTorso2Shoulder(body.at(JointType::JointType_SpineMid), body.at(JointType::JointType_ShoulderRight));
	euclidean.push_back(d7);

	return euclidean;
}

float SkeletonFeatures::getDistanceTorso(std::vector<JointLoc> const &body) const
{
	// torso
	if (body.at(JointType::JointType_SpineMid).tracked && body.at(JointType::JointType_SpineShoulder).tracked &&
		body.at(JointType::JointType_SpineBase).tracked)
	{
		vector<Location> head2torso;

		head2torso.push_back(body.at(JointType::JointType_SpineBase).Loc3D);
		head2torso.push_back(body.at(JointType::JointType_SpineMid).Loc3D);
		head2torso.push_back(body.at(JointType::JointType_SpineShoulder).Loc3D);

		float t_len = this->compute(head2torso);

		//cout << endl << "		1. Torso:   " << t_len;

		return t_len;
	}
	else
	{
		//cout << endl << "		1. Torso:  (NOT TRACKED CORRECTLY)";

		return -1;
	}
}

float SkeletonFeatures::getDistanceTorso2Head(std::vector<JointLoc> const &body, float t_len) const
{
	// torso to head
	if (body.at(JointType::JointType_SpineShoulder).tracked && body.at(JointType::JointType_Head).tracked)
	{
		vector<Location> head2torso;

		head2torso.push_back(body.at(JointType::JointType_Head).Loc3D);
		head2torso.push_back(body.at(JointType::JointType_Neck).Loc3D);
		head2torso.push_back(body.at(JointType::JointType_SpineShoulder).Loc3D);

		float h2t_len = t_len + this->compute(head2torso) + this->HEAD_SIZE;

		//cout << endl << "		1. Torso -> Head:   " << h2t_len;
		return h2t_len;
	}
	else
	{
		//cout << endl << "		1. Torso -> Head:  (NOT TRACKED CORRECTLY)";
		return -1;
	}
}

float SkeletonFeatures::getDistanceFoot2Hip(std::vector<JointLoc> const &body) const
{
	// foot to hip
	if (body.at(JointType::JointType_HipLeft).tracked)
	{
		// JointLoc at Left leg
		std::vector<JointLoc> legLeftJoints;
		legLeftJoints.push_back(body.at(JointType::JointType_HipLeft));
		legLeftJoints.push_back(body.at(JointType::JointType_KneeLeft));
		legLeftJoints.push_back(body.at(JointType::JointType_AnkleLeft));
		legLeftJoints.push_back(body.at(JointType::JointType_FootLeft));

		// Locations at Left leg
		std::vector<Location> legLeftLocations;
		legLeftLocations.push_back(body.at(JointType::JointType_HipLeft).Loc3D);
		legLeftLocations.push_back(body.at(JointType::JointType_KneeLeft).Loc3D);
		legLeftLocations.push_back(body.at(JointType::JointType_AnkleLeft).Loc3D);
		legLeftLocations.push_back(body.at(JointType::JointType_FootLeft).Loc3D);

		// JointLoc at Right leg
		std::vector<JointLoc> legRightJoints;
		legRightJoints.push_back(body.at(JointType::JointType_HipRight));
		legRightJoints.push_back(body.at(JointType::JointType_KneeRight));
		legRightJoints.push_back(body.at(JointType::JointType_AnkleRight));
		legRightJoints.push_back(body.at(JointType::JointType_FootRight));

		// Locations at Right leg
		std::vector<Location> legRightLocations;
		legRightLocations.push_back(body.at(JointType::JointType_HipRight).Loc3D);
		legRightLocations.push_back(body.at(JointType::JointType_KneeRight).Loc3D);
		legRightLocations.push_back(body.at(JointType::JointType_AnkleRight).Loc3D);
		legRightLocations.push_back(body.at(JointType::JointType_FootRight).Loc3D);

		// find which leg is tracked more accurately
		int legLeftTrackedJoints = numberOfTrackedJoints(legLeftJoints);
		int legRightTrackedJoints = numberOfTrackedJoints(legRightJoints);

		// *******************************************
		// TODO: decide how to do it with the leg
		// *******************************************

		// compute the distance with the leg that is tracked more accurately

		float h2t_len;

		if (legLeftTrackedJoints > legRightTrackedJoints) // left is tracked more accurately
		{
			h2t_len = this->compute(legLeftLocations);
		}
		else // right is tracked more accurately
		{
			h2t_len = this->compute(legRightLocations);
		}

		return h2t_len;

	}
	else
	{
		//cout << endl << "		1. Hip -> Foot:  (NOT TRACKED CORRECTLY)";
		return -1;
	}
}

int SkeletonFeatures::numberOfTrackedJoints(std::vector<JointLoc> const &joints) const
{
	int trackedJoints = 0;

	for (int i = 0; i < joints.size(); i++)
	{
		if (joints.at(i).tracked)
		{
			trackedJoints++;
		}
	}

	return trackedJoints;
}

float SkeletonFeatures::getDistanceNeck2Shoulder(JointLoc const &neck, JointLoc const &shoulder) const
{
	// neck to shoulder
	if ( neck.tracked && shoulder.tracked )
	{
		float n2s_len = this->compute(neck.Loc3D,shoulder.Loc3D);
		//cout << endl << "		1. Neck -> Shoulder:   " << n2s_len;
		return n2s_len;
	}
	else
	{
		//cout << endl << "		1. Neck -> Shoulder:  (NOT TRACKED CORRECTLY)";
		return -1;
	}
}

float SkeletonFeatures::getDistanceTorso2Shoulder(JointLoc const &torso, JointLoc const &shoulder) const
{
	// torso to shoulder
	if (torso.tracked && shoulder.tracked)
	{
		float t2s_len = this->compute(torso.Loc3D, shoulder.Loc3D);
		//cout << endl << "		1. Torso -> Shoulder:   " << t2s_len;
		return t2s_len;
	}
	else
	{
		//cout << endl << "		1. Torso -> Shoulder:  (NOT TRACKED CORRECTLY)";
		return -1;
	}
}

float SkeletonFeatures::getDistanceTorso2Hip(std::vector<JointLoc> const &body) const
{
	// foot to hip
	if (body.at(JointType::JointType_HipLeft).tracked)
	{
		// JointLoc at Left leg
		std::vector<JointLoc> legLeftJoints;
		legLeftJoints.push_back(body.at(JointType::JointType_HipLeft));
		legLeftJoints.push_back(body.at(JointType::JointType_KneeLeft));
		legLeftJoints.push_back(body.at(JointType::JointType_AnkleLeft));
		legLeftJoints.push_back(body.at(JointType::JointType_FootLeft));

		// JointLoc at Right leg
		std::vector<JointLoc> legRightJoints;
		legRightJoints.push_back(body.at(JointType::JointType_HipRight));
		legRightJoints.push_back(body.at(JointType::JointType_KneeRight));
		legRightJoints.push_back(body.at(JointType::JointType_AnkleRight));
		legRightJoints.push_back(body.at(JointType::JointType_FootRight));

		// find which leg is tracked more accurately
		int legLeftTrackedJoints = numberOfTrackedJoints(legLeftJoints);
		int legRightTrackedJoints = numberOfTrackedJoints(legRightJoints);

		// *******************************************
		// TODO: decide how to do it with the leg
		// *******************************************

		// compute the distance with the leg that is tracked more accurately

		float t2h_len;

		if (legLeftTrackedJoints > legRightTrackedJoints) // left is tracked more accurately
		{
			t2h_len = this->compute(body.at(JointType::JointType_HipLeft).Loc3D, body.at(JointType::JointType_SpineBase).Loc3D);
		}
		else // right is tracked more accurately
		{
			t2h_len = this->compute(body.at(JointType::JointType_HipRight).Loc3D, body.at(JointType::JointType_SpineBase).Loc3D);
		}

		return t2h_len;

	}
	else
	{
		//cout << endl << "		1. Hip -> Foot:  (NOT TRACKED CORRECTLY)";
		return -1;
	}
}

float SkeletonFeatures::getDistancePoint2Plane(Vector4 const & floorPlane, Location const & p) const
{
	float numerator = (floorPlane.x * p.x) + (floorPlane.y * p.y) + (floorPlane.z * p.z) + floorPlane.w;
	float denominator = sqrt((floorPlane.x*floorPlane.x) + (floorPlane.y*floorPlane.y) + (floorPlane.z*floorPlane.z) );
	float D = numerator / denominator;

	return D;
}

//float SkeletonFeatures::computeSegments(const Location* loc, ...) const
//{
//	//for (int i = 0; i<)
//	va_list args;
//	va_start(args, loc);
//
//	Location l;
//
//	while (loc != NULL)
//	{
//		l = va_arg(loc, Location);
//		cout << " ::> Location (" << l.x << "," << l.y << "," << l.z << ") " << endl;
//	//	if (*fmt == 'd') {
//	//		int i = va_arg(args, int);
//	//		std::cout << i << '\n';
//	//	}
//	//	else if (*fmt == 'c') {
//	//		// note automatic conversion to integral type
//	//		int c = va_arg(args, int);
//	//		std::cout << static_cast<char>(c) << '\n';
//	//	}
//	//	else if (*fmt == 'f') {
//	//		double d = va_arg(args, double);
//	//		std::cout << d << '\n';
//	//	}
//	//	++fmt;
//	}
//
//	va_end(args);
//
//	return 0;
//}