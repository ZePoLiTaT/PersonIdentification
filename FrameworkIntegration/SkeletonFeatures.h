#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Kinect.h>
#include "Global_def.h"

#include <iostream>
#include <vector>
#include <math.h>

using namespace std;

class SkeletonFeatures
{
public:
	SkeletonFeatures();
	~SkeletonFeatures();

	/*float computeSegments(const Location* loc, ...) const;*/
	float compute(const Location &ji, const Location &je) const;
	float compute(const vector<Location> &locations) const;

	vector <float> extract(std::vector<JointLoc> const &body, Vector4 const & floorPlane) const;

	float getDistancePoint2Plane(Vector4 const & floorPlane, Location const & p) const;

	float getDistanceTorso(std::vector<JointLoc> const &body) const;
	float getDistanceTorso2Head(std::vector<JointLoc> const &body, float t_len) const;
	float getDistanceFoot2Hip(std::vector<JointLoc> const &body) const;
	int numberOfTrackedJoints(std::vector<JointLoc> const &joints) const;
	float getDistanceNeck2Shoulder(JointLoc const &neck, JointLoc const &shoulder) const;
	float getDistanceTorso2Shoulder(JointLoc const &torso, JointLoc const &shoulder) const;
	float getDistanceTorso2Hip(std::vector<JointLoc> const &body) const;

private:
	const float HEAD_SIZE = 0.13;
};

