/*
    Quaternion.h
    Author: Seb Madgwick

    C++ library for basic usage of quaternions.
    See: http://www.x-io.co.uk/quaternions/
*/

#ifndef Quaternion_h
#define Quaternion_h
#include<qobject.h>
#include <QMetaType>
#include<math.h>
//------------------------------------------------------------------------------
// Definitions

typedef struct _EulerAngle
{
    float roll;     /* rotation around x axis in degrees */
    float pitch;    /* rotation around y axis in degrees */
    float yaw;      /* rotation around z axis in degrees */
} EulerAnglesStruct;
Q_DECLARE_METATYPE(EulerAnglesStruct);
//------------------------------------------------------------------------------
// Class declaration

class Quaternion {
    public:
        Quaternion(void);
        Quaternion(const float w, const float x, const float y, const float z);
        Quaternion getConjugate(void) const;
        EulerAnglesStruct getEulerAngles(void) const;

    private:
        float q[4];
        float radiansToDegrees (float radians) const;
};

#endif

//------------------------------------------------------------------------------
// End of file
