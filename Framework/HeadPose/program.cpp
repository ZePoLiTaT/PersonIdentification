#include <QApplication>
#include "RenderWindowUISingleInheritance.h"
#include "IMU.h"
#include<stdafx.h>
int main( int argc, char** argv )
{
  // QT Stuff
  QApplication app( argc, argv );
  qRegisterMetaType<QuaternionValue>();
  qRegisterMetaType<EulerAnglesStruct>();
  RenderWindowUISingleInheritance renderWindowUISingleInheritance;
  renderWindowUISingleInheritance.show();
 
  return app.exec();
}