#include <GL/glew.h>
#include <maya/MBoundingBox.h>
#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>
#include <maya/M3dView.h>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MPxCommand.h>
#include <maya/MAnimControl.h>
#include <maya/MRenderUtil.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MIntArray.h>
#include <maya/MMatrix.h>
#include <maya/MFnMesh.h>
#include <maya/MItDag.h>
#include <maya/MFloatVector.h>
#include <maya/MFloatPoint.h>
#include <maya/MImage.h>
#include <maya/MFloatMatrix.h>
#include <maya/MPointArray.h>
#include <maya/MMatrixArray.h>
#include <maya/MDagPath.h>
#include <maya/MFnCamera.h>
#include <maya/MArgDatabase.h>
#include <maya/MSyntax.h>
#include <maya/MTransformationMatrix.h>
#include <gl/GLU.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_lm.h>
