#include "stdafx.h"


class MultiPerspective : public MPxCommand 
{
public:
	static void *Creator();

	virtual MStatus doIt( const MArgList& args );
};

void *
MultiPerspective::Creator()
{
	return new MultiPerspective;
}

GLuint g_frameBuffer;
GLuint g_frameBufferTexture;
GLuint g_renderBuffer;

GLuint g_width = 2048, g_height = 1024;

void
InitFramebuffer()
{
	glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
    glGenTextures( 1, &g_frameBufferTexture );
    glBindTexture( GL_TEXTURE_2D, g_frameBufferTexture );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, g_width, g_height,
                  0, GL_RGBA, GL_UNSIGNED_BYTE, 0 );
	glBindTexture(GL_TEXTURE_2D, 0);

	glGenRenderbuffers( 1, &g_renderBuffer );
	glBindRenderbuffer( GL_RENDERBUFFER_EXT, g_renderBuffer );
    glRenderbufferStorage( GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT,
                              g_width, g_height );
	glBindRenderbuffer(GL_RENDERBUFFER, 0);

    glGenFramebuffers( 1, &g_frameBuffer );
    glBindFramebuffer( GL_FRAMEBUFFER_EXT, g_frameBuffer );

    glFramebufferTexture2D( GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
                               GL_TEXTURE_2D, g_frameBufferTexture, 0 );
    glFramebufferRenderbuffer( GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
                                  GL_RENDERBUFFER_EXT, g_renderBuffer );

    glBindFramebufferEXT( GL_FRAMEBUFFER_EXT, 0 );
}
void
UninitFramebuffer()
{
	glDeleteFramebuffers(1, &g_frameBuffer);
	glDeleteRenderbuffers(1, &g_renderBuffer);
	glDeleteTextures(1, &g_frameBufferTexture);
}

inline void
addvert(float u, float v, float aspect)
{
	float x = (1-2*u);
	float y = (2*v-1)/aspect;
	float z = -sqrt(x*x+y*y);
	glTexCoord2f(u, v);
	glVertex3d(x, y, z);
}

double computeTotalLength(pcl::PointCloud<pcl::PointXYZ> const &points)
{
	double length = 0;
	for (int i = 0; i < points.size(); ++i) {
		MVector p0(points[i].x, points[i].y, points[i].z);
		for (int j = i+1; j < points.size(); ++j) {
			MVector p1(points[j].x, points[j].y, points[j].z);
			length += (p1-p0).length();
		}
	}
	return length;
}

MMatrix estimateTransfrom(pcl::PointCloud<pcl::PointXYZ> const & source, pcl::PointCloud<pcl::PointXYZ> const & target)
{
	pcl::PointCloud<pcl::PointXYZ> scaled_target;

	if (source.size() < 2) {
		std::cerr << "Identity !\n";
		MMatrix mat;
		mat.setToIdentity();
		return mat;
	}

	double len = computeTotalLength(source);
	double len2 = computeTotalLength(target);
	double scale = len2/len;

	scale = 1.0;

	static double prevScale = 0;

//	scale = std::min(1.1, scale);
//	scale = std::max(0.9, scale);
	if (scale > 1.1 || scale < 0.9) scale = prevScale;

	if (prevScale == 0) prevScale = scale;

	double iscale = 1.0/scale;

	for(int i = 0; i < target.size(); ++i) {
		scaled_target.push_back(pcl::PointXYZ(target[i].x * iscale, target[i].y * iscale, target[i].z * iscale));
	}
	
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;

	MMatrix mat;
	Eigen::Matrix4f transform;
	svd.estimateRigidTransformation(source, scaled_target, transform);
	for(int i = 0; i < 4; ++i)
		for(int j = 0; j < 4; ++j)
			mat(i, j) = transform(j, i);

	MTransformationMatrix tmat;
	double sc[3] = {scale, scale, scale};
	tmat.setScale(sc, MSpace::kWorld);

//	std::cerr << mat << "\n";
	mat *= tmat.asMatrix();

//	std::cerr << "Scale = " << scale << "\n";

	// verify
#if 0
	for (int i = 0; i < source.size(); ++i) {
		MPoint p(source[i].x, source[i].y, source[i].z);
		MPoint q(target[i].x, target[i].y, target[i].z);
		p *= mat;
		std::cerr << "target = " << q << " // " << p << "\n";
	}
	std::cerr << mat << "\n";
#endif

	return mat;
}

void test()
{
	pcl::PointCloud<pcl::PointXYZ> source;
	pcl::PointCloud<pcl::PointXYZ> target;

	source.push_back(pcl::PointXYZ(1, 1, 0));
	target.push_back(pcl::PointXYZ(0.5, 0.5, 0));

	source.push_back(pcl::PointXYZ(-1, -1, 0));
	target.push_back(pcl::PointXYZ( -1, -1, 0));

	estimateTransfrom(source, target);
}

bool isJumped(MMatrix const &a, MMatrix const &b)
{
	double epsilon = 0.1;
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			if (fabs(a[i][j] - b[i][j]) > epsilon) return true;
	return false;
}

MStatus
MultiPerspective::doIt(const MArgList &args)
{

	MSyntax syntax;
	syntax.addFlag("f", "file", MSyntax::kString);
	MArgDatabase argDB(syntax, args);

	if (!argDB.isFlagSet("f")) {
		MGlobal::displayError("please specify output file : -f <filename.png>");
		return MS::kFailure;
	}
	MString filename;
	argDB.getFlagArgument("f", 0, filename);

	MTime start = MAnimControl::animationStartTime();
	MTime end = MAnimControl::animationEndTime();
	M3dView view = M3dView::active3dView();
	MDagPath cameraPath;
	view.getCamera(cameraPath);
	MFnCamera cameraFn(cameraPath);

	float cumulativeDepth = 0.0;
	int cumulativeDepthCount = 0;


	unsigned int viewport[4];
	unsigned int width, height;
	view.viewport(viewport[0], viewport[1], viewport[2], viewport[3]);
	width = viewport[2];
	height = viewport[3];

	std::cerr << "Viewport = " << viewport[0] << "," << viewport[1] << ", " << viewport[2] << ", " << viewport[3] << "\n";

	double aspect = width/(float)height;

	MPointArray points, spoints;

	MMatrix cumMat;
	MMatrix prevMat;
	cumMat.setToIdentity();
	MMatrixArray matrices;

	const int NUM_GRID_LEVEL = 5;
	// sample grids
	MFloatVectorArray gridPoints[NUM_GRID_LEVEL];
	for (int i = 0; i < NUM_GRID_LEVEL; ++i) {
		int d = 2 << i;
		for (int x = 0; x <= d; ++x) {
			for (int y = 0; y <= d; ++y) {
				float u = (x - d/2) * (0.99+i/(float)(NUM_GRID_LEVEL-1)) / (float)d;
				float v = (y - d/2) * (0.99+i/(float)(NUM_GRID_LEVEL-1)) / (float)d;
				gridPoints[i].append(MFloatVector(u, v, 0));
			}
		}
	}


//	std::cerr << "particle ";
	MBoundingBox bbox;
	for (MTime time = start; time < end; time++)
	{
		std::cerr << "---- Frame " << time << "\n";
		MAnimControl::setCurrentTime(time);
		view.refresh(true, true);

		MMatrix modelview, projection;

		MFloatMatrix pm = cameraFn.projectionMatrix();
		float fd[4][4];
		pm.get(fd);

		projection = MMatrix(fd);
		modelview = cameraPath.inclusiveMatrixInverse();
		
//		view.modelViewMatrix(modelview);
		view.projectionMatrix(projection);

//		std::cerr << modelview << "\n";
//	/	std::cerr << projection << "\n";

		if (time != start)
		{
			pcl::PointCloud<pcl::PointXYZ> source;
			pcl::PointCloud<pcl::PointXYZ> target;

			for (int i = 0; i < (int)points.length(); ++i)
			{
//				std::cerr << "  Points= " << points[i] << "\n";
				MPoint dd = points[i] * modelview * projection;
				dd.cartesianize();
//				std::cerr << "  DD = " << dd <<"\n";
//				std::cerr << spoints[i] << " --- ";
	//			std::cerr << dd << "\n";

				source.push_back(pcl::PointXYZ(-spoints[i].x, spoints[i].y/aspect, 0));
				target.push_back(pcl::PointXYZ(-dd.x, dd.y/aspect, 0));
			}

//			MMatrix mat = estimateTransfrom(source, target);
			MMatrix mat = estimateTransfrom(target, source);

			// jump check
			if (matrices.length() > 0) {
				if (isJumped(mat, prevMat)) {
					std::cerr << "Jump [" << time << " ! " << mat << "\n    " << prevMat << "\n";
					goto next;
				}
			}

			prevMat = mat;
			cumMat = cumMat * mat;

			matrices.append(cumMat);

			MPoint p = MPoint(-1, -1/aspect) * cumMat;
			bbox.expand(p);

			p = MPoint( 1, -1/aspect) * cumMat;
			bbox.expand(p);

			p = MPoint( 1,  1/aspect) * cumMat;
			bbox.expand(p);

			p = MPoint(-1,  1/aspect) * cumMat;
			bbox.expand(p);
		}

//		MMatrix mat = projection.inverse() * modelview.inverse();


//		std::cerr << "particle ";

		int max_sample = 4;
		int try_count = 0;
		int pre_count = 0;
		points.clear();
		spoints.clear();

		while(points.length() < max_sample && try_count < NUM_GRID_LEVEL) {


			points.clear();
			spoints.clear();
			for (int i = 0; i < gridPoints[try_count].length(); ++i) {
				MPoint p;
				p.x = gridPoints[try_count][i].x;
				p.y = gridPoints[try_count][i].y;
				p.z = 0;

				float buffer[1];
				buffer[0] = 1.0;

				int xx = width*(p.x*0.5+0.5);
				int yy = height*(p.y*0.5+0.5);
				view.readDepthMap(xx, yy, 1, 1, (unsigned char*)buffer, M3dView::kDepth_Float);

				if (buffer[0] != 1.0){
	//				std::cerr << xx << ", " << yy << ", " << buffer[0] << "\n";
				}
				if (buffer[0] == 1.0 || buffer[0] == 0.5) continue;

				// Hit a sample

				//std::cerr << "   (" << buffer[0] << ") \n";
				p.z = 2 * buffer[0] - 1.0;

				cumulativeDepth += p.z;
				cumulativeDepthCount++;

//				std::cerr << "\n" << "screen = " << p.x << ", " << p.y << ", " << p.z << "\n";
			
				spoints.append(p);

				p = p * projection.inverse() * modelview.inverse();
				p.cartesianize();

				points.append(p);

//				std::cerr << "object = " << p.x << ", " << p.y << ", " << p.z << "\n";
//
//				p *= modelview * projection;
//				p = p.cartesianize();

//				std::cerr << " -p " <<  p[0] << " " << p[1] << " " << p[2];
//				std::cerr << "r screen = " << p.x << ", " << p.y << ", " << p.z << "\n";
			}
			try_count++;
		}
//		std::cerr << "\n";
		next: ;
	}

	// ----------------------------------------------------------------
	glPushAttrib(GL_ALL_ATTRIB_BITS);

    MVector length = bbox.max() - bbox.min();
    double scale[3] = { 1/length.x, 1/length.y, 1 };

	g_width = g_height * (length.x/length.y);

	InitFramebuffer();

	glBindFramebuffer(GL_FRAMEBUFFER, g_frameBuffer);

	glClearColor(1, 1, 1, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);


	GLuint viewTexture;
	glGenTextures(1, &viewTexture);
	glBindTexture(GL_TEXTURE_2D, viewTexture);
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0 );



	MStringArray results;
	char cmd[1024];
	sprintf(cmd, "polyPlane -sh 1 -sw 1 -h 1 -w %f", aspect);
	MGlobal::executeCommand(cmd, results);
	sprintf(cmd, "hide %s", results[0].asChar());
	MGlobal::executeCommand(cmd);
	MString setKeyCmd = "setKeyframe -breakdown 0 -hierarchy none -controlPoints 0 -shape 0 {\"" + results[0] + ".map[0:3]\"};";


	char cmdline[1024];


	MVector c = bbox.center();


	MPoint min = bbox.min();
	MVector len = bbox.max() - bbox.min();

	MAnimControl::setCurrentTime(start);
	view.refresh(true, true);

	for (int i = 0; i < (int)matrices.length(); i++)
	{
		MMatrix mat = matrices[i];

		MPoint p0 = MPoint(-1, -1/aspect) * mat;
		MPoint p1 = MPoint( 1, -1/aspect) * mat;
		MPoint p2 = MPoint( 1,  1/aspect) * mat;
		MPoint p3 = MPoint(-1,  1/aspect) * mat;

		MPoint uv0((p0.x-min.x)/len.x, (p0.y-min.y)/len.y);
		MPoint uv1((p1.x-min.x)/len.x, (p1.y-min.y)/len.y);
		MPoint uv2((p2.x-min.x)/len.x, (p2.y-min.y)/len.y);
		MPoint uv3((p3.x-min.x)/len.x, (p3.y-min.y)/len.y);
		
		sprintf(cmdline, "polyEditUV -r off -u %f -v %f %s.map[0]", 1-uv0.x, uv0.y, results[0].asChar());
		MGlobal::executeCommand(cmdline);
		sprintf(cmdline, "polyEditUV -r off -u %f -v %f %s.map[1]", 1-uv1.x, uv1.y, results[0].asChar());
		MGlobal::executeCommand(cmdline);
		sprintf(cmdline, "polyEditUV -r off -u %f -v %f %s.map[3]", 1-uv2.x, uv2.y, results[0].asChar());
		MGlobal::executeCommand(cmdline);
		sprintf(cmdline, "polyEditUV -r off -u %f -v %f %s.map[2]", 1-uv3.x, uv3.y, results[0].asChar());
		MGlobal::executeCommand(cmdline);

		MGlobal::executeCommand(setKeyCmd);


		MAnimControl::setCurrentTime(start + i);
		view.refresh(true, true);


		glBindTexture(GL_TEXTURE_2D, viewTexture);
		MStatus stat = view.readBufferTo2dTexture(0, 0, width, height);

		glPushMatrix();

		glBindFramebuffer(GL_FRAMEBUFFER, g_frameBuffer);
		glViewport(0, 0, g_width, g_height);


		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(bbox.max().x, bbox.min().x, bbox.min().y, bbox.max().y, -100, 100);
		glMatrixMode(GL_MODELVIEW);

		float d[4][4];
		mat.get(d);
		glLoadIdentity();
		glMultMatrixf((GLfloat*)d);
		
		glColor4f(1, 1, 1, 1);
		glEnable(GL_TEXTURE_2D);
		glDisable(GL_LIGHTING);
#if 1
		glBindTexture(GL_TEXTURE_2D, viewTexture);		
		glBegin(GL_TRIANGLE_FAN);
		glTexCoord2f(0.5, 0.5); glVertex3d(0, 0, 0);
		const float div = 100;
		for (int t = 0; t < div; t++) {
			addvert(1-t/div, 0, aspect);
		}
		for (int t = 0; t < div; t++) {
			addvert(0, t/div, aspect);
		}
		for (int t = 0; t < div; t++) {
			addvert(t/div, 1, aspect);
		}
		for (int t = 0; t < div; t++) {
			addvert(1, 1-t/div, aspect);
		}
		addvert(1, 0, aspect);
		glEnd();

#endif

		glPopAttrib();
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glBindTexture(GL_TEXTURE_2D, 0);

		glPopMatrix();

		glViewport(0, 0, width, height);
	}

	glBindTexture(GL_TEXTURE_2D, 0);

	glDeleteTextures(1, &viewTexture);

	unsigned char *pixels = new unsigned char[g_width * g_height * 4];

	glBindFramebuffer(GL_FRAMEBUFFER, g_frameBuffer);
	glReadPixels(0, 0, g_width, g_height, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	MImage image;
	image.create(g_width, g_height);
	image.setPixels(pixels, g_width, g_height);

	image.writeToFile(filename.asChar(), "png");

	delete [] pixels;

	UninitFramebuffer();

	cumulativeDepth /= cumulativeDepthCount;

	MStringArray cmdResults;
	cmdResults.append(results[0]);
	cmdResults.append(MString() + cumulativeDepth);

	setResult(cmdResults);

	glPopAttrib();
	return MS::kSuccess;
}


__declspec(dllexport) MStatus initializePlugin(MObject obj)
{
	MFnPlugin plugin(obj);
	
	plugin.registerCommand("multiPerspective", MultiPerspective::Creator);

	glewInit();

	return MS::kSuccess;
}

__declspec(dllexport) MStatus uninitializePlugin(MObject obj)
{
	MFnPlugin plugin(obj);

	plugin.deregisterCommand("multiPerspective");

	return MS::kSuccess;

}