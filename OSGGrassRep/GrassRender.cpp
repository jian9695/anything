
#include <Windows.h>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>
#include <osgViewer/viewer>
#include <osg/MatrixTransform>
#include <osg/TextureCubeMap>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>
#include <osg/GLExtensions>
#include <osgViewer/Renderer>
#include <osgGA/TrackballManipulator>
#include <osgDB/WriteFile>
#include <osgViewer/ViewerEventHandlers>
#include "osg/ComputeBoundsVisitor"
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/Point>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>
#include <osg/ShapeDrawable>
#include "ScreenOverlay.h"
int Num_Particles = 65536;

double GetRandomPosition()
{
	return (rand() / (double)RAND_MAX);
}

osg::Texture2D* LoadTexture(const std::string& filename)
{
	osg::Texture2D* tex = new osg::Texture2D;
	tex->setResizeNonPowerOfTwoHint(false);
	tex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
	tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
	tex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
	tex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);
	osg::ref_ptr<osg::Image> img = osgDB::readImageFile(filename);
	tex->setResizeNonPowerOfTwoHint(false);
	tex->setImage(img.get());
	return tex;
}

std::string LoadTextFile(const std::string& filename)
{
	std::ifstream ifs(filename);
	std::string str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
	ifs.close();
	return str;
}

osg::Image* GenerateParticlesMap(int width, int height)
{
	osg::Image* img = new osg::Image;
	img->allocateImage(width, height, 1, GL_RGB, GL_FLOAT);
	int index = 0;
	float* destData = (float*)img->data();
	srand(time(NULL));
	int num = img->s() * img->t();
	for (int i = 0; i < num; i++)
	{
		destData[0] = GetRandomPosition();
		destData[1] = GetRandomPosition();
		destData[2] = 0.0;
		destData += 3;
	}
	return img;
}

osg::Geode* CreatePointArray(int width, int height)
{
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
	osg::ref_ptr <osg::Vec3Array> vertices = new osg::Vec3Array();
	for (int i = 0; i < height; i++)
	{
		float y = (float)i / (height - 1.0);
		for (int j = 0; j < width; j++)
		{
			float x = (float)j / (width - 1.0);
			vertices->push_back(osg::Vec3(x, y, 0));
		}
	}

	geom->setVertexArray(vertices.get());
	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
	geom->setNormalArray(normals.get(), osg::Array::BIND_OVERALL);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));
	osg::Geode* geode = new osg::Geode();
	geode->addDrawable(geom.get());
	return geode;
}

class ScreenRezieEventHandler : public osgGA::GUIEventHandler
{
public:

	ScreenRezieEventHandler() : osgGA::GUIEventHandler()
	{

	}

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
	{
		switch (ea.getEventType())
		{
		case(osgGA::GUIEventAdapter::RESIZE):
		{
			int newWidth = ea.getWindowWidth();
			int newHeight = ea.getWindowHeight();
			break;
		}

		default:
			return false;
		}
	}
};

struct RGB
{
	unsigned char r;
	unsigned char g;
	unsigned char b;
};

void Pixelate(std::string sourceFile, std::string maskFile)
{
	osg::ref_ptr<osg::Image> srcImg = osgDB::readImageFile(sourceFile);
	osg::ref_ptr<osg::Image> outImg = osgDB::readImageFile(sourceFile);
	osg::ref_ptr<osg::Image> img = osgDB::readImageFile(maskFile);
	size_t numPixels = img->s() * img->t();
	bool* mask = new bool[img->s() * img->t()];
	for (size_t i = 0; i < numPixels; i++)
	{
		mask[i] = false;
	}
	std::vector<osg::Vec4i> regions;
	RGB* pRGB = (RGB*)img->data();
	RGB* pRGBSrc = (RGB*)srcImg->data();
	RGB* pRGBOut = (RGB*)outImg->data();
	size_t index = -1;
	for (size_t y = 0; y < img->t(); y++)
	{
		for (size_t x = 0; x < img->s(); x++)
		{
			index++;
			if (mask[index])
				continue;

			RGB rgb = pRGB[index];
			if ((rgb.r + rgb.g + rgb.b) != 255 * 3)
			{
				mask[index] = true;
				continue;
			}

			size_t numInRow = 0;
			for (size_t x1 = x; x1 < img->s(); x1++)
			{
				RGB rgb = pRGB[y * img->s() + x1];
				if ((rgb.r + rgb.g + rgb.b) != 255 * 3)
					break;
				numInRow++;
			}
			if (numInRow < 20 || numInRow > img->s() / 2)
			{
				for (size_t x1 = x; x1 < x + numInRow; x1++)
				{
					mask[y * img->s() + x1] = true;
				}
				continue;
			}
			size_t numRows = 0;
			std::vector<RGB> blockData;
			for (size_t y1 = y; y1 < img->t(); y1++)
			{
				bool isRow = false;
				for (size_t x1 = x; x1 < x + numInRow; x1++)
				{
					RGB rgb = pRGB[y1 * img->s() + x1];
					if ((rgb.r + rgb.g + rgb.b) != 255 * 3)
						break;
					isRow = true;
				}
				if (!isRow)
					break;
				for (size_t x1 = x; x1 < x + numInRow; x1++)
				{
					mask[y1 * img->s() + x1] = true;
					blockData.push_back(pRGBSrc[y1 * img->s() + x1]);
				}
				numRows++;
			}
			if (x == 0 || y == 0)
				continue;

			osg::Vec4i region;
			region.x() = x;
			region.y() = numInRow;
			region.z() = y;
			region.x() = numRows;
			regions.push_back(region);

			//osg::ref_ptr<osg::Image> block = new osg::Image();
			//block->allocateImage(numInRow, numRows, 1, GL_RGB, GL_BYTE);
			//memcpy(block->data(), &blockData[0], blockData.size() * 3);
			size_t windowSize = 25;
			for (size_t by = y; by < y + numRows; by++)
			{
				for (size_t bx = x; bx < x + numInRow; bx++)
				{
					double sumR = 0;
					double sumG = 0;
					double sumB = 0;
					int count = 0;
					for (size_t wy = by - windowSize; wy <= by + windowSize; wy++)
					{
						if (wy < 0 || wy >= img->t())
							continue;
						for (size_t wx = bx - windowSize; wx <= bx + windowSize; wx++)
						{
							if (wx < 0 || wx >= img->s())
								continue;

							RGB rgb = pRGBSrc[wy * img->s() + wx];
							sumR += rgb.r;
							sumG += rgb.g;
							sumB += rgb.b;
							count++;
						}
					}
					RGB rgbOut;
					rgbOut.r = (unsigned char)(sumR / count);
					rgbOut.g = (unsigned char)(sumG / count);
					rgbOut.b = (unsigned char)(sumB / count);
					pRGBOut[by * img->s() + bx] = rgbOut;
				}
			}
		}
	}
	osgDB::writeImageFile(*outImg, "C:/Code/result.bmp");
}

//--------------------------------------------------------------------------
//int main(int argc, char **argv)
//{
//	int viewWidth = 1920;
//	int viewHeight = 1080;
//	osg::ref_ptr<osg::Texture2D> windMap = LoadTexture("data/2016112200.png");
//	int particleTexSize = sqrt(Num_Particles);
//	//Render target for updating the particles
//	osg::ref_ptr<OverlayRenderSurface> updateParticlesSurface = new OverlayRenderSurface(particleTexSize, particleTexSize, GL_RGB32F_ARB, GL_RGB, GL_FLOAT, true);
//	updateParticlesSurface->Overlay()->setProgramName("updateParticlesSurface");
//	updateParticlesSurface->Overlay()->SetFragmentShader(LoadTextFile("shaders/update.frag.glsl"));
//	osg::ref_ptr<osg::Image> particlesMap = GenerateParticlesMap(particleTexSize, particleTexSize);
//	//updateParticlesSurface->Texture()->setImage(particlesMap.get());
//	unsigned int pixelSize = particlesMap->getPixelSizeInBits() / 8;
//	unsigned int imageSize = particlesMap->s() * particlesMap->t() * pixelSize;
//
//	float u_drop_rate = 0.003;
//	float u_speed_factor = 0.25;
//	float u_drop_rate_bump = 0.01;
//	float u_rand_seed = GetRandomPosition();
//	float u_opacity = 0.996;
//	osg::Vec2 u_wind_min(-20, -20);
//	osg::Vec2 u_wind_max(20, 20);
//	osg::Vec2 u_wind_res(windMap->getImage()->s(), windMap->getImage()->t());
//	//Render target for storing the particles
//	osg::ref_ptr<OverlayRenderSurface> updateParticlesSurfaceStorage = new OverlayRenderSurface(particleTexSize, particleTexSize, GL_RGB32F_ARB, GL_RGB, GL_FLOAT, true);
//	updateParticlesSurfaceStorage->Overlay()->setProgramName("updateParticlesSurfaceStorage");
//	updateParticlesSurfaceStorage->Texture()->setImage(particlesMap.get());
//	//memcpy(updateParticlesSurfaceStorage->Image()->data(), particlesMap->data(), imageSize);
//	//Store the updated particles
//	updateParticlesSurface->Overlay()->SetTextureLayer(updateParticlesSurfaceStorage->Texture().get(), 0);
//	updateParticlesSurface->Overlay()->SetTextureLayer(windMap.get(), 1);
//	updateParticlesSurface->getOrCreateStateSet()->addUniform(new osg::Uniform("u_particles", 0));
//	updateParticlesSurface->getOrCreateStateSet()->addUniform(new osg::Uniform("u_wind", 1));
//	updateParticlesSurface->getOrCreateStateSet()->addUniform(new osg::Uniform("u_wind_res", u_wind_res));
//	updateParticlesSurface->getOrCreateStateSet()->addUniform(new osg::Uniform("u_wind_min", u_wind_min));
//	updateParticlesSurface->getOrCreateStateSet()->addUniform(new osg::Uniform("u_wind_max", u_wind_max));
//	updateParticlesSurface->getOrCreateStateSet()->addUniform(new osg::Uniform("u_rand_seed", u_rand_seed));
//	updateParticlesSurface->getOrCreateStateSet()->addUniform(new osg::Uniform("u_speed_factor", u_speed_factor));
//	updateParticlesSurface->getOrCreateStateSet()->addUniform(new osg::Uniform("u_drop_rate", u_drop_rate));
//	updateParticlesSurface->getOrCreateStateSet()->addUniform(new osg::Uniform("u_drop_rate_bump", u_drop_rate_bump));
//
//	updateParticlesSurfaceStorage->Overlay()->SetTextureLayer(updateParticlesSurface->Texture().get(), 0);
//
//	osg::ref_ptr<RenderSurface> drawParticlesSurface = new RenderSurface(viewWidth, viewHeight, GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, false);
//	osg::ref_ptr<OverlayRenderSurface> drawParticlesSurfaceStorage = new OverlayRenderSurface(viewWidth, viewHeight, GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, false);
//	drawParticlesSurfaceStorage->Overlay()->SetTextureLayer(drawParticlesSurface->Texture().get(), 0);
//	osg::ref_ptr<osg::Geode> particlesNode = CreatePointArray(particleTexSize, particleTexSize);
//	ProgramBinder binder("draw_particles", particlesNode->getOrCreateStateSet());
//	binder.SetVertexShader(LoadTextFile("shaders/draw.vert.glsl"));
//	binder.SetFragmentShader(LoadTextFile("shaders/draw.frag.glsl"));
//	particlesNode->getOrCreateStateSet()->setTextureAttributeAndModes(0, updateParticlesSurface->Texture().get(), osg::StateAttribute::ON);
//	particlesNode->getOrCreateStateSet()->setTextureAttributeAndModes(1, windMap.get(), osg::StateAttribute::ON);
//	particlesNode->getOrCreateStateSet()->addUniform(new osg::Uniform("u_particles", 0));
//	particlesNode->getOrCreateStateSet()->addUniform(new osg::Uniform("u_wind", 1));
//	particlesNode->getOrCreateStateSet()->addUniform(new osg::Uniform("u_wind_min", u_wind_min));
//	particlesNode->getOrCreateStateSet()->addUniform(new osg::Uniform("u_wind_max", u_wind_max));
//	particlesNode->getOrCreateStateSet()->addUniform(new osg::Uniform("u_opacity", u_opacity));
//
//	osg::ref_ptr<ScreenOverlay> particlesOverlay = new ScreenOverlay();
//	particlesOverlay->setProgramName("particlesOverlay");
//	particlesOverlay->SetTextureLayer(drawParticlesSurfaceStorage->Texture().get(), 0);
//	particlesOverlay->SetFragmentShader(LoadTextFile("shaders/screen.frag.glsl"));
//	particlesOverlay->getOrCreateStateSet()->addUniform(new osg::Uniform("u_screen", 0));
//	particlesOverlay->getOrCreateStateSet()->addUniform(new osg::Uniform("u_opacity", u_opacity));
//
//	particlesOverlay->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
//	particlesNode->getOrCreateStateSet()->setRenderBinDetails(2, "RenderBin");
//	drawParticlesSurface->addChild(particlesOverlay.get());
//	drawParticlesSurface->addChild(particlesNode.get());
//
//	updateParticlesSurface->setRenderOrder(osg::Camera::PRE_RENDER, 0);
//	updateParticlesSurfaceStorage->setRenderOrder(osg::Camera::PRE_RENDER, 1);
//	drawParticlesSurface->setRenderOrder(osg::Camera::PRE_RENDER, 2);
//	drawParticlesSurfaceStorage->setRenderOrder(osg::Camera::PRE_RENDER, 3);
//
//	//Render to screen
//	osg::ref_ptr<ScreenOverlay> baseMapScreenOverlay = new ScreenOverlay();
//	baseMapScreenOverlay->setProgramName("baseMapScreenOverlay");
//	osg::ref_ptr<osg::Texture2D> baseMap = LoadTexture("data/BlueMarble.jpg");
//	baseMapScreenOverlay->SetTextureLayer(baseMap.get());
//
//	osg::ref_ptr<ScreenOverlay> particlesScreenOverlay = new ScreenOverlay();
//	particlesScreenOverlay->setProgramName("particlesScreenOverlay");
//	particlesScreenOverlay->SetTextureLayer(drawParticlesSurface->Texture().get());
//
//	osgViewer::Viewer viewer;
//	//viewer.setUpViewOnSingleScreen(1);
//	viewer.setUpViewInWindow(50, 50, viewWidth, viewHeight);
//	osg::ref_ptr<osg::Group> root = new osg::Group;
//	root->addChild(updateParticlesSurface.get());
//	root->addChild(updateParticlesSurfaceStorage.get());
//	root->addChild(drawParticlesSurface.get());
//	root->addChild(drawParticlesSurfaceStorage.get());
//	root->addChild(baseMapScreenOverlay.get());
//	root->addChild(particlesScreenOverlay.get());
//
//	m_surfaces.push_back(drawParticlesSurface.get());
//	m_surfaces.push_back(drawParticlesSurfaceStorage.get());
//
//	baseMapScreenOverlay->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
//	particlesScreenOverlay->getOrCreateStateSet()->setRenderBinDetails(1000, "RenderBin");
//
//	osg::ref_ptr <osg::BlendFunc> fn = new osg::BlendFunc();
//	fn->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::DST_ALPHA);
//	drawParticlesSurface->getOrCreateStateSet()->setAttributeAndModes(fn.get(), osg::StateAttribute::ON);
//	drawParticlesSurface->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
//	root->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
//	root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
//
//	viewer.setSceneData(root.get());
//	viewer.setThreadingModel(osgViewer::ViewerBase::ThreadingModel::ThreadPerContext);
//	// add the state manipulator
//	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
//	viewer.setCameraManipulator(new osgGA::TrackballManipulator);
//	// add the thread model handler
//	viewer.addEventHandler(new osgViewer::ThreadingHandler);
//
//	// add the window size toggle handler
//	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
//
//	// add the stats handler
//	viewer.addEventHandler(new osgViewer::StatsHandler);
//
//	// add the LOD Scale handler
//	viewer.addEventHandler(new osgViewer::LODScaleHandler);
//
//	// add the screen capture handler
//	viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);
//
//	viewer.addEventHandler(new ScreenRezieEventHandler);
//	std::vector<osg::Vec3> oldParticles(particleTexSize * particleTexSize);
//	std::vector<osg::Vec3> newParticles(particleTexSize* particleTexSize);
//	osg::Vec3 oldParticle;
//	osg::Vec3 newParticle;
//	size_t frameCount  = 0;
//	while (!viewer.done())
//	{
//		frameCount++;
//		//if (frameCount % 100000000 != 0)
//		//	continue;
//		//frameCount = 0;
//		//printf("%d\n", frameCount);
//		viewer.frame();
//		if (frameCount % 30 != 0)
//			continue;
//		memcpy(&newParticles[0], updateParticlesSurface->Image()->data(), particleTexSize * particleTexSize * sizeof(osg::Vec3));
//		memcpy(&oldParticles[0], updateParticlesSurfaceStorage->Image()->data(), particleTexSize * particleTexSize * sizeof(osg::Vec3));
//		//memcpy(&oldParticles[0], &newParticles[0], particleTexSize * particleTexSize * sizeof(osg::Vec3));
//		osg::Vec3* pNewParticles = (osg::Vec3*)updateParticlesSurface->Image()->data();
//	  newParticle = pNewParticles[particleTexSize * 25 + particleTexSize / 2];
//		oldParticle = newParticle;
//	}
//	return viewer.run();
//}

bool intersectPlane(const osg::Vec3f& n, const osg::Vec3f& p0, const osg::Vec3f& l0, const osg::Vec3f& l, float& t)
{
	// assuming vectors are all normalized
	float denom = n * l;
	if (abs(denom) > 1e-6) {
		osg::Vec3f p0l0 = p0 - l0;
		t = p0l0 * n / denom;
		return (t >= 1e-6);
	}

	return false;
}

void TestRayPlaneIntersection()
{
	osg::Vec3f planeNormal(0, 0, 1);
	osg::Vec3f planePoint(0, 0, 0);
	osg::Vec3f rayOrigin(0, 0, 10000);
	osg::Vec3f rayDir(0, 0, -1);
	float intersectDist = 0.0;
	bool intersects = false;
	intersects = intersectPlane(planeNormal, planePoint, rayOrigin, rayDir, intersectDist);
	intersects = intersectPlane(planeNormal, planePoint, rayOrigin, rayDir, intersectDist);

	rayOrigin = osg::Vec3f(0, 0, 10000);
	rayDir = osg::Vec3f(0, 0, 5000);
	rayDir.normalize();
	intersects = intersectPlane(planeNormal, planePoint, rayOrigin, rayDir, intersectDist);
	intersects = intersectPlane(planeNormal, planePoint, rayOrigin, rayDir, intersectDist);
}

//int main(int argc, char** argv)
//{
//	//TestRayPlaneIntersection();
//	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
//	osg::Vec3Array* vertices = new osg::Vec3Array();
//	osg::Vec2Array* uvs = new osg::Vec2Array();
//	osg::Vec3Array* normals = new osg::Vec3Array();
//	normals->push_back(osg::Vec3(0, -1, 0));
//
//	int height = 50;
//	int strips = 10;
//	int stripH = height / strips;
//	for (size_t n = 0; n < 10; n++)
//	{
//		double rootX = -50 + GetRandomPosition() * 50;
//		double rootY = -50 + GetRandomPosition() * 50;
//		int baseH = 0;
//		for (size_t i = 0; i < strips; i++)
//		{
//			vertices->push_back(osg::Vec3(-1 + rootX, 0 + rootY, baseH));
//			uvs->push_back(osg::Vec2(0, baseH / (float)height));
//			vertices->push_back(osg::Vec3(-1 + rootX, 0 + rootY, baseH + stripH));
//			uvs->push_back(osg::Vec2(0, (baseH + stripH) / (float)height));
//			vertices->push_back(osg::Vec3(1 + rootX, 0 + rootY, baseH + stripH));
//			uvs->push_back(osg::Vec2(1, (baseH + stripH) / (float)height));
//			vertices->push_back(osg::Vec3(-1 + rootX, 0 + rootY, baseH));
//			uvs->push_back(osg::Vec2(0, baseH / (float)height));
//			vertices->push_back(osg::Vec3(1 + rootX, 0 + rootY, baseH + stripH));
//			uvs->push_back(osg::Vec2(1, (baseH + stripH) / (float)height));
//			vertices->push_back(osg::Vec3(1 + rootX, 0 + rootY, baseH));
//			uvs->push_back(osg::Vec2(1, baseH / (float)height));
//			baseH += stripH;
//		}
//	}
//
//	//geom->setVertexArray(vertices);
//	//geom->setTexCoordArray(0, uvs);
//	geom->setVertexAttribArray(0, vertices);
//	geom->setVertexAttribBinding(0, osg::Geometry::BIND_PER_VERTEX);
//	geom->setVertexAttribArray(1, uvs);
//	geom->setVertexAttribBinding(1, osg::Geometry::BIND_PER_VERTEX);
//	//geom->setVertexAttribArray(1, uvs);
//	//geom->setVertexAttribArray(3, uvs, osg::Array::BIND_PER_VERTEX);
//	//geom->setNormalArray(normals);
//	//geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
//	//geom->setCullingActive(false);
//	//geom->getOrCreateStateSet()->setMode(GL_CULL_FACE,
//	//osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
//	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, vertices->getNumElements()));
//	geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
//
//	osg::ref_ptr<osg::Texture2D> grassTex = LoadTexture("C:/Code/Grass.DirectX-master/Visual Studio/Content/Textures/Grass/grassBlade.png");
//	osg::ref_ptr<osg::Texture2D> grassAlpha = LoadTexture("C:/Code/Grass.DirectX-master/Visual Studio/Content/Textures/Grass/grassBladeAlpha2.png");
//	//osg::ref_ptr<osg::Texture2D> grassTex = LoadTexture("C:/Code/GrassOSG/bin/shaders/grass_grid.png");
//	//osg::ref_ptr<osg::Texture2D> grassAlpha = LoadTexture("C:/Code/GrassOSG/bin/shaders/heightmap.png");
//
//	osgViewer::Viewer viewer;
//	//viewer.realize();
//	//viewer.getCamera()->getGraphicsContext()->getState()->setUseVertexAttributeAliasing(true);
//	//viewer.setUpViewOnSingleScreen(1);
//	viewer.setUpViewInWindow(50, 50, 1024, 768);
//	osg::ref_ptr<osg::Geode> root = new osg::Geode;
//	root->addDrawable(geom.get());
//	osg::ref_ptr< osg::Program> program = new osg::Program;
//	osg::ref_ptr<osg::Shader> vertexShader = new osg::Shader(osg::Shader::VERTEX, LoadTextFile("C:/Code/GrassOSG/bin/shaders/grass.vert.glsl"));
//	program->addShader(vertexShader.get());
//
//	osg::ref_ptr<osg::Shader> fragmentShader = new osg::Shader(osg::Shader::FRAGMENT, LoadTextFile("C:/Code/GrassOSG/bin/shaders/grass.frag.glsl"));
//	program->addShader(fragmentShader.get());
//
//	program->addBindFragDataLocation("position", 0);
//	program->addBindFragDataLocation("texCoords", 1);
//	geom->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON);
//	geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, grassTex.get(), osg::StateAttribute::ON);
//	geom->getOrCreateStateSet()->setTextureAttributeAndModes(1, grassAlpha.get(), osg::StateAttribute::ON);
//	geom->getOrCreateStateSet()->addUniform(new osg::Uniform("u_grassTex", 0));
//	geom->getOrCreateStateSet()->addUniform(new osg::Uniform("u_grassAlpha", 1));
//	geom->getOrCreateStateSet()->getOrCreateUniform("osg_FrameTime", osg::Uniform::FLOAT);
//	geom->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4);
//	geom->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
//	geom->getOrCreateStateSet()->getOrCreateUniform("model", osg::Uniform::FLOAT_MAT4);
//
//	osg::ref_ptr <osg::BlendFunc> fn = new osg::BlendFunc();
//	fn->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::DST_ALPHA);
//	//geom->getOrCreateStateSet()->setAttributeAndModes(fn.get(), osg::StateAttribute::ON);
//	geom->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
//	//root->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
//	root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
//	osg::ref_ptr<osg::AlphaFunc> alphaFunc = new osg::AlphaFunc();
//	alphaFunc->setFunction(osg::AlphaFunc::ComparisonFunction::GREATER, 0.1);
//	geom->getOrCreateStateSet()->setAttributeAndModes(alphaFunc.get(), osg::StateAttribute::ON);
//	root->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
//	viewer.setSceneData(root.get());
//	viewer.setThreadingModel(osgViewer::ViewerBase::ThreadingModel::ThreadPerContext);
//	// add the state manipulator
//	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
//	viewer.setCameraManipulator(new osgGA::TrackballManipulator);
//	// add the thread model handler
//	viewer.addEventHandler(new osgViewer::ThreadingHandler);
//	
//	// add the window size toggle handler
//	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
//
//	// add the stats handler
//	viewer.addEventHandler(new osgViewer::StatsHandler);
//
//	// add the LOD Scale handler
//	viewer.addEventHandler(new osgViewer::LODScaleHandler);
//
//	// add the screen capture handler
//	viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);
//	
//	viewer.addEventHandler(new ScreenRezieEventHandler);
//
//	float frameCount = 0;
//	osg::FloatArray* projectArr = new osg::FloatArray(16);
//	osg::FloatArray* viewArr = new osg::FloatArray(16);
//	while (!viewer.done())
//	{
//		frameCount = frameCount + 0.02;
//		
//		geom->getOrCreateStateSet()->getOrCreateUniform("osg_FrameTime", osg::Uniform::FLOAT)->set((float)frameCount);
//		osg::Matrix project = viewer.getCamera()->getProjectionMatrix();
//		double* pProj = (double*)project.ptr();
//		for (int n = 0; n < 16; n++)
//		{
//			(*projectArr)[n] = (float)pProj[n];
//		}
//
//		osg::Matrix view = viewer.getCamera()->getViewMatrix();
//		double* pView = (double*)view.ptr();
//		for (int n = 0; n < 16; n++)
//		{
//			(*viewArr)[n] = (float)pView[n];
//		}
//
//		geom->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4)->setArray(projectArr);
//		geom->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->setArray(viewArr);
//
//		viewer.frame();
//	}
//
//	return viewer.run();
//}

class GrassObject : public osg::Geometry
{
private:
	osg::BoundingBox m_bb;

public:
	GrassObject(osg::BoundingBox bb)
		:m_bb(bb)
	{}

	/** Compute the bounding sphere around Drawables's geometry.*/
	osg::BoundingSphere computeBound() const
	{
		return osg::BoundingSphere(m_bb);
	}

	/** Compute the bounding box around Drawables's geometry.*/
	osg::BoundingBox computeBoundingBox() const
	{
		return m_bb;
	}
};


osg::Geometry* CreateWiredBox(const osg::BoundingBox& bb)
{
	osg::Geometry* geom = new osg::Geometry;
	osg::Vec3Array* vertices = new osg::Vec3Array();
	for (size_t i = 0; i < 8; i++)
	{
		osg::Vec3 p0 = bb.corner(i);
		for (size_t j = 0; j < 8; j++)
		{
			osg::Vec3 p1 = bb.corner(j);
			int numEquals = 0;
			if (p0.x() == p1.x())
				numEquals++;
			if (p0.y() == p1.y())
				numEquals++;
			if (p0.z() == p1.z())
				numEquals++;
			if (numEquals < 2)
				continue;
			vertices->push_back(p0);
			vertices->push_back(p1);
		}
	}
	geom->setVertexArray(vertices);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertices->getNumElements()));
	geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	return geom;
}

osg::Geometry* CreateSolidBox(const osg::BoundingBox& bb)
{
	osg::ref_ptr<osg::Material> material = new osg::Material();
	material->setDiffuse(osg::Material::Face::FRONT_AND_BACK, osg::Vec4(0.2, 0.2, 0.2, 1.0));
	//material->setEmission(osg::Material::Face::FRONT_AND_BACK, osg::Vec4(0.5, 0.5, 0.5, 1.0));
	//material->setShininess(osg::Material::Face::FRONT_AND_BACK, 0.5);
	material->setAmbient(osg::Material::Face::FRONT_AND_BACK, osg::Vec4(0.2, 0.2, 0.2, 1.0));
	material->setSpecular(osg::Material::Face::FRONT_AND_BACK, osg::Vec4(0.2, 0.2, 0.2, 1.0));
	osg::ShapeDrawable* boundingboxHull = new osg::ShapeDrawable;
	boundingboxHull->setShape(new osg::Box(osg::Vec3(0.0f, 0.0f, 0.0f), bb.xMax() - bb.xMin(), bb.yMax() - bb.yMin(), bb.zMax() - bb.zMin()));
	boundingboxHull->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);
	boundingboxHull->getOrCreateStateSet()->setAttribute(material.get(), osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);
	return boundingboxHull;
}

int main(int argc, char** argv)
{
	//TestRayPlaneIntersection();
	float scale = 100;
	osg::Vec3 sceneSize(scale, scale, scale);
	osg::BoundingBox sceneBB(-sceneSize.x() * 0.5, -sceneSize.y() * 0.5, -sceneSize.z() * 0.5, sceneSize.x() * 0.5, sceneSize.y() * 0.5, sceneSize.z() * 0.5);
	//osg::ref_ptr<osg::ShapeDrawable> boundingboxHull = new osg::ShapeDrawable;
	//boundingboxHull->setShape(new osg::Box(osg::Vec3(0.0f, 0.0f, 0.0f), sceneSize.x(), sceneSize.y(), sceneSize.z()));
	//osg::PolygonMode* polygonMode = new osg::PolygonMode(osg::PolygonMode::Face::FRONT_AND_BACK, osg::PolygonMode::Mode::LINE);
	//boundingboxHull->getOrCreateStateSet()->setAttributeAndModes(polygonMode, osg::StateAttribute::ON);
	osg::Vec3 terrainSize(scale, scale, 2);
	osg::BoundingBox terrainBB(-terrainSize.x() * 0.5, -terrainSize.y() * 0.5, -terrainSize.z() * 0.5, terrainSize.x() * 0.5, sceneSize.y() * 0.5, terrainSize.z() * 0.5);
	//osg::ref_ptr<osg::Geometry> boundingboxHull = CreateWiredBox(sceneBB);
	osg::ref_ptr<osg::Geometry> boundingboxHull = CreateSolidBox(terrainBB);
	osg::ref_ptr<osg::MatrixTransform> terrain = new osg::MatrixTransform;
	terrain->addChild(boundingboxHull.get());
	terrain->setMatrix(osg::Matrix::translate(osg::Vec3(0, 0, -5)));

	osg::ref_ptr<GrassObject> geom = new GrassObject(sceneBB);
	osg::Vec3Array* vertices = new osg::Vec3Array();
	vertices->push_back(osg::Vec3(-1 * scale * 0.5, -1 * scale * 0.5, 0));
	vertices->push_back(osg::Vec3(-1 * scale * 0.5, 1 * scale * 0.5, 0));
	vertices->push_back(osg::Vec3(1 * scale * 0.5, -1 * scale * 0.5, 0));
	vertices->push_back(osg::Vec3(1 * scale * 0.5, -1 * scale * 0.5, 0));
	vertices->push_back(osg::Vec3(-1 * scale * 0.5, 1 * scale * 0.5, 0));
	vertices->push_back(osg::Vec3(1 * scale * 0.5, 1 * scale * 0.5, 0));

	//geom->setVertexArray(vertices);
	//geom->setTexCoordArray(0, uvs);
	geom->setVertexAttribArray(0, vertices);
	geom->setVertexAttribBinding(0, osg::Geometry::BIND_PER_VERTEX);
	//geom->setVertexAttribArray(1, uvs);
	//geom->setVertexAttribArray(3, uvs, osg::Array::BIND_PER_VERTEX);
	//geom->setNormalArray(normals);
	//geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
	//geom->setCullingActive(false);
	//geom->getOrCreateStateSet()->setMode(GL_CULL_FACE,
	//osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, vertices->getNumElements()));
	geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osg::ref_ptr<osg::Texture2D> grassTex = LoadTexture("./data/grass.png");
	//osg::ref_ptr<osg::Texture2D> grassAlpha = LoadTexture("./bin/data/grass_0_90.png");
	//osg::ref_ptr<osg::Texture2D> grassTex = LoadTexture("C:/Code/GrassOSG/bin/shaders/grass_grid.png");
	//osg::ref_ptr<osg::Texture2D> grassAlpha = LoadTexture("C:/Code/GrassOSG/bin/shaders/heightmap.png");

	osgViewer::Viewer viewer;
	//viewer.realize();
	//viewer.getCamera()->getGraphicsContext()->getState()->setUseVertexAttributeAliasing(true);
	//viewer.setUpViewOnSingleScreen(1);
	viewer.setUpViewInWindow(50, 50, 1024, 768);
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geom.get());
	osg::ref_ptr<osg::Group> root = new osg::Geode;
	root->addChild(geode.get());
	root->addChild(terrain.get());

	//osg::ref_ptr<RenderSurface> renderCamera = new RenderSurface(512, 512, GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, true);
	//renderCamera->addChild(geode.get());
	//renderCamera->addChild(terrain.get());
	//renderCamera->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4);
	//renderCamera->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());

	osg::ref_ptr< osg::Program> program = new osg::Program;
	osg::ref_ptr<osg::Shader> vertexShader = new osg::Shader(osg::Shader::VERTEX, LoadTextFile("./shaders/render_grass.vert.glsl"));
	program->addShader(vertexShader.get());

	osg::ref_ptr<osg::Shader> fragmentShader = new osg::Shader(osg::Shader::FRAGMENT, LoadTextFile("./shaders/render_grass.frag.glsl"));
	program->addShader(fragmentShader.get());

	program->addBindFragDataLocation("position", 0);
	program->addBindFragDataLocation("texCoords", 1);
	geom->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON);
	geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, grassTex.get(), osg::StateAttribute::ON);
	//geom->getOrCreateStateSet()->setTextureAttributeAndModes(1, grassAlpha.get(), osg::StateAttribute::ON);
	geom->getOrCreateStateSet()->addUniform(new osg::Uniform("u_grassTex", 0));
	//geom->getOrCreateStateSet()->addUniform(new osg::Uniform("u_grassAlpha", 1));
	geom->getOrCreateStateSet()->getOrCreateUniform("osg_FrameTime", osg::Uniform::FLOAT);
	geom->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4);
	geom->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	geom->getOrCreateStateSet()->getOrCreateUniform("model", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());

	osg::ref_ptr <osg::BlendFunc> fn = new osg::BlendFunc();
	fn->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::DST_ALPHA);
	//geom->getOrCreateStateSet()->setAttributeAndModes(fn.get(), osg::StateAttribute::ON);
	geom->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	//root->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	osg::ref_ptr<osg::AlphaFunc> alphaFunc = new osg::AlphaFunc();
	alphaFunc->setFunction(osg::AlphaFunc::ComparisonFunction::GREATER, 0.1);
	geom->getOrCreateStateSet()->setAttributeAndModes(alphaFunc.get(), osg::StateAttribute::ON);
	root->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	viewer.setSceneData(root.get());
	viewer.setThreadingModel(osgViewer::ViewerBase::ThreadingModel::ThreadPerContext);
	// add the state manipulator
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	viewer.setCameraManipulator(new osgGA::TrackballManipulator);
	// add the thread model handler
	viewer.addEventHandler(new osgViewer::ThreadingHandler);

	// add the window size toggle handler
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);

	// add the stats handler
	viewer.addEventHandler(new osgViewer::StatsHandler);

	// add the LOD Scale handler
	viewer.addEventHandler(new osgViewer::LODScaleHandler);

	// add the screen capture handler
	viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);

	viewer.addEventHandler(new ScreenRezieEventHandler);

	float frameCount = 0;
	osg::FloatArray* projectArr = new osg::FloatArray(16);
	osg::FloatArray* viewArr = new osg::FloatArray(16);
	osg::FloatArray* modelArr = new osg::FloatArray(16);
	while (!viewer.done())
	{
		frameCount = frameCount + 0.02;

		geom->getOrCreateStateSet()->getOrCreateUniform("osg_FrameTime", osg::Uniform::FLOAT)->set((float)frameCount);
		osg::Matrix project = viewer.getCamera()->getProjectionMatrix();
		double* pProj = (double*)project.ptr();
		for (int n = 0; n < 16; n++)
		{
			(*projectArr)[n] = (float)pProj[n];
		}

		osg::Matrix view = viewer.getCamera()->getViewMatrix();
		double* pView = (double*)view.ptr();
		for (int n = 0; n < 16; n++)
		{
			(*viewArr)[n] = (float)pView[n];
		}

		geom->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4)->setArray(projectArr);
		geom->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->setArray(viewArr);
		//geom->getOrCreateStateSet()->getOrCreateUniform("model", osg::Uniform::FLOAT_MAT4)->setArray(viewArr);

		viewer.frame();
	}

	return viewer.run();
}