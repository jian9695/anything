
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
	//tex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR_MIPMAP_NEAREST);
	//tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR_MIPMAP_NEAREST);
	tex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::CLAMP_TO_EDGE);
	tex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::CLAMP_TO_EDGE);
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
	int num= img->s() * img->t();
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

osg::Node* m_grassland = nullptr;
bool m_isMouseDown = false;
RenderSurface* m_renderCircleCamera = nullptr;
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
				if (m_renderCircleCamera)
					m_renderCircleCamera->resize(newWidth, newHeight);
				break;
			}
			case(osgGA::GUIEventAdapter::KEYDOWN):
			{
				if (m_grassland && ea.getKey() == osgGA::GUIEventAdapter::KEY_G)
				{
					bool val = false;
					m_grassland->getOrCreateStateSet()->getOrCreateUniform("showGrid", osg::Uniform::BOOL)->get(val);
					m_grassland->getOrCreateStateSet()->getOrCreateUniform("showGrid", osg::Uniform::BOOL)->set(!val);
				}
				break;
			}
		default:
			return false;
		}
		return false;
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

	GrassObject(){}

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
  boundingboxHull->setShape(new osg::Box(osg::Vec3(0.0f, 0.0f, 0.0f), bb.xMax()-bb.xMin(), bb.yMax() - bb.yMin(), bb.zMax() - bb.zMin()));
	boundingboxHull->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);
	boundingboxHull->getOrCreateStateSet()->setAttribute(material.get(), osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);
	return boundingboxHull;
}

double CalPerimeter(double rx, double ry)
{
	return 2 * 3.14159 * sqrt((rx * rx + ry * ry) / 2);
}

void DrawEllipse()
{
	double height = 10;
	double perimeter = height * 4;
	float r = perimeter / (2 * 3.14159);
	double ry = 5;
	double rx = sqrt(pow(perimeter / (2 * 3.14159), 2) * 2 - ry * ry);
	ry = rx = r;
	double peri = CalPerimeter(rx, ry);
	peri = CalPerimeter(rx, ry);
	double cx = rx;
	double cy = 0;
	double step = 10;
	double angle = 180;
	while (angle >= 90)
	{

		double theta = angle * 3.1415926 / 180;
		double vx = cos(theta) * rx + cx;
		double vy = sin(theta) * ry + cy;

		osg::Vec2 pa(0, (180 - angle) / 90 * height);
		//vec0.normalize();
		osg::Vec2 pb(vx, vy);
		osg::Vec2 vec0 = pa - pb;
		double dist = vec0.length();
		dist = vec0.length();
		vec0.normalize();
		osg::Vec2 vec1(-1, 0);
		//vec1.normalize();
		double dot = vec0 * vec1;
		double sumMag = vec0.length() * vec1.length();
		double angleBetween = acos(dot / sumMag);
		angleBetween = angleBetween * 180 / 3.1415926;
		//angleBetween = angleBetween;

		osg::Vec3 windir(1, 1, 0);
		windir.normalize();
		osg::Vec3 axis = windir ^ osg::Vec3(0, 0, -1);
		axis = windir ^ osg::Vec3(0, 0, -1);
		osg::Matrix rotate = osg::Matrix::rotate(angleBetween, axis);
		windir = windir * rotate;
		windir.normalize();
		osg::Vec3 oriPos(0, 0, height);
		osg::Vec3 newPos = oriPos * rotate;
		newPos = oriPos * rotate;
		//vec3.normalize();
		//printf("%f,%f,%f", vec3.x(), vec3.y(), vec3.z());
		//osg::vec2 v1()

		//System.Diagnostics.Debug.WriteLine(vx.ToString("0.00") + "," + vy.ToString("0.00"));
		//System.Console.WriteLine(vx.ToString("0.00") + "," + vy.ToString("0.00"));
		angle -= step;
	}
	//double theta = 2.0f * 3.1415926f / (double)num_segments;
	//for (int i = 0; i < num_segments; i++)
	//{
	//  double vx = Math.Cos(theta * i) * rx + cx;
	//  double vy = Math.Sin(theta * i) * ry + cy;
	//  System.Diagnostics.Debug.WriteLine(vx.ToString("0.00") + "," + vy.ToString("0.00"));
	//  System.Console.WriteLine(vx.ToString("0.00") + "," + vy.ToString("0.00"));
	//}

	//for (int ii = 0; ii < num_segments; ii++)
	//{
	//  //apply radius and offset
	//  //glVertex2f(x * rx + cx, y * ry + cy);//output vertex 

	//  double vx = ConsoleSpecialKey() * rx + cx;
	//  double vy = y * ry + cy;
	//  System.Diagnostics.Debug.WriteLine(vx.ToString("0.00") + "," + vy.ToString("0.00"));
	//  System.Console.WriteLine(vx.ToString("0.00") + "," + vy.ToString("0.00"));

	//  //apply the rotation matrix
	//  t = x;
	//  x = c * x - s * y;
	//  y = s * t + c * y;
	//}

}

osg::Geometry* createGrassPatch(size_t num, double radius, float flag)
{
	double minBladeWidth = 0.0015875;
	double maxBladeWidth = 0.002;
	double cellSize = maxBladeWidth;
	double gridSize = cellSize * 10;
	double minBladeHeight = gridSize * 0.5;
	double maxBladeHeight = gridSize * 2;
	osg::BoundingBox bb(-radius * 2, -radius * 2, -radius * 2, radius * 2, radius * 2, radius * 2);
	GrassObject* geom = new GrassObject(bb);
	osg::Vec3Array* vertices = new osg::Vec3Array();
	osg::Vec4Array* uvs = new osg::Vec4Array();
	size_t curNum = 0;
	double xmin = -radius;
	double ymin = -radius;
	while (curNum < num)
	{
		double x = xmin + GetRandomPosition() * radius * 2;
		double y = ymin + GetRandomPosition	() * radius * 2;
		//if (sqrt(x * x + y * y) > radius)
		//	continue;
		double z = GetRandomPosition();
		vertices->push_back(osg::Vec3(x, y, z));
		double rotate = GetRandomPosition() * 2 * osg::PI;
		double tilt = GetRandomPosition() * (osg::PI * 0.5) * 0.5;
		double h = minBladeHeight + (maxBladeHeight - minBladeHeight) * GetRandomPosition();
		double w = minBladeWidth + (maxBladeWidth - minBladeWidth) * GetRandomPosition();
		//uvs->push_back(osg::Vec4(GetRandomPosition(), GetRandomPosition(), GetRandomPosition(), 1));
		uvs->push_back(osg::Vec4(rotate, tilt, w, h));
		curNum++;
	}
	//for (size_t n = 0; n < num; n++)
	//{
	//	double r = GetRandomPosition() * radius;
	//	double theta = GetRandomPosition() * 3.1415926 * 2;
	//	double rootX = r * cos(theta);
	//	double rootY = r * sin(theta);
	//	vertices->push_back(osg::Vec3(rootX, rootY, GetRandomPosition()));
	//}
	geom->setVertexAttribArray(0, vertices);
	geom->setVertexAttribBinding(0, osg::Geometry::BIND_PER_VERTEX);

	geom->setVertexAttribArray(1, uvs);
	geom->setVertexAttribBinding(1, osg::Geometry::BIND_PER_VERTEX);
	
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->getNumElements()));
	geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	geom->getOrCreateStateSet()->getOrCreateUniform("ringFlag", osg::Uniform::FLOAT)->set(flag);

	osg::ref_ptr <osg::BlendFunc> fn = new osg::BlendFunc();
	fn->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::DST_ALPHA);
	//geom->getOrCreateStateSet()->setAttributeAndModes(fn.get(), osg::StateAttribute::ON);
	geom->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	osg::ref_ptr<osg::AlphaFunc> alphaFunc = new osg::AlphaFunc();
	alphaFunc->setFunction(osg::AlphaFunc::ComparisonFunction::GREATER, 0.1);
	geom->getOrCreateStateSet()->setAttributeAndModes(alphaFunc.get(), osg::StateAttribute::ON);
	return geom;
}

osg::Node* CreateRect(double xmin, double ymin, double xmax, double ymax, double z)
{
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
	osg::Geode* geode = new osg::Geode();
	osg::Vec3Array* vertices = new osg::Vec3Array();
	osg::Vec3Array* normals = new osg::Vec3Array();
	osg::Vec2Array* uvs = new osg::Vec2Array();
	vertices->push_back(osg::Vec3(xmin, ymin, z));
	uvs->push_back(osg::Vec2(0, 1));
	vertices->push_back(osg::Vec3(xmin, ymax, z));
	uvs->push_back(osg::Vec2(0, 0));
	vertices->push_back(osg::Vec3(xmax, ymin, z));
	uvs->push_back(osg::Vec2(1, 1));
	vertices->push_back(osg::Vec3(xmax, ymin, z));
	uvs->push_back(osg::Vec2(1, 1));
	vertices->push_back(osg::Vec3(xmin, ymax, z));
	uvs->push_back(osg::Vec2(0, 0));
	vertices->push_back(osg::Vec3(xmax, ymax, z));
	uvs->push_back(osg::Vec2(1, 0));
	normals->push_back(osg::Vec3(0, 0, 1));
	geom->setVertexArray(vertices);
	geom->setNormalArray(normals);
	geom->setTexCoordArray(0, uvs);
	geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
	geom->setCullingActive(false);
	geode->setCullingActive(false);
	geode->getOrCreateStateSet()->setMode(GL_CULL_FACE,
		osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, 6));
	geode->addDrawable(geom.get());
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osg::ref_ptr<osg::Texture2D> terrainTex = LoadTexture("./data/terrain.png");
	terrainTex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
	terrainTex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);

	geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, terrainTex.get(), osg::StateAttribute::ON);
	geom->getOrCreateStateSet()->addUniform(new osg::Uniform("u_terrainTex", 0));
	
	geom->getOrCreateStateSet()->getOrCreateUniform("top", osg::Uniform::FLOAT)->set((float)ymax);
	geom->getOrCreateStateSet()->getOrCreateUniform("left", osg::Uniform::FLOAT)->set((float)xmin);
	geom->getOrCreateStateSet()->getOrCreateUniform("right", osg::Uniform::FLOAT)->set((float)xmax);
	geom->getOrCreateStateSet()->getOrCreateUniform("bottom", osg::Uniform::FLOAT)->set((float)ymin);
	geode->getOrCreateStateSet()->getOrCreateUniform("showGrid", osg::Uniform::BOOL)->set(false);

	//geode->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4);
	//geode->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("model", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("circleProjection", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("circleView", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("cameraPosition", osg::Uniform::FLOAT_VEC3);

	osg::ref_ptr< osg::Program> program = new osg::Program;
	osg::ref_ptr<osg::Shader> vertexShader = new osg::Shader(osg::Shader::VERTEX, LoadTextFile("./shaders/simple.vert.glsl"));
	program->addShader(vertexShader.get());

	osg::ref_ptr<osg::Shader> fragmentShader = new osg::Shader(osg::Shader::FRAGMENT, LoadTextFile("./shaders/simple.frag.glsl"));
	program->addShader(fragmentShader.get());
	geode->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON);

	osg::ref_ptr <osg::BlendFunc> fn = new osg::BlendFunc();
	fn->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::DST_ALPHA);
	//geom->getOrCreateStateSet()->setAttributeAndModes(fn.get(), osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	//root->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	//osg::ref_ptr<osg::AlphaFunc> alphaFunc = new osg::AlphaFunc();
	//alphaFunc->setFunction(osg::AlphaFunc::ComparisonFunction::GREATER, 0.1);
	//geode->getOrCreateStateSet()->setAttributeAndModes(alphaFunc.get(), osg::StateAttribute::ON);


	return geode;
}


// The MIT License
// Copyright © 2017 Inigo Quilez
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
float cross2d(osg::Vec2 a, osg::Vec2 b) { return a.x() * b.y() - a.y() * b.x(); }

const int lut[4] = { 1, 2, 0, 1 };

// 0--b--3
// |\
// a c
// |  \
// 1    2
//
osg::Vec3 quadIntersect(osg::Vec3 ro, osg::Vec3 rd, osg::Vec3 v0, osg::Vec3 v1, osg::Vec3 v2, osg::Vec3 v3)
{
	rd.normalize();
	// lets make v0 the origin
	osg::Vec3 a = v1 - v0;
	osg::Vec3 b = v3 - v0;
	osg::Vec3 c = v2 - v0;
	osg::Vec3 p = ro - v0;

	// intersect plane
	osg::Vec3 nor = a ^ b;
	float t = -(p * nor) / (rd * nor);
	if (t < 0.0 || isinf(t) || isnan(t))
		return osg::Vec3(-1.0, -1.0, -1.0);

	// intersection point
	osg::Vec3 pos = p + rd * t;

	// see here: https://www.shadertoy.com/view/lsBSDm

	// select projection plane
	osg::Vec3 mor = osg::Vec3(abs(nor.x()), abs(nor.y()), abs(nor.z()));
	int id = (mor.x() > mor.y() && mor.x() > mor.z()) ? 0 :
		(mor.y() > mor.z()) ? 1 :
		2;

	int idu = lut[id];
	int idv = lut[id + 1];

	// project to 2D
	osg::Vec2 kp = osg::Vec2(pos[idu], pos[idv]);
	osg::Vec2 ka = osg::Vec2(a[idu], a[idv]);
	osg::Vec2 kb = osg::Vec2(b[idu], b[idv]);
	osg::Vec2 kc = osg::Vec2(c[idu], c[idv]);

	// find barycentric coords of the quadrilateral
	osg::Vec2 kg = kc - kb - ka;

	float k0 = cross2d(kp, kb);
	float k2 = cross2d(kc - kb, ka);        // float k2 = cross2d( kg, ka );
	float k1 = cross2d(kp, kg) - nor[id]; // float k1 = cross2d( kb, ka ) + cross2d( kp, kg );

	// if edges are parallel, this is a linear equation
	float u, v;
	if (abs(k2) < 0.00001)
	{
		v = -k0 / k1;
		u = cross2d(kp, ka) / k1;
	}
	else
	{
		// otherwise, it's a quadratic
		float w = k1 * k1 - 4.0 * k0 * k2;
		if (w < 0.0) return osg::Vec3(-1.0, -1.0, -1.0);
		w = sqrt(w);

		float ik2 = 1.0 / (2.0 * k2);

		v = (-k1 - w) * ik2;
		if (v < 0.0 || v>1.0) v = (-k1 + w) * ik2;

		u = (kp.x() - ka.x() * v) / (kb.x() + kg.x() * v);
	}

	if (u < 0.0 || u>1.0 || v < 0.0 || v>1.0)
		return osg::Vec3(-1.0, -1.0, -1.0);

	return osg::Vec3(t, u, v);
}

class GrassBlade
{
public:
	double m_x, m_y, m_w, m_h, m_roll, m_rotate, m_tilt;
	osg::Vec3 m_v0, m_v1, m_v2, m_v3, m_center;
	osg::Matrix m_inverse;
	osg::BoundingBox m_bb;
public:
	GrassBlade() {}

	void Create(double x, double y, double w, double h, double roll, double rotate, double tilt)
	{
		m_x = x;
		m_y = y;
		m_w = w;
		m_h = h;
		m_rotate = rotate;
		m_tilt = tilt;
		m_center = osg::Vec3(m_x, m_y, 0);
		osg::Matrix rotationMatrix = osg::Matrix::rotate(m_roll, osg::Vec3(0, 1, 0));
		rotationMatrix *= osg::Matrix::rotate(m_tilt, osg::Vec3(1, 0, 0));
		rotationMatrix *= osg::Matrix::rotate(m_rotate, osg::Vec3(0, 0, -1));
		rotationMatrix *= osg::Matrix::translate(osg::Vec3(m_x, m_y, 0));
		m_v0 = osg::Vec3(-m_w * 0.5, m_h, 0) * rotationMatrix;
		m_v1 = osg::Vec3(-m_w * 0.5, 0, 0) * rotationMatrix;
		m_v2 = osg::Vec3(m_w * 0.5, 0, 0) * rotationMatrix;
		m_v3 = osg::Vec3(m_w * 0.5, m_h, 0) * rotationMatrix;

		m_bb.init();
		m_bb.expandBy(m_v0);
		m_bb.expandBy(m_v1);
		m_bb.expandBy(m_v2);
		m_bb.expandBy(m_v3);
		m_inverse = osg::Matrix::inverse(rotationMatrix);
	}

	bool IsPointInRect(osg::Vec3 p)
	{
		p = p * m_inverse;
		if (p.x() < -m_w * 0.5 || p.x() > m_w * 0.5 || p.y() < -m_h * 0.5 || p.y() > m_h * 0.5)
			return false;
		return true;
	}
};

struct RGBPixel
{
	unsigned char R, G, B, A;
	RGBPixel()
	{
		R = 0;
		G = 0;
		B = 0;
		A = 0;
	}

	RGBPixel(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
	{
		Set(r, g, b, a);
	}

	void Set(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
	{
		R = r;
		G = g;
		B = b;
		A = a;
	}
};

osg::Group* createGrassGrid(double xmin, double ymin, double xmax, double ymax, double resolution)
{
	double minBladeWidth = 0.005;
	double maxBladeWidth = 0.0075;
	double cellSize = maxBladeWidth;
	double gridSize = 1;
	double minBladeHeight = 0.05;
	double maxBladeHeight = 0.10;

	long bladesPerSqrtM = 10000;
	double gridArea = gridSize * gridSize;
	long numBlades = (long)(bladesPerSqrtM / (1.0 / gridArea));

	double xmin2 = -gridSize * 0.5;
	double xmax2 = gridSize * 0.5;
	double ymin2 = -gridSize * 0.5;
	double ymax2 = gridSize * 0.5;
	std::vector<GrassBlade> blades;
	blades.reserve(numBlades);
	while (blades.size() < numBlades)
	{
		double x = xmin2 + GetRandomPosition() * gridSize;
		double y = ymin2 + GetRandomPosition() * gridSize;
		double roll = GetRandomPosition() * 2 * osg::PI;
		double rotate = GetRandomPosition() * 2 * osg::PI;
		double tilt = (25 + GetRandomPosition() * 65) * 0.0174533;
		double h = minBladeHeight + (maxBladeHeight - minBladeHeight) * GetRandomPosition();
		double w = minBladeWidth + (maxBladeWidth - minBladeWidth) * GetRandomPosition();
		GrassBlade blade;
		blade.Create(x, y, w, h, roll, rotate, tilt);
		if (xmin2 - blade.m_bb.xMin() > gridSize * 0.5
			|| ymin2 - blade.m_bb.yMin() > gridSize * 0.5
			|| blade.m_bb.xMax() - xmax2 > gridSize * 0.5
			|| blade.m_bb.yMax() - ymax2 > gridSize * 0.5)
			continue;

		blades.push_back(blade);
	}
	osg::BoundingBox bb(-gridSize * 2, -gridSize * 2, -gridSize * 2, gridSize * 2, gridSize * 2, gridSize * 2);
	std::vector<GrassObject*> lods;
	for (long lod = 1; lod <= 2; lod++)
	{
		GrassObject* geom = new GrassObject(bb);
		osg::Vec3Array* vertices = new osg::Vec3Array();
		osg::Vec4Array* uvs = new osg::Vec4Array();
		long num = numBlades;
		if (lod != 1)
			num = numBlades / (lod * 2);
		for (long n = 0; n < num; n++)
		{
			GrassBlade& blade = blades[n];
			vertices->push_back(osg::Vec3(blade.m_x, blade.m_y, GetRandomPosition()));
			uvs->push_back(osg::Vec4(blade.m_rotate, blade.m_tilt, blade.m_w * lod, blade.m_h * lod));
		}
		geom->setVertexAttribArray(0, vertices);
		geom->setVertexAttribBinding(0, osg::Geometry::BIND_PER_VERTEX);

		geom->setVertexAttribArray(1, uvs);
		geom->setVertexAttribBinding(1, osg::Geometry::BIND_PER_VERTEX);

		geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->getNumElements()));
		lods.push_back(geom);
	}

	//osg::ref_ptr <osg::BlendFunc> fn = new osg::BlendFunc();
	//fn->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::DST_ALPHA);
	//geom->getOrCreateStateSet()->setAttributeAndModes(fn.get(), osg::StateAttribute::ON);
	//root->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	//osg::ref_ptr<osg::AlphaFunc> alphaFunc = new osg::AlphaFunc();
	//alphaFunc->setFunction(osg::AlphaFunc::ComparisonFunction::GREATER, 0.1);
	//geode->getOrCreateStateSet()->setAttributeAndModes(alphaFunc.get(), osg::StateAttribute::ON);

	//osg::ref_ptr<osg::Texture2D> grassTex = LoadTexture("C:/Code/GrassOSG/bin/shaders/grass_grid.png");
	//osg::ref_ptr<osg::Texture2D> grassAlpha = LoadTexture("C:/Code/GrassOSG/bin/shaders/heightmap.png");
	//osg::Geometry* outerRing = createGrassPatch(1000000, 1000, 1);
	//osg::Geometry* middleRing = createGrassPatch(1000000, 100, 0.8);
	//geode->addDrawable(outerRing);
	//geode->addDrawable(middleRing);

	osg::Group* grassland = new osg::Group();
	grassland->getOrCreateStateSet()->getOrCreateUniform("osg_FrameTime", osg::Uniform::FLOAT);
	grassland->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4);
	grassland->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	grassland->getOrCreateStateSet()->getOrCreateUniform("model", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	grassland->getOrCreateStateSet()->getOrCreateUniform("cameraPosition", osg::Uniform::FLOAT_VEC3);
	osg::ref_ptr< osg::Program> program = new osg::Program;
	osg::ref_ptr<osg::Shader> geomShader = new osg::Shader(osg::Shader::GEOMETRY, LoadTextFile("./shaders/grass.geom.glsl"));
	program->addShader(geomShader.get());

	osg::ref_ptr<osg::Shader> vertexShader = new osg::Shader(osg::Shader::VERTEX, LoadTextFile("./shaders/grass.vert.glsl"));
	program->addShader(vertexShader.get());

	osg::ref_ptr<osg::Shader> fragmentShader = new osg::Shader(osg::Shader::FRAGMENT, LoadTextFile("./shaders/grass.frag.glsl"));
	program->addShader(fragmentShader.get());

	program->addBindFragDataLocation("position", 0);
	program->addBindFragDataLocation("texCoords", 1);
	grassland->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osg::ref_ptr <osg::BlendFunc> fn = new osg::BlendFunc();
	fn->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::DST_ALPHA);
	//geom->getOrCreateStateSet()->setAttributeAndModes(fn.get(), osg::StateAttribute::ON);
	grassland->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	osg::ref_ptr<osg::AlphaFunc> alphaFunc = new osg::AlphaFunc();
	alphaFunc->setFunction(osg::AlphaFunc::ComparisonFunction::GREATER, 0.1);
	grassland->getOrCreateStateSet()->setAttributeAndModes(alphaFunc.get(), osg::StateAttribute::ON);
	osg::ref_ptr<osg::Texture2D> grassTex = LoadTexture("./data/grassBlade.png");
	osg::ref_ptr<osg::Texture2D> grassAlpha = LoadTexture("./data/grassBladeAlpha2.png");
	grassland->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON);
	grassland->getOrCreateStateSet()->setTextureAttributeAndModes(0, grassTex.get(), osg::StateAttribute::ON);
	grassland->getOrCreateStateSet()->setTextureAttributeAndModes(1, grassAlpha.get(), osg::StateAttribute::ON);
	grassland->getOrCreateStateSet()->addUniform(new osg::Uniform("u_grassTex", 0));
	grassland->getOrCreateStateSet()->addUniform(new osg::Uniform("u_grassAlpha", 1));
	grassland->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	double curY = ymin;
	long row = 1;
	long rows = (long)((ymax - ymin) / resolution);
	while (curY + resolution <= ymax)
	{
		double curX = xmin;
		while (curX + resolution <= xmax)
		{
			osg::MatrixTransform* transform = new osg::MatrixTransform;
			for (long lod = 1; lod <= 2; lod++)
			{
				if (row < rows / 2)
				{
					if (lod == 2)
						continue;
				}
				else
				{
					if (lod == 1)
						continue;
				}					
				osg::ref_ptr<osg::Geode> geode = new osg::Geode;
				geode->addDrawable(lods[lod - 1]);
				transform->addChild(geode);
			}
			transform->setMatrix(osg::Matrix::translate(curX + resolution * 0.5, curY + resolution * 0.5, 0));
			transform->getOrCreateStateSet()->getOrCreateUniform("translate", osg::Uniform::FLOAT_VEC3)->set(osg::Vec3(curX + resolution * 0.5, curY + resolution * 0.5, 0));
			grassland->addChild(transform);
			curX += resolution;
		}
		row++;
		curY += resolution;
	}
	return grassland;
}

osg::Geometry* Trace(double& gridSize)
{
	double minBladeWidth = 0.005;
	double maxBladeWidth = 0.0075;
	double cellSize = maxBladeWidth;
	gridSize = 2;
	double minBladeHeight = 0.05;
	double maxBladeHeight = 0.10;

	osg::ref_ptr<osg::Image> grassTex = osgDB::readImageFile("./data/grassBlade.png");
	osg::ref_ptr<osg::Image> grassAlpha = osgDB::readImageFile("./data/grassBladeAlpha2.png");
	osg::ref_ptr<osg::Image> output = new osg::Image;
	int imgW = 512;
	int imgH = 512;
	output->allocateImage(imgW, imgH, 1, GL_RGBA, GL_BYTE);
	RGBPixel* pData = (RGBPixel*)output->data();
	long bladesPerSqrtM = 20000;
	double gridArea = gridSize * gridSize;
	long numBlades = (long)(bladesPerSqrtM / (1.0 / gridArea));

	double xmin = -gridSize * 0.5;
	double xmax = gridSize * 0.5;
	double ymin = -gridSize * 0.5;
	double ymax = gridSize * 0.5;
	std::vector<GrassBlade> blades;
	blades.reserve(numBlades);
	while (blades.size() < numBlades)
	{
		double x = xmin + GetRandomPosition() * gridSize;
		double y = ymin + GetRandomPosition() * gridSize;
		double roll = GetRandomPosition() * 2 * osg::PI;
		double rotate = GetRandomPosition() * 2 * osg::PI;
		double tilt = (25 + GetRandomPosition() * 65) * 0.0174533;
		double h = minBladeHeight + (maxBladeHeight - minBladeHeight) * GetRandomPosition();
		double w = minBladeWidth + (maxBladeWidth - minBladeWidth) * GetRandomPosition();
		GrassBlade blade;
		blade.Create(x, y, w, h, roll, rotate, tilt);
		if (xmin - blade.m_bb.xMin() > gridSize * 0.5
			|| ymin - blade.m_bb.yMin() > gridSize * 0.5
			|| blade.m_bb.xMax() - xmax > gridSize * 0.5
			|| blade.m_bb.yMax() - ymax > gridSize * 0.5)
			continue;

		blades.push_back(blade);
	}
	gridSize = 1;
	//double padding = gridSize * 0.5;
 // xmin = -gridSize * 0.5 - padding;
 // xmax = gridSize * 0.5 + padding;
	//ymin = -gridSize * 0.5 - padding;
	//ymax = gridSize * 0.5 + padding;
	//double pixelW = (xmax - xmin) / imgW;
	//double pixelH = (ymax - ymin) / imgH;
	//for (int py = imgH - 1; py >= 0; py--)
	//{
	//	double cy = ymax - pixelH * py - pixelW * 0.5;
	//	for (int px = 0; px < imgW; px++)
	//	{
	//		double cx = xmin + pixelW * px + pixelW * 0.5;
	//		osg::Vec3 rayOrigin(cx, cy, maxBladeWidth * 100);
	//		RGBPixel rgb;
	//		//if (py < imgH / 2)
	//		//	rgb.Set(255, 0, 0);
	//		double intDist = -1;
	//		for (long n = 0; n < numBlades; n++)
	//		{
	//			GrassBlade& blade = blades[n];
	//			//if (!blade.IsPointInRect(osg::Vec3(cx, cy, 0)))
	//			//	continue;

	//			osg::Vec3 intersect = quadIntersect(rayOrigin, osg::Vec3(0, 0, -1), blade.m_v0, blade.m_v1, blade.m_v2, blade.m_v3);
	//			if (intersect.x() < 0)
	//				continue;

	//			if (intDist > 0)
	//			{
	//				if (intersect.x() - intDist > 0.0000001)
	//					continue;
	//			}
	//			osg::Vec2 uv(intersect.y(), 1 - intersect.z());
	//			float alpha = grassAlpha->getColor(uv).x();
	//			if (alpha < 0.1)
	//				continue;

	//			intDist = intersect.x();
	//			osg::Vec4 color = grassTex->getColor(uv);
	//			rgb.R = (unsigned char)(color.r() * 0.15 * 255.0);
	//			rgb.G = (unsigned char)(color.g() * 1.00 * 255.0);
	//			rgb.B = (unsigned char)(color.b() * 0.15 * 255.0);
	//			rgb.A = (unsigned char)(alpha * 255.0);
	//			//output->setColor(color, osg::Vec2(px / (imgW - 1), py / (imgH - 1)));
	//		}
	//		*pData++ = rgb;
	//	}
	//}
	//osgDB::writeImageFile(*output, "output.png");

	osg::BoundingBox bb(-gridSize * 2, -gridSize * 2, -gridSize * 2, gridSize * 2, gridSize * 2, gridSize * 2);
	GrassObject* geom = new GrassObject(bb);
	osg::Vec3Array* vertices = new osg::Vec3Array();
	osg::Vec4Array* uvs = new osg::Vec4Array();
	for (long n = 0; n < numBlades; n++)
	{
		GrassBlade& blade = blades[n];
		vertices->push_back(osg::Vec3(blade.m_x, blade.m_y, 0));
		uvs->push_back(osg::Vec4(blade.m_rotate, blade.m_tilt, blade.m_w, blade.m_h));
	}

	geom->setVertexAttribArray(0, vertices);
	geom->setVertexAttribBinding(0, osg::Geometry::BIND_PER_VERTEX);

	geom->setVertexAttribArray(1, uvs);
	geom->setVertexAttribBinding(1, osg::Geometry::BIND_PER_VERTEX);

	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->getNumElements()));
	geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osg::ref_ptr <osg::BlendFunc> fn = new osg::BlendFunc();
	fn->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::DST_ALPHA);
	//geom->getOrCreateStateSet()->setAttributeAndModes(fn.get(), osg::StateAttribute::ON);
	geom->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	osg::ref_ptr<osg::AlphaFunc> alphaFunc = new osg::AlphaFunc();
	alphaFunc->setFunction(osg::AlphaFunc::ComparisonFunction::GREATER, 0.1);
	geom->getOrCreateStateSet()->setAttributeAndModes(alphaFunc.get(), osg::StateAttribute::ON);
	return geom;


	//osg::Vec4 color = grassAlpha->getColor(osg::Vec2(0.1f, 0.9f));
	//color = grassAlpha->getColor(osg::Vec2(0.1f, 0.1f));
	//float halfLen = 50;
	//osg::Vec3 v0, v1, v2, v3;
	//v0 = osg::Vec3(-halfLen, 0, halfLen);
	//v1 = osg::Vec3(-halfLen, 0, -halfLen);
	//v2 = osg::Vec3(halfLen, 0, -halfLen);
	//v3 = osg::Vec3(halfLen, 0, halfLen);
	//osg::Vec3 origin(0, 100, 0);
	//osg::Vec3 intersect = quadIntersect(origin, osg::Vec3(0, -1, -0.3), v0, v1, v2, v3);
	//osg::Vec3 target = osg::Vec3(20, 0, 0);
	//intersect = quadIntersect(origin, target - origin, v0, v1, v2, v3);
	//target = osg::Vec3(-20, 0, 0);
	//intersect = quadIntersect(origin, target - origin, v0, v1, v2, v3);
	//target = osg::Vec3(0, 45, 0);
	//intersect = quadIntersect(origin, target - origin, v0, v1, v2, v3);
}

osg::Node* createGrassInstance(float xmin, float ymin, float xmax, float ymax, long bladesPerSqrtM)
{
	float minBladeWidth = 0.002 * 3;
	float maxBladeWidth = 0.006 * 3;
	float cellSize = maxBladeWidth;
	float gridSize = 1;
	float minBladeHeight = 0.02 * 10;
	float maxBladeHeight = 0.05 * 10;
	long numBlades = (long)((xmax - xmin) * (ymax - ymin) * bladesPerSqrtM);
	GrassObject* geom = new GrassObject(osg::BoundingBox(xmin, ymin, xmin, xmax, ymax, xmax));
	geom->setCullingActive(false);
	osg::Vec3Array* vertices = new osg::Vec3Array();
	for (long n = 0; n < numBlades; n++)
	{
		vertices->push_back(osg::Vec3(xmin + (xmax - xmin) * GetRandomPosition(), ymin + (ymax - ymin) * GetRandomPosition(), GetRandomPosition()));
	}
	geom->setVertexAttribArray(0, vertices);
	geom->setVertexAttribBinding(0, osg::Geometry::BIND_PER_VERTEX);

	//geom->setVertexAttribArray(1, uvs);
	//geom->setVertexAttribBinding(1, osg::Geometry::BIND_PER_VERTEX);

	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->getNumElements()));

	osg::ref_ptr<osg::Program> program = new osg::Program;
	osg::ref_ptr<osg::Shader> geomShader = new osg::Shader(osg::Shader::GEOMETRY, LoadTextFile("./shaders/grass.geom.glsl"));
	program->addShader(geomShader.get());

	osg::ref_ptr<osg::Shader> vertexShader = new osg::Shader(osg::Shader::VERTEX, LoadTextFile("./shaders/grass.vert.glsl"));
	program->addShader(vertexShader.get());

	osg::ref_ptr<osg::Shader> fragmentShader = new osg::Shader(osg::Shader::FRAGMENT, LoadTextFile("./shaders/grass.frag.glsl"));
	program->addShader(fragmentShader.get());

	program->addBindFragDataLocation("position", 0);
	program->addBindFragDataLocation("texCoords", 1);
	osg::Geode* geode = new osg::Geode;
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osg::ref_ptr <osg::BlendFunc> fn = new osg::BlendFunc();
	fn->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::DST_ALPHA);
	//geom->getOrCreateStateSet()->setAttributeAndModes(fn.get(), osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	osg::ref_ptr<osg::AlphaFunc> alphaFunc = new osg::AlphaFunc();
	alphaFunc->setFunction(osg::AlphaFunc::ComparisonFunction::GREATER, 0.1);
	geode->getOrCreateStateSet()->setAttributeAndModes(alphaFunc.get(), osg::StateAttribute::ON);
	osg::ref_ptr<osg::Texture2D> grassTex = LoadTexture("./data/grassBlade.png");
	osg::ref_ptr<osg::Texture2D> grassAlpha = LoadTexture("./data/grassBladeAlpha2.png");
	geode->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, grassTex.get(), osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setTextureAttributeAndModes(1, grassAlpha.get(), osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->addUniform(new osg::Uniform("u_grassTex", 0));
	geode->getOrCreateStateSet()->addUniform(new osg::Uniform("u_grassAlpha", 1));
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	geode->getOrCreateStateSet()->getOrCreateUniform("minBladeWidth", osg::Uniform::FLOAT)->set(minBladeWidth);
	geode->getOrCreateStateSet()->getOrCreateUniform("maxBladeWidth", osg::Uniform::FLOAT)->set(maxBladeWidth);
	geode->getOrCreateStateSet()->getOrCreateUniform("minBladeHeight", osg::Uniform::FLOAT)->set(minBladeHeight);
	geode->getOrCreateStateSet()->getOrCreateUniform("maxBladeHeight", osg::Uniform::FLOAT)->set(maxBladeHeight);

	geode->getOrCreateStateSet()->getOrCreateUniform("top", osg::Uniform::FLOAT)->set(ymax);
	geode->getOrCreateStateSet()->getOrCreateUniform("left", osg::Uniform::FLOAT)->set(xmin);
	geode->getOrCreateStateSet()->getOrCreateUniform("right", osg::Uniform::FLOAT)->set(xmax);
	geode->getOrCreateStateSet()->getOrCreateUniform("bottom", osg::Uniform::FLOAT)->set(ymin);

	//geode->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("model", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("cameraPosition", osg::Uniform::FLOAT_VEC3);

	geode->addChild(geom);
	return geode;
}

osg::Node* createGrassCircleInstance(double radius, long bladesPerSqrtM)
{
	float minBladeWidth = 0.002 * 3;
	float maxBladeWidth = 0.006 * 3;
	float cellSize = maxBladeWidth;
	float gridSize = 1;
	float minBladeHeight = 0.02 * 10;
	float maxBladeHeight = 0.05 * 10;
	double xmin = -radius;
	double xmax = radius;
	double ymin = -radius;
	double ymax = radius;
	long numBlades = (long)((osg::PI * radius * radius) * bladesPerSqrtM);
	GrassObject* geom = new GrassObject(osg::BoundingBox(xmin, ymin, xmin, xmax, ymax, xmax));
	geom->setCullingActive(false);
	osg::Vec3Array* vertices = new osg::Vec3Array();
	long n = 0;
	while (n < numBlades)
	{
		double x = xmin + (xmax - xmin) * GetRandomPosition();
		double y = ymin + (ymax - ymin) * GetRandomPosition();
		double dist = sqrt(x * x + y * y);
		if (dist > radius)
			continue;
		vertices->push_back(osg::Vec3(x, y, GetRandomPosition()));
		n++;
	}
	geom->setVertexAttribArray(0, vertices);
	geom->setVertexAttribBinding(0, osg::Geometry::BIND_PER_VERTEX);

	//geom->setVertexAttribArray(1, uvs);
	//geom->setVertexAttribBinding(1, osg::Geometry::BIND_PER_VERTEX);

	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->getNumElements()));

	osg::ref_ptr<osg::Program> program = new osg::Program;
	osg::ref_ptr<osg::Shader> geomShader = new osg::Shader(osg::Shader::GEOMETRY, LoadTextFile("./shaders/grass.geom.glsl"));
	program->addShader(geomShader.get());

	osg::ref_ptr<osg::Shader> vertexShader = new osg::Shader(osg::Shader::VERTEX, LoadTextFile("./shaders/grass.vert.glsl"));
	program->addShader(vertexShader.get());

	osg::ref_ptr<osg::Shader> fragmentShader = new osg::Shader(osg::Shader::FRAGMENT, LoadTextFile("./shaders/grass.frag.glsl"));
	program->addShader(fragmentShader.get());

	program->addBindFragDataLocation("position", 0);
	program->addBindFragDataLocation("texCoords", 1);
	osg::Geode* geode = new osg::Geode;
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osg::ref_ptr <osg::BlendFunc> fn = new osg::BlendFunc();
	fn->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::DST_ALPHA);
	//geom->getOrCreateStateSet()->setAttributeAndModes(fn.get(), osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	osg::ref_ptr<osg::AlphaFunc> alphaFunc = new osg::AlphaFunc();
	alphaFunc->setFunction(osg::AlphaFunc::ComparisonFunction::GREATER, 0.1);
	geode->getOrCreateStateSet()->setAttributeAndModes(alphaFunc.get(), osg::StateAttribute::ON);
	osg::ref_ptr<osg::Texture2D> grassTex = LoadTexture("./data/grassBlade.png");
	osg::ref_ptr<osg::Texture2D> grassAlpha = LoadTexture("./data/grassBladeAlpha2.png");
	geode->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, grassTex.get(), osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setTextureAttributeAndModes(1, grassAlpha.get(), osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->addUniform(new osg::Uniform("u_grassTex", 0));
	geode->getOrCreateStateSet()->addUniform(new osg::Uniform("u_grassAlpha", 1));
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	geode->getOrCreateStateSet()->getOrCreateUniform("minBladeWidth", osg::Uniform::FLOAT)->set(minBladeWidth);
	geode->getOrCreateStateSet()->getOrCreateUniform("maxBladeWidth", osg::Uniform::FLOAT)->set(maxBladeWidth);
	geode->getOrCreateStateSet()->getOrCreateUniform("minBladeHeight", osg::Uniform::FLOAT)->set(minBladeHeight);
	geode->getOrCreateStateSet()->getOrCreateUniform("maxBladeHeight", osg::Uniform::FLOAT)->set(maxBladeHeight);

	geode->getOrCreateStateSet()->getOrCreateUniform("top", osg::Uniform::FLOAT)->set(ymax);
	geode->getOrCreateStateSet()->getOrCreateUniform("left", osg::Uniform::FLOAT)->set(xmin);
	geode->getOrCreateStateSet()->getOrCreateUniform("right", osg::Uniform::FLOAT)->set(xmax);
	geode->getOrCreateStateSet()->getOrCreateUniform("bottom", osg::Uniform::FLOAT)->set(ymin);

	//geode->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("model", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("cameraPosition", osg::Uniform::FLOAT_VEC3);

	geode->addChild(geom);
	return geode;
}

int main(int argc, char** argv)
{
	//DrawEllipse();
	//TestRayPlaneIntersection();
	//osg::Vec3 sceneSize(1, 1, 1);
	//osg::BoundingBox sceneBB(-sceneSize.x() * 0.5, -sceneSize.y() * 0.5, -sceneSize.z() * 0.5, sceneSize.x() * 0.5, sceneSize.y() * 0.5, sceneSize.z() * 0.5);
	////osg::ref_ptr<osg::ShapeDrawable> boundingboxHull = new osg::ShapeDrawable;
	////boundingboxHull->setShape(new osg::Box(osg::Vec3(0.0f, 0.0f, 0.0f), sceneSize.x(), sceneSize.y(), sceneSize.z()));
	////osg::PolygonMode* polygonMode = new osg::PolygonMode(osg::PolygonMode::Face::FRONT_AND_BACK, osg::PolygonMode::Mode::LINE);
	////boundingboxHull->getOrCreateStateSet()->setAttributeAndModes(polygonMode, osg::StateAttribute::ON);
	//osg::Vec3 terrainSize(sceneSize.x(), sceneSize.x(), 2);
	//osg::BoundingBox terrainBB(-100 * 0.5, -100 * 0.5, -100 * 0.5, 100 * 0.5, 100 * 0.5, 100 * 0.5);
	////osg::ref_ptr<osg::Geometry> boundingboxHull = CreateWiredBox(sceneBB);
	//osg::ref_ptr<osg::Geometry> boundingboxHull = CreateSolidBox(terrainBB);
	//osg::ref_ptr<osg::MatrixTransform> terrain = new osg::MatrixTransform;
	//terrain->addChild(boundingboxHull.get());
	//terrain->setMatrix(osg::Matrix::translate(osg::Vec3(0, 0, -1)));

	//osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	//osg::ref_ptr<osg::Texture2D> grassTex = LoadTexture("./data/grassBlade.png");
	//osg::ref_ptr<osg::Texture2D> grassAlpha = LoadTexture("./data/grassBladeAlpha2.png");
	//osg::ref_ptr< osg::Program> program = new osg::Program;
	//osg::ref_ptr<osg::Shader> geomShader = new osg::Shader(osg::Shader::GEOMETRY, LoadTextFile("./shaders/grass.geom.glsl"));
	//program->addShader(geomShader.get());

	//osg::ref_ptr<osg::Shader> vertexShader = new osg::Shader(osg::Shader::VERTEX, LoadTextFile("./shaders/grass.vert.glsl"));
	//program->addShader(vertexShader.get());

	//osg::ref_ptr<osg::Shader> fragmentShader = new osg::Shader(osg::Shader::FRAGMENT, LoadTextFile("./shaders/grass.frag.glsl"));
	//program->addShader(fragmentShader.get());

	//program->addBindFragDataLocation("position", 0);
	//program->addBindFragDataLocation("texCoords", 1);
	//geode->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON);
	//geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, grassTex.get(), osg::StateAttribute::ON);
	//geode->getOrCreateStateSet()->setTextureAttributeAndModes(1, grassAlpha.get(), osg::StateAttribute::ON);
	//geode->getOrCreateStateSet()->addUniform(new osg::Uniform("u_grassTex", 0));
	//geode->getOrCreateStateSet()->addUniform(new osg::Uniform("u_grassAlpha", 1));
	//geode->getOrCreateStateSet()->getOrCreateUniform("osg_FrameTime", osg::Uniform::FLOAT);
	//geode->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4);
	//geode->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("model", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("cameraPosition", osg::Uniform::FLOAT_VEC3);
	//geode->getOrCreateStateSet()->getOrCreateUniform("translate", osg::Uniform::FLOAT_VEC3)->set(osg::Vec3(0, 0, 0));
	//osg::ref_ptr <osg::BlendFunc> fn = new osg::BlendFunc();
	//fn->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::DST_ALPHA);
	////geom->getOrCreateStateSet()->setAttributeAndModes(fn.get(), osg::StateAttribute::ON);
	//geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	////root->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	//osg::ref_ptr<osg::AlphaFunc> alphaFunc = new osg::AlphaFunc();
	//alphaFunc->setFunction(osg::AlphaFunc::ComparisonFunction::GREATER, 0.1);
	//geode->getOrCreateStateSet()->setAttributeAndModes(alphaFunc.get(), osg::StateAttribute::ON);

	////osg::ref_ptr<osg::Texture2D> grassTex = LoadTexture("C:/Code/GrassOSG/bin/shaders/grass_grid.png");
	////osg::ref_ptr<osg::Texture2D> grassAlpha = LoadTexture("C:/Code/GrassOSG/bin/shaders/heightmap.png");
	//srand(time(0));
	////osg::Geometry* outerRing = createGrassPatch(1000000, 1000, 1);
	////osg::Geometry* middleRing = createGrassPatch(1000000, 100, 0.8);
	//double gridSize;
	//osg::Geometry* grassPatch = Trace(gridSize);// createGrassPatch(100000, 0.5, 0.0);
	////geode->addDrawable(outerRing);
	////geode->addDrawable(middleRing);
	//geode->addDrawable(grassPatch);

	osg::ref_ptr<osg::Group> root = new osg::Group;

	//root->addChild(renderCamera);
	//root->addChild(geode.get());

	//m_grassland = CreateRect(-4, -4, 4, 4, 0);
	//m_grassland->getOrCreateStateSet()->setTextureAttributeAndModes(2, renderCamera->Texture().get(), osg::StateAttribute::ON);
  //m_grassland->getOrCreateStateSet()->addUniform(new osg::Uniform("u_grassSample", 2));
	//osg::ref_ptr<osg::Group> grassInstances = createGrassGrid(-4, -4, 4, 4, 1);
	//root->addChild(grassInstances.get());
	//root->addChild(m_grassland);
	//root->addChild(osgDB::readNodeFile("./data/CAD.osgb"));
	//root->addChild(osgDB::readNodeFile("./data/OAP3D/Tile_-002_-016/Tile_-002_-016.osgb"));
	

	//osg::ComputeBoundsVisitor vis;
	//boston->accept(vis);
 // auto bb = vis.getBoundingBox();
	//osg::MatrixTransform* mat = new osg::MatrixTransform;
	//mat->addChild(boston);
	//mat->setMatrix(osg::Matrix::translate(-bb.center().x(), -bb.center().y(), 0));
	//osgDB::writeNodeFile(*mat, "./data/city.ive");
	//osg::Group* boston = (osg::Group*)osgDB::readNodeFile("./data/boston.ive");
	//osg::Group* newGroup = new osg::Group;
	//osg::Vec3 center;
	//double maxH = -1;
	//int buildings = boston->getNumChildren();
	//for (int i = 0; i < buildings; i++)
	//{
	//	osg::Node* building = boston->getChild(i);
	//	osg::ComputeBoundsVisitor vis;
	//	building->accept(vis);
	//	double bHeight = vis.getBoundingBox().zMax();
	//	if (bHeight > maxH)
	//	{
	//		bHeight = maxH;
	//		center = vis.getBoundingBox().center();
	//	}
	//}
	//osg::BoundingBox blockBB(center.x() - 500, center.y() - 500, -500, center.x() + 500, center.y() + 500, 500);
	//for (int i = 0; i < buildings; i++)
	//{
	//	osg::Node* building = boston->getChild(i);
	//	osg::ComputeBoundsVisitor vis;
	//	building->accept(vis);
	//	auto bb = vis.getBoundingBox();
	//	if (!blockBB.contains(bb.center()))
	//		continue;
	//	newGroup->addChild(building);
	//}
	//osgDB::writeNodeFile(*newGroup, "./data/city.ive");
	osg::Group* boston = (osg::Group*)osgDB::readNodeFile("./data/boston.ive");
	root->addChild(boston);
	//osg::Node* grassInstance = createGrassInstance(-0.5, -0.5, 0.5, 0.5, 10000);


	m_grassland = CreateRect(-1000, -1000, 1000, 1000, 0);
	root->addChild(m_grassland);
	RenderSurface* renderCamera = new RenderSurface(1024, 1024, GL_RGBA, GL_RGBA, GL_BYTE, false);
	float resolution = 1.0;
	float sampleResol = resolution * 1.5;
	float circleRadius = 10;
	renderCamera->getOrCreateStateSet()->getOrCreateUniform("resolution", osg::Uniform::FLOAT)->set(resolution);
	renderCamera->getOrCreateStateSet()->getOrCreateUniform("translate", osg::Uniform::FLOAT_VEC3)->set(osg::Vec3(0, 0, 0));
	osg::Node* sampleGrass = createGrassInstance(-0.5 * sampleResol, -0.5 * sampleResol, 0.5 * sampleResol, 0.5 * sampleResol, 2500);
	sampleGrass->getOrCreateStateSet()->getOrCreateUniform("minTilt", osg::Uniform::FLOAT)->set(45.0f);
	sampleGrass->getOrCreateStateSet()->getOrCreateUniform("maxTilt", osg::Uniform::FLOAT)->set(90.0f);
	sampleGrass->getOrCreateStateSet()->getOrCreateUniform("minRotate", osg::Uniform::FLOAT)->set(0.0f);
	sampleGrass->getOrCreateStateSet()->getOrCreateUniform("maxRotate", osg::Uniform::FLOAT)->set(360.0f);
	sampleGrass->getOrCreateStateSet()->getOrCreateUniform("isSample", osg::Uniform::BOOL)->set(true);
	sampleGrass->getOrCreateStateSet()->getOrCreateUniform("circleRadius", osg::Uniform::FLOAT)->set(circleRadius);
	renderCamera->addChild(sampleGrass);
	
	m_renderCircleCamera = new RenderSurface(1024, 768, GL_RGBA, GL_RGBA, GL_BYTE, false);
	m_renderCircleCamera->getOrCreateStateSet()->getOrCreateUniform("resolution", osg::Uniform::FLOAT)->set(resolution);
	m_renderCircleCamera->getOrCreateStateSet()->getOrCreateUniform("translate", osg::Uniform::FLOAT_VEC3)->set(osg::Vec3(0, 0, 0));
	osg::Node* circleGrass = createGrassCircleInstance(circleRadius, 1500);
	circleGrass->getOrCreateStateSet()->getOrCreateUniform("minTilt", osg::Uniform::FLOAT)->set(25.0f);
	circleGrass->getOrCreateStateSet()->getOrCreateUniform("maxTilt", osg::Uniform::FLOAT)->set(80.0f);
	circleGrass->getOrCreateStateSet()->getOrCreateUniform("minRotate", osg::Uniform::FLOAT)->set(0.0f);
	circleGrass->getOrCreateStateSet()->getOrCreateUniform("maxRotate", osg::Uniform::FLOAT)->set(360.0f);
	circleGrass->getOrCreateStateSet()->getOrCreateUniform("isSample", osg::Uniform::BOOL)->set(false);
	m_renderCircleCamera->addChild(circleGrass);

	root->addChild(renderCamera);
	root->addChild(m_renderCircleCamera);
	m_grassland->getOrCreateStateSet()->setTextureAttributeAndModes(1, renderCamera->Texture().get(), osg::StateAttribute::ON);
  m_grassland->getOrCreateStateSet()->addUniform(new osg::Uniform("u_grassSample", 1));
	m_grassland->getOrCreateStateSet()->setTextureAttributeAndModes(2, m_renderCircleCamera->Texture().get(), osg::StateAttribute::ON);
	m_grassland->getOrCreateStateSet()->addUniform(new osg::Uniform("u_grassCircle", 2));
	m_grassland->getOrCreateStateSet()->getOrCreateUniform("circleCenter", osg::Uniform::FLOAT_VEC3)->set(osg::Vec3(0, 0, 0));
	m_grassland->getOrCreateStateSet()->getOrCreateUniform("circleRadius", osg::Uniform::FLOAT)->set(circleRadius);
	m_grassland->getOrCreateStateSet()->getOrCreateUniform("resolution", osg::Uniform::FLOAT)->set(resolution);
	//root->addChild(geode);
	//root->addChild(terrain.get());
	//root->addChild(CreateRect(sceneBB.xMin(), sceneBB.yMin(), sceneBB.xMax(), sceneBB.yMax(), 0));
	

	
	//root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	//root->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	
	osgViewer::Viewer viewer;
	//viewer.realize();
	//viewer.getCamera()->getGraphicsContext()->getState()->setUseVertexAttributeAliasing(true);
	//viewer.setUpViewOnSingleScreen(1);
	viewer.setUpViewInWindow(50, 50, 1024, 768);
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
	viewer.init();
	viewer.realize();
	osg::Image* colorBuffer = new osg::Image;
	//colorBuffer->allocateImage(512, 512, 1, GL_RGB, GL_BYTE);
	//viewer.getCamera()->attach(osg::Camera::COLOR_BUFFER0, colorBuffer);
	//viewer.getCamera()->setClearColor(osg::Vec4(0, 0, 0, 0));
	long frameCount = 0;
	osg::FloatArray* projectArr = new osg::FloatArray(16);
	osg::FloatArray* viewArr = new osg::FloatArray(16);
	osg::FloatArray* modelArr = new osg::FloatArray(16);

	osg::FloatArray* renderProjArr = new osg::FloatArray(16);
	osg::FloatArray* renderViewArr = new osg::FloatArray(16);
	while (!viewer.done())
	{
		frameCount = frameCount + 1;
		//if (frameCount == 100)
		//	osgDB::writeImageFile(*colorBuffer, "img.png");
		//geode->getOrCreateStateSet()->getOrCreateUniform("osg_FrameTime", osg::Uniform::FLOAT)->set((float)frameCount);
		//
		////viewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3(0, 0, 100), osg::Vec3(0, 0, 0), osg::Vec3(0, 1, 0));
		////viewer.getCamera()->setProjectionMatrixAsOrtho(-gridSize*0.5, gridSize * 0.5, -gridSize * 0.5, gridSize * 0.5, 0.01, 100);
		//
		osg::Matrix project = viewer.getCamera()->getProjectionMatrix();
		osg::Vec3 eye, center, up;
		viewer.getCamera()->getViewMatrixAsLookAt(eye, center, up);

		//osg::Vec3 sceneCenter = osg::Vec3()
		osg::Vec3 v0 = osg::Vec3(-100000, 100000, 0);
		osg::Vec3 v1 = osg::Vec3(-100000, -100000, 0);
		osg::Vec3 v2 = osg::Vec3(100000, -100000, 0);
		osg::Vec3 v3 = osg::Vec3(100000, 100000, 0);
		osg::Vec3 look = center - eye;
		look.normalize();
		osg::Vec3 intersect = quadIntersect(eye, look, v0, v1, v2, v3);
		float cameraTilt = 90;
		if (intersect.x() > 0.0)
		{
			osg::Vec3 p = eye + look * intersect.x();
			p.z() = 0;
			m_renderCircleCamera->getOrCreateStateSet()->getOrCreateUniform("translate", osg::Uniform::FLOAT_VEC3)->set(p);
			m_grassland->getOrCreateStateSet()->getOrCreateUniform("circleCenter", osg::Uniform::FLOAT_VEC3)->set(p);
			sampleGrass->getOrCreateStateSet()->getOrCreateUniform("circleCenter", osg::Uniform::FLOAT_VEC3)->set(p);
			cameraTilt = asin(eye.z() / (eye - p).length()) * 57.2958;
			if (cameraTilt < 45)
				cameraTilt = 45;
		}
		else
		{
			m_renderCircleCamera->getOrCreateStateSet()->getOrCreateUniform("translate", osg::Uniform::FLOAT_VEC3)->set(osg::Vec3(0, 0, 0));
			m_grassland->getOrCreateStateSet()->getOrCreateUniform("circleCenter", osg::Uniform::FLOAT_VEC3)->set(osg::Vec3(0, 0, 0));
			sampleGrass->getOrCreateStateSet()->getOrCreateUniform("circleCenter", osg::Uniform::FLOAT_VEC3)->set(osg::Vec3(0, 0, 0));
		}


		//osg::Vec3 forward = (eye - center);
		//eye = osg::Vec3(0, 0, 0) + forward * 100;
		//forward.normalize();
		osg::Vec3 oriVec(0, -1, 0);
		oriVec = oriVec * osg::Matrix::rotate(osg::DegreesToRadians(cameraTilt), osg::Vec3(1, 0, 0));
		oriVec.normalize();
		osg::Vec3 newEye = oriVec * 100;
		osg::Matrix renderView = osg::Matrix::lookAt(osg::Vec3(0, 0, 100), osg::Vec3(0, 0, 0), osg::Vec3(0, 1, 0));
		osg::BoundingBox grassBB(-0.5 * resolution, -0.5 * resolution, 0, 0.5 * resolution, 0.5 * resolution, 0.5 * resolution);
		osg::BoundingBox transformedBB = grassBB;
		//transformedBB.init();
		//for (long i = 0; i < 8; i++)
		//{
		//	transformedBB.expandBy(grassBB.corner(i) * renderView);
		//}
		double transformedWidth = (transformedBB.yMax() - transformedBB.yMin()) * 0.5;
		double transformedHeight = (transformedBB.xMax() - transformedBB.xMin()) * 0.5;
		osg::Matrix renderProj = osg::Matrix::ortho(-transformedWidth, transformedWidth, -transformedHeight, transformedHeight, 1, 1000);
		double* pRenderProj = (double*)renderProj.ptr();
		for (int n = 0; n < 16; n++)
		{
			(*renderProjArr)[n] = (float)pRenderProj[n];
		}
		double* pViewProj = (double*)renderView.ptr();
		for (int n = 0; n < 16; n++)
		{
			(*renderViewArr)[n] = (float)pViewProj[n];
		}
		
		m_grassland->getOrCreateStateSet()->getOrCreateUniform("texProjection", osg::Uniform::FLOAT_MAT4)->setArray(renderProjArr);
		m_grassland->getOrCreateStateSet()->getOrCreateUniform("texView", osg::Uniform::FLOAT_MAT4)->setArray(renderViewArr);
		
		renderCamera->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4)->setArray(renderProjArr);
		renderCamera->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->setArray(renderViewArr);
		////geom->getOrCreateStateSet()->getOrCreateUniform("model", osg::Uniform::FLOAT_MAT4)->setArray(viewArr);
		//m_grassland->getOrCreateStateSet()->getOrCreateUniform("cameraPosition", osg::Uniform::FLOAT_VEC3)->set(eye);

		double* pProj = (double*)project.ptr();
		for (int n = 0; n < 16; n++)
		{
			(*projectArr)[n] = (float)pProj[n];
		}

		osg::Matrix view = viewer.getCamera()->getViewMatrix();
	
		////view = osg::Matrix::lookAt(osg::Vec3(0, 0, 10), osg::Vec3(0, 0, 0), osg::Vec3(0, 1, 0));
		//
		////osg::Vec3 ps = osg::Vec3(-0.000793749990, 0.00000000, 0.00000000) * view * project;
		////ps = osg::Vec3(-0.000793749990, 0.01, 0.00000000) * view * project;
		double* pView = (double*)view.ptr();
		for (int n = 0; n < 16; n++)
		{
			(*viewArr)[n] = (float)pView[n];
		}
		////viewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3(0, 0, 100), osg::Vec3(0, 0, 0), osg::Vec3(0, 1, 0));
		////
		viewer.getCamera()->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4)->setArray(projectArr);
		viewer.getCamera()->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->setArray(viewArr);
		////geom->getOrCreateStateSet()->getOrCreateUniform("model", osg::Uniform::FLOAT_MAT4)->setArray(viewArr);
		viewer.getCamera()->getOrCreateStateSet()->getOrCreateUniform("cameraPosition", osg::Uniform::FLOAT_VEC3)->set(eye);

		//m_grassland->getOrCreateStateSet()->getOrCreateUniform("projection2", osg::Uniform::FLOAT_MAT4)->setArray(projectArr);
		//m_grassland->getOrCreateStateSet()->getOrCreateUniform("view2", osg::Uniform::FLOAT_MAT4)->setArray(viewArr);

		m_renderCircleCamera->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4)->setArray(projectArr);
		m_renderCircleCamera->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->setArray(viewArr);
	
		//viewer.getCamera()->getViewMatrixAsLookAt(eye, center, up);
		//for (size_t i = 0; i < grassInstances->getNumChildren(); i++)
		//{
		//	osg::MatrixTransform* grassInstance = (osg::MatrixTransform*)grassInstances->getChild(i);
		//	osg::Vec3 bottomLeft = osg::Vec3(-0.5, -0.5, 0) * grassInstance->getMatrix() * view * project;
		//	osg::Vec3 topRight = osg::Vec3(0.5, 0.5, 0) * grassInstance->getMatrix() * view * project;
		//	double h = viewer.getCamera()->getViewport()->height();
		//	double w = viewer.getCamera()->getViewport()->width();
		//	bottomLeft.x() = bottomLeft.x() * 0.5 + 0.5;
		//	bottomLeft.y() = bottomLeft.y() * 0.5 + 0.5;
		//	topRight.x() = topRight.x() * 0.5 + 0.5;
		//	topRight.y() = topRight.y() * 0.5 + 0.5;
		//	if (bottomLeft.x() < 0)
		//		bottomLeft.x() = 0;
		//	if (bottomLeft.x() > 1)
		//		bottomLeft.x() = 1;
		//	if (bottomLeft.y() < 0)
		//		bottomLeft.y() = 0;
		//	if (bottomLeft.y() > 1)
		//		bottomLeft.y() = 1;
		//	if (topRight.x() < 0)
		//		topRight.x() = 0;
		//	if (topRight.x() > 1)
		//		topRight.x() = 1;
		//	if (topRight.y() < 0)
		//		topRight.y() = 0;
		//	if (topRight.y() > 1)
		//		topRight.y() = 1;

		//	double minScreenX = w;
		//  minScreenX = min(minScreenX, bottomLeft.x());
		//	minScreenX = min(minScreenX, topRight.x());

		//	double maxScreenX = 0;
		//	maxScreenX = max(maxScreenX, bottomLeft.x());
		//	maxScreenX = max(maxScreenX, topRight.x());

		//	double minScreenY = h;
		//	minScreenY = min(minScreenY, bottomLeft.y());
		//	minScreenY = min(minScreenY, topRight.y());

		//	double maxScreenY = 0;
		//	maxScreenY = max(maxScreenY, bottomLeft.y());
		//	maxScreenY = max(maxScreenY, topRight.y());

		//	double pixels = ((maxScreenX - minScreenX) * w) * ((maxScreenY - minScreenY) * h);
		//	double distToCam = (osg::Vec3(0.0, 0.0, 0.0) * grassInstance->getMatrix() - eye).length();
		//	//if (distToCam < 200 && pixels > 0)
		//	//{
		//	//	long lod = (long)(distToCam / 10);
		//	//	if (lod > 10)
		//	//		lod = 10;
		//	//	if (distToCam < 10)
		//	//	{
		//	//		distToCam = 2;
		//	//		if (distToCam < 3)
		//	//			lod = 1;
		//	//	}
		//	//	for (long i = 1; i <= 10; i++)
		//	//	{
		//	//		grassInstance->getChild(i - 1)->setNodeMask(i == lod ? 0xffffffff : 0x0);
		//	//	}
		//	//	grassInstance->setNodeMask(0xffffffff);
		//	//	//if (pixels < 10000 && pixels > 5000)
		//	//	//	scale = 2.0;
		//	//	//else if (pixels < 5000 && pixels > 1000)
		//	//	//	scale = 3.0;
		//	//	//else if (pixels < 1000)
		//	//	//	scale = 5.0;
		//	//	//if (pixels > 10000)
		//	//	//	pixels = 10000;
		//	//	//float scale = sqrt(10000 / pixels);
		//	//	//float scale = 1.0;//
		//	//	//grassInstance->getOrCreateStateSet()->getOrCreateUniform("scale", osg::Uniform::FLOAT)->set(scale);
		//	//}
		//	//else
		//	//	grassInstance->setNodeMask(0x0);
		//}
		//grassInstances->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4)->setArray(projectArr);
		//grassInstances->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->setArray(viewArr);
		////geom->getOrCreateStateSet()->getOrCreateUniform("model", osg::Uniform::FLOAT_MAT4)->setArray(viewArr);
		//grassInstances->getOrCreateStateSet()->getOrCreateUniform("cameraPosition", osg::Uniform::FLOAT_VEC3)->set(eye);
		//renderCamera->setViewMatrixAsLookAt(osg::Vec3(0, 0, 10), osg::Vec3(0, 0, 0), osg::Vec3(0, 1, 0));
		//renderCamera->setProjectionMatrixAsOrtho(-5, 5, -5, 5, 1, 1000);
		/*project = renderCamera->getProjectionMatrix();
		pProj = (double*)project.ptr();
		for (int n = 0; n < 16; n++)
		{
			(*projectArr)[n] = (float)pProj[n];
		}

		view = renderCamera->getViewMatrix();
		pView = (double*)view.ptr();
		for (int n = 0; n < 16; n++)
		{
			(*viewArr)[n] = (float)pView[n];
		}

		osg::Matrix model = osg::Matrix::identity();
		double* pModelArr = (double*)model.ptr();
		for (int n = 0; n < 16; n++)
		{
			(*modelArr)[n] = (float)pModelArr[n];
		}*/

		//renderCamera->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4)->setArray(projectArr);
		//renderCamera->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->setArray(viewArr);
		viewer.frame();
		renderCamera->setNodeMask(0x0);
		if (frameCount % 100 == 0)
		{
			//renderCamera->setNodeMask(0x0);
			//osgDB::writeImageFile(*(m_renderCircleCamera->Image().get()), "grass.png");
		}
	}

	return viewer.run();
}