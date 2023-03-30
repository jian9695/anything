
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
#include <osg/Texture2DArray>
#include <osg/Point>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>
#include <osg/ShapeDrawable>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowTexture>
#include <osgShadow/ShadowMap>
#include <osgShadow/SoftShadowMap>
#include <osgShadow/ParallelSplitShadowMap>
#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <osgShadow/StandardShadowMap>
#include <osgShadow/ViewDependentShadowMap>
#include <osg/BlendFunc>
#include <osg/Stencil>
#include <osg/ColorMask>
#include <osg/Depth>
#include <osgDB/ReadFile>

#include "ScreenOverlay.h"
int Num_Particles = 65536;
const float DEG2RAD = 0.0174533;
const float RAD2DEG = 57.2958;

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

osg::Node* m_grassland = nullptr;
bool m_isMouseDown = false;
RenderSurface* m_renderGrassCamera = nullptr;
osg::Camera* m_viewCamera = nullptr;
osg::Node* m_grassSampleRef = nullptr;
RenderSurface* m_resampleTexCamera = nullptr;
OverlayRenderSurface* m_resampleTex;
osg::Node* m_sampleGrassLowResol;
osg::Node* m_sampleGrassHighResol;
osg::Group* m_sampleGrassQuads;

float m_highResol = 0.25;
float m_lowResol = 1.0f;
osg::BoundingBox m_grassBB;

class ScreenRezieEventHandler : public osgGA::GUIEventHandler
{
public:

	ScreenRezieEventHandler() : osgGA::GUIEventHandler()
	{

	}

	static void toggleState(osg::StateSet* stateSet, std::string stateName)
	{
		bool val = false;
		stateSet->getOrCreateUniform(stateName, osg::Uniform::BOOL)->get(val);
		stateSet->getOrCreateUniform(stateName, osg::Uniform::BOOL)->set(!val);
	}

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
	{
		switch (ea.getEventType())
		{
		case(osgGA::GUIEventAdapter::RESIZE):
		{
			int newWidth = ea.getWindowWidth();
			int newHeight = ea.getWindowHeight();
			if (m_renderGrassCamera)
				m_renderGrassCamera->resize(newWidth, newHeight);
			break;
		}
		case(osgGA::GUIEventAdapter::KEYDOWN):
		{
			auto stateSet = m_viewCamera->getOrCreateStateSet();
			if (ea.getKey() == osgGA::GUIEventAdapter::KEY_G)
				toggleState(stateSet, "u_showGrid");
			else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_A)
				toggleState(stateSet, "u_animate");
			else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_L)
				toggleState(stateSet, "u_light");

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

struct RGBA
{
	unsigned char r;
	unsigned char g;
	unsigned char b;
	unsigned char a;
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

class GrassObject : public osg::Geometry
{
private:
	osg::BoundingBox m_bb;

public:
	GrassObject(osg::BoundingBox bb)
		:m_bb(bb)
	{}

	GrassObject() {}

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

class DummyBoundGroup : public osg::MatrixTransform
{
private:
	osg::BoundingBox m_bb;

public:
	DummyBoundGroup(osg::BoundingBox bb)
		:m_bb(bb)
	{}

	DummyBoundGroup() {}

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

//osg::Geometry* createGrassPatch(size_t num, double radius, float flag)
//{
//	double minBladeWidth = 0.0015875;
//	double maxBladeWidth = 0.002;
//	double cellSize = maxBladeWidth;
//	double gridSize = cellSize * 10;
//	double minBladeHeight = gridSize * 0.5;
//	double maxBladeHeight = gridSize * 2;
//	osg::BoundingBox bb(-radius * 2, -radius * 2, -radius * 2, radius * 2, radius * 2, radius * 2);
//	GrassObject* geom = new GrassObject(bb);
//	osg::Vec3Array* vertices = new osg::Vec3Array();
//	osg::Vec4Array* uvs = new osg::Vec4Array();
//	size_t curNum = 0;
//	double xmin = -radius;
//	double ymin = -radius;
//	while (curNum < num)
//	{
//		double x = xmin + GetRandomPosition() * radius * 2;
//		double y = ymin + GetRandomPosition	() * radius * 2;
//		//if (sqrt(x * x + y * y) > radius)
//		//	continue;
//		double z = GetRandomPosition();
//		vertices->push_back(osg::Vec3(x, y, z));
//		double rotate = GetRandomPosition() * 2 * osg::PI;
//		double tilt = GetRandomPosition() * (osg::PI * 0.5) * 0.5;
//		double h = minBladeHeight + (maxBladeHeight - minBladeHeight) * GetRandomPosition();
//		double w = minBladeWidth + (maxBladeWidth - minBladeWidth) * GetRandomPosition();
//		//uvs->push_back(osg::Vec4(GetRandomPosition(), GetRandomPosition(), GetRandomPosition(), 1));
//		uvs->push_back(osg::Vec4(rotate, tilt, w, h));
//		curNum++;
//	}
//	//for (size_t n = 0; n < num; n++)
//	//{
//	//	double r = GetRandomPosition() * radius;
//	//	double theta = GetRandomPosition() * 3.1415926 * 2;
//	//	double rootX = r * cos(theta);
//	//	double rootY = r * sin(theta);
//	//	vertices->push_back(osg::Vec3(rootX, rootY, GetRandomPosition()));
//	//}
//	geom->setVertexAttribArray(0, vertices);
//	geom->setVertexAttribBinding(0, osg::Geometry::BIND_PER_VERTEX);
//
//	geom->setVertexAttribArray(1, uvs);
//	geom->setVertexAttribBinding(1, osg::Geometry::BIND_PER_VERTEX);
//	
//	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->getNumElements()));
//	geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
//	geom->getOrCreateStateSet()->getOrCreateUniform("ringFlag", osg::Uniform::FLOAT)->set(flag);
//
//	osg::ref_ptr <osg::BlendFunc> fn = new osg::BlendFunc();
//	fn->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::DST_ALPHA);
//	//geom->getOrCreateStateSet()->setAttributeAndModes(fn.get(), osg::StateAttribute::ON);
//	geom->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
//
//	osg::ref_ptr<osg::AlphaFunc> alphaFunc = new osg::AlphaFunc();
//	alphaFunc->setFunction(osg::AlphaFunc::ComparisonFunction::GREATER, 0.1);
//	geom->getOrCreateStateSet()->setAttributeAndModes(alphaFunc.get(), osg::StateAttribute::ON);
//	return geom;
//}

//osg::Node* CreateRect(float xmin, float ymin, float xmax, float ymax, float z)
//{
//	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
//	osg::Geode* geode = new osg::Geode();
//	osg::Vec3Array* vertices = new osg::Vec3Array();
//	osg::Vec3Array* normals = new osg::Vec3Array();
//	osg::Vec2Array* uvs = new osg::Vec2Array();
//	vertices->push_back(osg::Vec3(xmin, ymin, z));
//	uvs->push_back(osg::Vec2(0, 1));
//	vertices->push_back(osg::Vec3(xmin, ymax, z));
//	uvs->push_back(osg::Vec2(0, 0));
//	vertices->push_back(osg::Vec3(xmax, ymin, z));
//	uvs->push_back(osg::Vec2(1, 1));
//	vertices->push_back(osg::Vec3(xmax, ymin, z));
//	uvs->push_back(osg::Vec2(1, 1));
//	vertices->push_back(osg::Vec3(xmin, ymax, z));
//	uvs->push_back(osg::Vec2(0, 0));
//	vertices->push_back(osg::Vec3(xmax, ymax, z));
//	uvs->push_back(osg::Vec2(1, 0));
//	normals->push_back(osg::Vec3(0, 0, 1));
//	geom->setVertexArray(vertices);
//	geom->setNormalArray(normals);
//	geom->setTexCoordArray(0, uvs);
//	geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
//	geom->setCullingActive(false);
//	geode->setCullingActive(false);
//	geode->getOrCreateStateSet()->setMode(GL_CULL_FACE,
//		osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
//	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, 6));
//	geode->addDrawable(geom.get());
//	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
//
//	osg::ref_ptr<osg::Texture2D> terrainTex = LoadTexture("./data/terrain.png");
//	terrainTex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
//	terrainTex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);
//
//	geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, terrainTex.get(), osg::StateAttribute::ON);
//	geom->getOrCreateStateSet()->addUniform(new osg::Uniform("u_terrainTex", 0));
//	
//	geom->getOrCreateStateSet()->getOrCreateUniform("top", osg::Uniform::FLOAT)->set((float)ymax);
//	geom->getOrCreateStateSet()->getOrCreateUniform("left", osg::Uniform::FLOAT)->set((float)xmin);
//	geom->getOrCreateStateSet()->getOrCreateUniform("right", osg::Uniform::FLOAT)->set((float)xmax);
//	geom->getOrCreateStateSet()->getOrCreateUniform("bottom", osg::Uniform::FLOAT)->set((float)ymin);
//	geode->getOrCreateStateSet()->getOrCreateUniform("showGrid", osg::Uniform::BOOL)->set(false);
//
//	//geode->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4);
//	//geode->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
//	//geode->getOrCreateStateSet()->getOrCreateUniform("model", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
//	//geode->getOrCreateStateSet()->getOrCreateUniform("circleProjection", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
//	//geode->getOrCreateStateSet()->getOrCreateUniform("circleView", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
//	//geode->getOrCreateStateSet()->getOrCreateUniform("cameraPosition", osg::Uniform::FLOAT_VEC3);
//
//	osg::ref_ptr< osg::Program> program = new osg::Program;
//	osg::ref_ptr<osg::Shader> vertexShader = new osg::Shader(osg::Shader::VERTEX, LoadTextFile("./shaders/simple.vert.glsl"));
//	program->addShader(vertexShader.get());
//
//	osg::ref_ptr<osg::Shader> fragmentShader = new osg::Shader(osg::Shader::FRAGMENT, LoadTextFile("./shaders/simple.frag.glsl"));
//	program->addShader(fragmentShader.get());
//	geode->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON);
//
//	osg::ref_ptr <osg::BlendFunc> fn = new osg::BlendFunc();
//	fn->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::DST_ALPHA);
//	//geom->getOrCreateStateSet()->setAttributeAndModes(fn.get(), osg::StateAttribute::ON);
//	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
//	//root->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
//	//osg::ref_ptr<osg::AlphaFunc> alphaFunc = new osg::AlphaFunc();
//	//alphaFunc->setFunction(osg::AlphaFunc::ComparisonFunction::GREATER, 0.1);
//	//geode->getOrCreateStateSet()->setAttributeAndModes(alphaFunc.get(), osg::StateAttribute::ON);
//
//
//	return geode;
//}

osg::Node* createGrassland(float xmin, float ymin, float xmax, float ymax, float z)
{
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
	osg::Geode* geode = new osg::Geode();
	osg::Vec3Array* vertices = new osg::Vec3Array();
	//osg::Vec3Array* normals = new osg::Vec3Array();
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

	//normals->push_back(osg::Vec3(0, 0, 1));
	//geom->setVertexArray(vertices);
	geom->setVertexAttribArray(0, vertices);
	geom->setVertexAttribBinding(0, osg::Geometry::BIND_PER_VERTEX);

	geom->setVertexAttribArray(1, uvs);
	geom->setVertexAttribBinding(1, osg::Geometry::BIND_PER_VERTEX);

	//geom->setNormalArray(normals);
	//geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
	geom->setVertexAttribBinding(1, osg::Geometry::BIND_PER_VERTEX);
	geom->setCullingActive(false);
	geode->setCullingActive(false);
	geode->getOrCreateStateSet()->setMode(GL_CULL_FACE,
		osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, 6));
	geode->addDrawable(geom.get());
	//geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	//osg::ref_ptr<osg::Texture2D> terrainTex = LoadTexture("./data/terrain.png");
	//terrainTex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
	//terrainTex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);

	//geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, terrainTex.get(), osg::StateAttribute::ON);
	//geom->getOrCreateStateSet()->addUniform(new osg::Uniform("u_terrainTex", 0));

	geom->getOrCreateStateSet()->getOrCreateUniform("top", osg::Uniform::FLOAT)->set((float)ymax);
	geom->getOrCreateStateSet()->getOrCreateUniform("left", osg::Uniform::FLOAT)->set((float)xmin);
	geom->getOrCreateStateSet()->getOrCreateUniform("right", osg::Uniform::FLOAT)->set((float)xmax);
	geom->getOrCreateStateSet()->getOrCreateUniform("bottom", osg::Uniform::FLOAT)->set((float)ymin);

	osg::ref_ptr< osg::Program> program = new osg::Program;
	osg::ref_ptr<osg::Shader> vertexShader = new osg::Shader(osg::Shader::VERTEX, LoadTextFile("./shaders/grassland.vert.glsl"));
	program->addShader(vertexShader.get());

	osg::ref_ptr<osg::Shader> fragmentShader = new osg::Shader(osg::Shader::FRAGMENT, LoadTextFile("./shaders/grassland.frag.glsl"));
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

osg::Node* CreateGround(float xmin, float ymin, float xmax, float ymax, float z)
{
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
	osg::Geode* geode = new osg::Geode();
	osg::Vec3Array* vertices = new osg::Vec3Array();
	osg::Vec3Array* normals = new osg::Vec3Array();
	osg::Vec2Array* uvs = new osg::Vec2Array();
	vertices->push_back(osg::Vec3(xmin, ymin, z));
	uvs->push_back(osg::Vec2(0, ymax - ymin));
	vertices->push_back(osg::Vec3(xmin, ymax, z));
	uvs->push_back(osg::Vec2(0, 0));
	vertices->push_back(osg::Vec3(xmax, ymin, z));
	uvs->push_back(osg::Vec2(xmax - xmin, ymax - ymin));
	vertices->push_back(osg::Vec3(xmax, ymin, z));
	uvs->push_back(osg::Vec2(xmax - xmin, ymax - ymin));
	vertices->push_back(osg::Vec3(xmin, ymax, z));
	uvs->push_back(osg::Vec2(0, 0));
	vertices->push_back(osg::Vec3(xmax, ymax, z));
	uvs->push_back(osg::Vec2(xmax - xmin, 0));

	normals->push_back(osg::Vec3(0, 0, 1));
	geom->setVertexArray(vertices);
	geom->setTexCoordArray(0, uvs);
	geom->setNormalArray(normals);
	geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, 6));
	geode->addDrawable(geom.get());
	osg::ref_ptr<osg::Texture2D> terrainTex = LoadTexture("./data/terrain.png");
	terrainTex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
	terrainTex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);
	geom->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, terrainTex.get(), osg::StateAttribute::ON);
	geom->getOrCreateStateSet()->addUniform(new osg::Uniform("u_terrainTex", 0));
	return geode;
}

osg::Geometry* CreateBoxFace(osg::Vec3 normal, osg::Vec3 up, float halfLen, std::string texFile)
{
	osg::Vec3 center = normal * halfLen;
	osg::Vec3 right = -normal ^ up;
	osg::Vec3 left = -right;
	osg::Vec3 down = -up;
	osg::Geometry* geom = new osg::Geometry();
	osg::Vec3Array* vertices = new osg::Vec3Array();
	osg::Vec3Array* normals = new osg::Vec3Array();
	osg::Vec2Array* uvs = new osg::Vec2Array();

	vertices->push_back(osg::Vec3(center + down * halfLen + left * halfLen));
	vertices->push_back(osg::Vec3(center + down * halfLen + right * halfLen));
	vertices->push_back(osg::Vec3(center + up * halfLen + left * halfLen));
	vertices->push_back(osg::Vec3(center + up * halfLen + right * halfLen));

	uvs->push_back(osg::Vec2(0, 0));
	uvs->push_back(osg::Vec2(1, 0));
	uvs->push_back(osg::Vec2(0, 1));
	uvs->push_back(osg::Vec2(1, 1));

	normals->push_back(normal);
	geom->setVertexArray(vertices);
	geom->setNormalArray(normals);
	geom->setTexCoordArray(0, uvs);
	geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
	geom->setCullingActive(false);
	geom->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP, 0, 4));
	geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osg::ref_ptr<osg::Texture2D> tex = LoadTexture(texFile);
	tex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
	tex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);
	tex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR_MIPMAP_NEAREST);
	tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR_MIPMAP_NEAREST);
	geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
	return geom;
}

osg::Node* CreateHouse()
{
	osg::Geode* geode = new osg::Geode();
	float halfLen = 0.5;
	geode->addDrawable(CreateBoxFace(osg::Vec3(0, 0, 1), osg::Vec3(0, 1, 0), halfLen, "./data/PositiveZ.png"));
	geode->addDrawable(CreateBoxFace(osg::Vec3(0, 0, -1), osg::Vec3(0, -1, 0), halfLen, "./data/NegativeZ.png"));
	geode->addDrawable(CreateBoxFace(osg::Vec3(1, 0, 0), osg::Vec3(0, 0, 1), halfLen, "./data/PositiveX.png"));
	geode->addDrawable(CreateBoxFace(osg::Vec3(-1, 0, 0), osg::Vec3(0, 0, 1), halfLen, "./data/NegativeX.png"));
	geode->addDrawable(CreateBoxFace(osg::Vec3(0, 1, 0), osg::Vec3(0, 0, 1), halfLen, "./data/PositiveY.png"));
	geode->addDrawable(CreateBoxFace(osg::Vec3(0, -1, 0), osg::Vec3(0, 0, 1), halfLen, "./data/NegativeY.png"));

	osg::ref_ptr<osg::Program> program = new osg::Program;
	osg::ref_ptr<osg::Shader> vertexShader = new osg::Shader(osg::Shader::VERTEX, LoadTextFile("./shaders/simple.tex.vert.glsl"));
	program->addShader(vertexShader.get());
	osg::ref_ptr<osg::Shader> fragmentShader = new osg::Shader(osg::Shader::FRAGMENT, LoadTextFile("./shaders/simple.tex.frag.glsl"));
	program->addShader(fragmentShader.get());
	geode->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON);

	//geode->setCullingActive(false);
	//geode->getOrCreateStateSet()->setMode(GL_CULL_FACE,
	//	osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	osg::ref_ptr <osg::BlendFunc> fn = new osg::BlendFunc();
	fn->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
	geode->getOrCreateStateSet()->setAttributeAndModes(fn.get(), osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	geode->getOrCreateStateSet()->addUniform(new osg::Uniform("u_texture0", 0));
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

osg::Node* createGrassSample(osg::BoundingBox bb, osg::Vec2 bladeWidthRange, osg::Vec2 bladeHeightRange, osg::Vec2 tiltRange, long bladesPerSqrtM)
{
	float minBladeWidth = bladeWidthRange.x();
	float maxBladeWidth = bladeWidthRange.y();
	float minBladeHeight = bladeHeightRange.x();
	float maxBladeHeight = bladeHeightRange.y();
	float minTilt = tiltRange.x();
	float maxTilt = tiltRange.y();
	float xmin = bb.xMin();
	float xmax = bb.xMax();
	float ymin = bb.yMin();
	float ymax = bb.yMax();
	long numBlades = (long)((xmax - xmin) * (ymax - ymin) * bladesPerSqrtM);
	GrassObject* geom = new GrassObject(osg::BoundingBox(xmin, ymin, minBladeWidth, xmax, ymax, maxBladeWidth * 10));
	geom->setCullingActive(false);
	osg::Vec3Array* vertices = new osg::Vec3Array();
	osg::Vec4Array* uvs = new osg::Vec4Array();
	size_t curNum = 0;
	while (curNum < numBlades)
	{
		vertices->push_back(osg::Vec3(xmin + (xmax - xmin) * GetRandomPosition(), ymin + (ymax - ymin) * GetRandomPosition(), GetRandomPosition()));
		//if (sqrt(x * x + y * y) > radius)
		//	continue;
		float rotate = GetRandomPosition() * 2 * osg::PI;
		float tilt = (minTilt + (maxTilt - minTilt) * GetRandomPosition()) * DEG2RAD;
		float h = minBladeHeight + (maxBladeHeight - minBladeHeight) * GetRandomPosition();
		float w = minBladeWidth + (maxBladeWidth - minBladeWidth) * GetRandomPosition();
		//uvs->push_back(osg::Vec4(GetRandomPosition(), GetRandomPosition(), GetRandomPosition(), 1));
		uvs->push_back(osg::Vec4(rotate, tilt, w, h));
		curNum++;
	}

	geom->setVertexAttribArray(0, vertices);
	geom->setVertexAttribBinding(0, osg::Geometry::BIND_PER_VERTEX);

	geom->setVertexAttribArray(1, uvs);
	geom->setVertexAttribBinding(1, osg::Geometry::BIND_PER_VERTEX);
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
	osg::ref_ptr<osg::Texture2D> grassTex = LoadTexture("./data/grassBlade2.png");
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
	geode->getOrCreateStateSet()->getOrCreateUniform("minTilt", osg::Uniform::FLOAT)->set(minTilt);
	geode->getOrCreateStateSet()->getOrCreateUniform("maxTilt", osg::Uniform::FLOAT)->set(maxTilt);
	geode->getOrCreateStateSet()->getOrCreateUniform("minRotate", osg::Uniform::FLOAT)->set(0.0f);
	geode->getOrCreateStateSet()->getOrCreateUniform("maxRotate", osg::Uniform::FLOAT)->set(360.0f);

	geode->getOrCreateStateSet()->getOrCreateUniform("top", osg::Uniform::FLOAT)->set(ymax);
	geode->getOrCreateStateSet()->getOrCreateUniform("left", osg::Uniform::FLOAT)->set(xmin);
	geode->getOrCreateStateSet()->getOrCreateUniform("right", osg::Uniform::FLOAT)->set(xmax);
	geode->getOrCreateStateSet()->getOrCreateUniform("bottom", osg::Uniform::FLOAT)->set(ymin);
	//geode->getOrCreateStateSet()->getOrCreateUniform("u_translate", osg::Uniform::FLOAT_VEC3)->set(u_translate);

	//geode->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("model", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("cameraPosition", osg::Uniform::FLOAT_VEC3);
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	geode->addChild(geom);
	return geode;
}

osg::Node* createGrassSampleGrid(float gridSize, osg::Vec2 bladeWidthRange, osg::Vec2 bladeHeightRange, osg::Vec2 tiltRange, long bladesPerSqrtM, int reps)
{
	osg::Group* grassGrid = new osg::Group();
	osg::BoundingBox bb(-gridSize * 0.5, -gridSize * 0.5, 0, gridSize * 0.5, gridSize * 0.5, gridSize * 0.5);
	osg::ref_ptr<osg::Node> grass = createGrassSample(bb, bladeWidthRange, bladeHeightRange, tiltRange, bladesPerSqrtM);
	int steps = reps % 2;
	for (int row = -steps; row <= steps; row++)
	{
		for (int col = -steps; col <= steps; col++)
		{
			osg::ref_ptr<osg::Group> mat = new osg::Group();
			osg::Vec3 u_translate(row * gridSize, col * gridSize, 0.0);
			//mat->setMatrix(osg::Matrix::translate(u_translate));
			mat->getOrCreateStateSet()->getOrCreateUniform("u_translate", osg::Uniform::FLOAT_VEC3)->set(u_translate);
			mat->addChild(grass.get());
			mat->setCullingActive(false);
			grassGrid->addChild(mat.get());
		}
	}
	grassGrid->setCullingActive(false);
	return grassGrid;
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

	geode->getOrCreateStateSet()->getOrCreateUniform("top", osg::Uniform::FLOAT)->set((float)ymax);
	geode->getOrCreateStateSet()->getOrCreateUniform("left", osg::Uniform::FLOAT)->set((float)xmin);
	geode->getOrCreateStateSet()->getOrCreateUniform("right", osg::Uniform::FLOAT)->set((float)xmax);
	geode->getOrCreateStateSet()->getOrCreateUniform("bottom", osg::Uniform::FLOAT)->set((float)ymin);

	//geode->getOrCreateStateSet()->getOrCreateUniform("projection", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("model", osg::Uniform::FLOAT_MAT4)->set(osg::Matrix::identity());
	//geode->getOrCreateStateSet()->getOrCreateUniform("cameraPosition", osg::Uniform::FLOAT_VEC3);

	geode->addChild(geom);
	return geode;
}

osg::Matrix createLookAt(osg::Vec3 eye, osg::Vec3 at, osg::Vec3 up)
{
	osg::Vec3 zaxis = osg::Vec3(at - eye);
	zaxis.normalize();
	osg::Vec3 xaxis = zaxis ^ up;
	xaxis.normalize();
	osg::Vec3 yaxis = xaxis ^ zaxis;
	yaxis.normalize();

	//negate(zaxis);
	zaxis = -zaxis;
	//osg::Matrix viewMatrix(
	//	xaxis.x(), xaxis.y(), xaxis.z(), -xaxis * eye,
	//	yaxis.x(), yaxis.y(), yaxis.z(), -yaxis * eye,
	//	zaxis.x(), zaxis.y(), zaxis.z(), -zaxis * eye,
	//	0, 0, 0, 1);

	osg::Matrix viewMatrix(
		xaxis.x(), yaxis.x(), zaxis.x(), 0,
		xaxis.y(), yaxis.y(), zaxis.y(), 0,
		xaxis.z(), yaxis.z(), zaxis.z(), 0,
		-xaxis * eye, -yaxis * eye, -zaxis * eye, 1);

	//osg::Matrix viewMatrix(
	//	xaxis.x(), xaxis.y(), xaxis.z(), 0,
	//	yaxis.x(), yaxis.y(), yaxis.z(), 0,
	//	zaxis.x(), zaxis.y(), zaxis.z(), 0,
	//	-xaxis * eye, -yaxis * eye, -zaxis * eye, 1);
	return viewMatrix;
}

float angleBetween(osg::Vec3 v0, osg::Vec3 v1)
{
	if (v0 == v1)
		return 0;

	v0.normalize();
	v1.normalize();

	osg::Vec3 n = v0 ^ v1;
	n.normalize();
	float dot = osg::Vec3(0, 0, 1) * n;
	float angle = osg::RadiansToDegrees(acos(v0 * v1));
	if (dot < 0)
		return 360 - angle;
	return angle;
}

osg::Vec2 vectorToTiltRotate(osg::Vec3 fromEye)
{
	fromEye.normalize();
	osg::Vec3 right = fromEye ^ osg::Vec3(0, 0, -1);
	right.normalize();
	float angleFromUp = 90 - angleBetween(fromEye, osg::Vec3(0, 0, 1));
	if (fromEye.z() > 0 && (angleFromUp < 0.5 || angleFromUp > 359.5))
		return osg::Vec2(90, 0);

	float tilt = angleFromUp;
	osg::Matrix tiltBack = osg::Matrix::rotate(tilt * DEG2RAD, right);
	fromEye = fromEye * tiltBack;
	float rotate = 360 - angleBetween(fromEye, osg::Vec3(0, -1, 0));
	if (tilt < 0)
		tilt = 0;
	return osg::Vec2(tilt, rotate);
}

osg::Vec2 rotateTex(osg::Vec3 fromEye)
{
	fromEye.normalize();
	osg::Vec3 right = fromEye ^ osg::Vec3(0, 0, -1);
	right.normalize();
	float angleFromUp = 90 - angleBetween(fromEye, osg::Vec3(0, 0, 1));
	//if (fromEye.z() > 0 && (angleFromUp < 10 || angleFromUp > 350))
	//	return osg::Vec2(90, 0);

	float tilt = angleFromUp;
	osg::Matrix tiltBack = osg::Matrix::rotate(tilt * DEG2RAD, right);
	fromEye = fromEye * tiltBack;
	float rotate = 360 - angleBetween(fromEye, osg::Vec3(0, -1, 0));
	if (tilt < 0)
		tilt = 0;
	return osg::Vec2(tilt, rotate);
}

//+ X{ _v = 0x000000f8d70ff358 {-5.73457289, -7.25949717, 9.00768280} }
//- X{ _v = 0x000000f8d70ff358 {-5.28047991, -7.40520525, 9.16533947} }
//Create a light
osg::ref_ptr <osg::Node> createLight()
{
	osg::ref_ptr <osg::Light> lt = new osg::Light;
	lt->setLightNum(0);
	lt->setDirection(osg::Vec3(1, 1, 1));
	lt->setPosition(osg::Vec4(500, 100, 1000, 1));
	//Set the color of the ambient light
	lt->setAmbient(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));

	osg::ref_ptr <osg::Light> lt2 = new osg::Light;
	lt2->setLightNum(0);
	lt2->setDirection(osg::Vec3(-1, -1, 1));
	lt2->setPosition(osg::Vec4(-500, -100, 1000, 1));
	//Set the color of the ambient light
	lt2->setAmbient(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));

	osg::ref_ptr <osg::LightSource> ls = new osg::LightSource();

	ls->setLight(lt.get());
	ls->setLight(lt2.get());
	return ls.get();
}

osg::BoundingBox m_grasslandBB(-1000, -1000, 0, 1000, 1000, 1);

bool canRenderGrass(osg::Vec3& gridCenter)
{
	osg::Vec3 corners[8];
	corners[0].set(-1.0, -1.0, -1.0);
	corners[1].set(1.0, -1.0, -1.0);
	corners[2].set(1.0, -1.0, 1.0);
	corners[3].set(-1.0, -1.0, 1.0);
	corners[4].set(-1.0, 1.0, -1.0);
	corners[5].set(1.0, 1.0, -1.0);
	corners[6].set(1.0, 1.0, 1.0);
	corners[7].set(-1.0, 1.0, 1.0);

	osg::Matrix view = m_viewCamera->getViewMatrix();
	osg::Matrix proj = m_viewCamera->getProjectionMatrix();

	osg::Matrixd clipToWorld;
	clipToWorld.invert(view * proj);

	// transform frustum corners from clipspace to world coords, and compute center
	for (size_t i = 0; i < 8; i++)
	{
		corners[i] = corners[i] * clipToWorld;
	}

	// compute eye point
	//osg::Vec3  eye = osg::Vec3d(0.0, 0.0, 0.0) * osg::Matrix::inverse(view);

	// compute center and the frustumCenterLine
	osg::Vec3 centerNearPlane = (corners[0] + corners[1] + corners[5] + corners[4]) * 0.25;
	osg::Vec3  centerFarPlane = (corners[3] + corners[2] + corners[6] + corners[7]) * 0.25;
	osg::Vec3  fcenter = (centerNearPlane + centerFarPlane) * 0.5;
	osg::Vec3  frustumCenterLine = centerFarPlane - centerNearPlane;
	frustumCenterLine.normalize();

	double fov, asp, znear, zfar;
	m_viewCamera->getProjectionMatrixAsPerspective(fov, asp, znear, zfar);
	osg::Vec3 eye, center, up;
	m_viewCamera->getViewMatrixAsLookAt(eye, center, up);
	////osg::Vec3 sceneCenter = osg::Vec3()
	osg::Vec3 v0 = osg::Vec3(m_grasslandBB.xMin(), m_grasslandBB.yMax(), 0);
	osg::Vec3 v1 = osg::Vec3(m_grasslandBB.xMin(), m_grasslandBB.yMin(), 0);
	osg::Vec3 v2 = osg::Vec3(m_grasslandBB.xMax(), m_grasslandBB.yMin(), 0);
	osg::Vec3 v3 = osg::Vec3(m_grasslandBB.xMax(), m_grasslandBB.yMax(), 0);
	osg::Vec3 look = center - eye;
	look.normalize();
	osg::Vec3 intersect = quadIntersect(eye, look, v0, v1, v2, v3);
	if (intersect.x() <= 0)
		return false;

	osg::Vec3 lookat = eye + look * intersect.x();
	int row = (int)(lookat.y() - m_grasslandBB.yMin());
	int col = (int)(lookat.x() - m_grasslandBB.xMin());
	int cellInView = 0;
	gridCenter = osg::Vec3(m_grasslandBB.xMin() + col + 0.5, m_grasslandBB.yMin() + row + 0.5, 0);
	for (int n = -8; n <= 8; n++)
	{
		float centerY = gridCenter.y() + n;
		if ((centerY - 0.5) < m_grasslandBB.yMin() || (centerY + 0.5) > m_grasslandBB.yMax())
			continue;

		for (int m = -8; m <= 8; m++)
		{
			float centerX = gridCenter.x() + m;
			if ((centerX - 0.5) < m_grasslandBB.xMin() || (centerX + 0.5) > m_grasslandBB.xMax())
				continue;

			if ((osg::Vec3(centerX, centerY, 0) - eye).length() > 10)
				continue;

			std::vector<osg::Vec4> corners;
			corners.push_back(osg::Vec4(centerX - 0.5, centerY + 0.5, 0, 1));
			corners.push_back(osg::Vec4(centerX - 0.5, centerY - 0.5, 0, 1));
			corners.push_back(osg::Vec4(centerX + 0.5, centerY - 0.5, 0, 1));
			corners.push_back(osg::Vec4(centerX + 0.5, centerY + 0.5, 0, 1));

			float xmin = FLT_MAX;
			float ymin = FLT_MAX;
			float xmax = -FLT_MAX;
			float ymax = -FLT_MAX;
			bool isInView = false;
			for (auto& p : corners)
			{
				osg::Vec4 projCoords = p * view * proj;
				projCoords.x() = projCoords.x() / projCoords.w();
				projCoords.y() = projCoords.y() / projCoords.w();
				if (abs(projCoords.x()) <= 1 && abs(projCoords.y()) <= 1)
				{
					isInView = true;
					cellInView++;
					break;
				}

				if (xmin > projCoords.x())
					xmin = projCoords.x();
				if (ymin > projCoords.y())
					ymin = projCoords.y();
				if (xmax < projCoords.x())
					xmax = projCoords.x();
				if (ymax < projCoords.y())
					ymax = projCoords.y();
			}
			//if (!isInView && xmin <= -1 && ymin <= -1 && xmax >= 1 && ymax >= 1)
				//cellInView++;
		}
	}

	return true;
}

struct GrassQuad
{
	osg::Vec3 center;
	double distToEye;
};

osg::Node* CreateBlackQuad(float xmin, float ymin, float xmax, float ymax, float z)
{
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
	osg::Geode* geode = new osg::Geode();
	osg::Vec3Array* vertices = new osg::Vec3Array();
	osg::Vec3Array* normals = new osg::Vec3Array();
	osg::Vec2Array* uvs = new osg::Vec2Array();
	vertices->push_back(osg::Vec3(xmin, ymin, z));
	uvs->push_back(osg::Vec2(0, ymax - ymin));
	vertices->push_back(osg::Vec3(xmin, ymax, z));
	uvs->push_back(osg::Vec2(0, 0));
	vertices->push_back(osg::Vec3(xmax, ymin, z));
	uvs->push_back(osg::Vec2(xmax - xmin, ymax - ymin));
	vertices->push_back(osg::Vec3(xmax, ymin, z));
	uvs->push_back(osg::Vec2(xmax - xmin, ymax - ymin));
	vertices->push_back(osg::Vec3(xmin, ymax, z));
	uvs->push_back(osg::Vec2(0, 0));
	vertices->push_back(osg::Vec3(xmax, ymax, z));
	uvs->push_back(osg::Vec2(xmax - xmin, 0));

	normals->push_back(osg::Vec3(0, 0, 1));
	geom->setVertexArray(vertices);
	//geom->setVertexAttribArray(0, vertices);
	//geom->setTexCoordArray(1, uvs);
	geom->setNormalArray(normals);
	geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, 6));

	//geom->setVertexAttribBinding(1, osg::Geometry::BIND_PER_VERTEX);
	geom->setCullingActive(false);
	geode->setCullingActive(false);
	geode->getOrCreateStateSet()->setMode(GL_CULL_FACE,
		osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, 6));

	geode->addDrawable(geom.get());
	//osg::ref_ptr<osg::Texture2D> terrainTex = LoadTexture("./data/terrain.png");
	//terrainTex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
	//terrainTex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);
	geom->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	//geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, terrainTex.get(), osg::StateAttribute::ON);
	//geom->getOrCreateStateSet()->addUniform(new osg::Uniform("u_terrainTex", 0));
	char vertexSource[] =
		"uniform vec3 u_translate;\n"
		"uniform mat4 projMat;\n"
		"uniform mat4 viewMat;\n"
		"void main(void)\n"
		"{\n"
		"   gl_TexCoord[0] = vec4(gl_Vertex.x*0.5+0.5,gl_Vertex.y*0.5+0.5,0,1);\n"
		"   gl_Position = projMat * viewMat * vec4(gl_Vertex.x+u_translate.x,gl_Vertex.y+u_translate.y,u_translate.z,1.0);\n"
		"}\n";

	char fragmentSource[] =
		"void main(void) \n"
		"{\n"
		"    gl_FragColor = vec4(0,0,0,1);\n"
		"}\n";

	osg::ref_ptr< osg::Program> program = new osg::Program;
	program->addShader(new osg::Shader(osg::Shader::VERTEX, vertexSource));
	program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentSource));
	geode->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->getOrCreateUniform("u_translate", osg::Uniform::FLOAT_VEC3)->set(osg::Vec3(0, 0, 0));
	return geode;
}


osg::Node* CreateColorQuad(float xmin, float ymin, float xmax, float ymax, float z, osg::Vec4 color)
{
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
	osg::Geode* geode = new osg::Geode();
	osg::Vec3Array* vertices = new osg::Vec3Array();
	osg::Vec3Array* normals = new osg::Vec3Array();
	osg::Vec2Array* uvs = new osg::Vec2Array();
	vertices->push_back(osg::Vec3(xmin, ymin, z));
	uvs->push_back(osg::Vec2(0, ymax - ymin));
	vertices->push_back(osg::Vec3(xmin, ymax, z));
	uvs->push_back(osg::Vec2(0, 0));
	vertices->push_back(osg::Vec3(xmax, ymin, z));
	uvs->push_back(osg::Vec2(xmax - xmin, ymax - ymin));
	vertices->push_back(osg::Vec3(xmax, ymin, z));
	uvs->push_back(osg::Vec2(xmax - xmin, ymax - ymin));
	vertices->push_back(osg::Vec3(xmin, ymax, z));
	uvs->push_back(osg::Vec2(0, 0));
	vertices->push_back(osg::Vec3(xmax, ymax, z));
	uvs->push_back(osg::Vec2(xmax - xmin, 0));

	normals->push_back(osg::Vec3(0, 0, 1));
	geom->setVertexArray(vertices);
	//geom->setVertexAttribArray(0, vertices);
	//geom->setTexCoordArray(1, uvs);
	geom->setNormalArray(normals);
	geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, 6));

	//geom->setVertexAttribBinding(1, osg::Geometry::BIND_PER_VERTEX);
	geom->setCullingActive(false);
	geode->setCullingActive(false);
	geode->getOrCreateStateSet()->setMode(GL_CULL_FACE,
		osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, 6));

	geode->addDrawable(geom.get());
	//osg::ref_ptr<osg::Texture2D> terrainTex = LoadTexture("./data/terrain.png");
	//terrainTex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
	//terrainTex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);
	geom->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	//geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, terrainTex.get(), osg::StateAttribute::ON);
	//geom->getOrCreateStateSet()->addUniform(new osg::Uniform("u_terrainTex", 0));
	char vertexSource[] =
		"uniform vec3 u_translate;\n"
		"void main(void)\n"
		"{\n"
		"   gl_TexCoord[0] = vec4(gl_Vertex.x*0.5+0.5,gl_Vertex.y*0.5+0.5,0,1);\n"
		"   gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.x+u_translate.x,gl_Vertex.y+u_translate.y,u_translate.z,1.0);\n"
		"}\n";

	char fragmentSource[] =
		"uniform vec4 u_color;\n"
		"void main(void) \n"
		"{\n"
		"    gl_FragColor = u_color;\n"
		"}\n";

	osg::ref_ptr< osg::Program> program = new osg::Program;
	program->addShader(new osg::Shader(osg::Shader::VERTEX, vertexSource));
	program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentSource));
	geode->getOrCreateStateSet()->setAttributeAndModes(program.get(), osg::StateAttribute::ON);
	//geode->getOrCreateStateSet()->getOrCreateUniform("u_translate", osg::Uniform::FLOAT_VEC3)->set(osg::Vec3(0, 0, 0));
	geode->getOrCreateStateSet()->getOrCreateUniform("u_color", osg::Uniform::FLOAT_VEC4)->set(color);
	return geode;
}

bool renderRealGrass(osg::Vec3& gridCenter, float resol)
{
	osg::Vec3 corners[8];
	corners[0].set(-1.0, -1.0, -1.0);
	corners[1].set(1.0, -1.0, -1.0);
	corners[2].set(1.0, -1.0, 1.0);
	corners[3].set(-1.0, -1.0, 1.0);
	corners[4].set(-1.0, 1.0, -1.0);
	corners[5].set(1.0, 1.0, -1.0);
	corners[6].set(1.0, 1.0, 1.0);
	corners[7].set(-1.0, 1.0, 1.0);

	osg::Matrix view = m_viewCamera->getViewMatrix();
	osg::Matrix proj = m_viewCamera->getProjectionMatrix();

	osg::Matrixd clipToWorld;
	clipToWorld.invert(view * proj);

	// transform frustum corners from clipspace to world coords, and compute center
	for (size_t i = 0; i < 8; i++)
	{
		corners[i] = corners[i] * clipToWorld;
	}

	// compute eye point
	//osg::Vec3  eye = osg::Vec3d(0.0, 0.0, 0.0) * osg::Matrix::inverse(view);

	// compute center and the frustumCenterLine
	osg::Vec3 centerNearPlane = (corners[0] + corners[1] + corners[5] + corners[4]) * 0.25;
	osg::Vec3  centerFarPlane = (corners[3] + corners[2] + corners[6] + corners[7]) * 0.25;
	osg::Vec3  fcenter = (centerNearPlane + centerFarPlane) * 0.5;
	osg::Vec3  frustumCenterLine = centerFarPlane - centerNearPlane;
	frustumCenterLine.normalize();

	double fov, asp, znear, zfar;
	m_viewCamera->getProjectionMatrixAsPerspective(fov, asp, znear, zfar);
	osg::Vec3 eye, center, up;
	m_viewCamera->getViewMatrixAsLookAt(eye, center, up);
	////osg::Vec3 sceneCenter = osg::Vec3()
	osg::Vec3 v0 = osg::Vec3(m_grasslandBB.xMin(), m_grasslandBB.yMax(), 0);
	osg::Vec3 v1 = osg::Vec3(m_grasslandBB.xMin(), m_grasslandBB.yMin(), 0);
	osg::Vec3 v2 = osg::Vec3(m_grasslandBB.xMax(), m_grasslandBB.yMin(), 0);
	osg::Vec3 v3 = osg::Vec3(m_grasslandBB.xMax(), m_grasslandBB.yMax(), 0);
	osg::Vec3 look = center - eye;
	look.normalize();
	osg::Vec3 intersect = quadIntersect(eye, look, v0, v1, v2, v3);
	if (intersect.x() <= 0)
		return false;

	osg::Vec3 lookat = eye + look * intersect.x();
	int row = (int)((lookat.y() - m_grasslandBB.yMin()) / resol);
	int col = (int)((lookat.x() - m_grasslandBB.xMin()) / resol);
	int cellInView = 0;
	gridCenter = osg::Vec3(m_grasslandBB.xMin() + col * resol + 0.5 * resol, m_grasslandBB.yMin() + row * resol + 0.5 * resol, 0);
	std::vector<GrassQuad> quads;
	float minDist = FLT_MAX;
	for (int n = -20; n <= 20; n++)
	{
		float centerY = gridCenter.y() + n * resol;
		if ((centerY - 0.5) < m_grasslandBB.yMin() || (centerY + 0.5) > m_grasslandBB.yMax())
			continue;

		for (int m = -20; m <= 20; m++)
		{
			float centerX = gridCenter.x() + m * resol;
			if ((centerX - 0.5) < m_grasslandBB.xMin() || (centerX + 0.5) > m_grasslandBB.xMax())
				continue;

			//if ((osg::Vec3(centerX, centerY, 0) - eye).length() > 2.5)
			//	continue;

			GrassQuad quad;
			quad.center = osg::Vec3(centerX, centerY, 0);

			osg::Vec3 centerToEye = quad.center - eye;
			centerToEye.normalize();
			if (centerToEye * look < 0)
				continue;
			//osg::Vec4 projCoords = osg::Vec4(centerX, centerY, 0, 1) * view * proj;
			//projCoords.x() = projCoords.x() / projCoords.w();
			//projCoords.y() = projCoords.y() / projCoords.w();
			//if (projCoords.x() < 0 || projCoords.x() > 1 || projCoords.y() < 0 || projCoords.y() > 1)
			//	continue;

			quad.distToEye = (osg::Vec3(centerX, centerY, 0) - eye).length();
			quads.push_back(quad);

			if (minDist > quad.distToEye)
				minDist = quad.distToEye;
		}
	}

	for (size_t i = 0; i < m_sampleGrassQuads->getNumChildren(); i++)
	{
		m_sampleGrassQuads->getChild(i)->setNodeMask(0x0);
	}

	if (minDist > 2)
		return true;

	std::sort(quads.begin(), quads.end(),
		[](const GrassQuad& x, const GrassQuad& y) {
			if (x.distToEye != y.distToEye) {
				return x.distToEye < y.distToEye;
			}

			// compare first names only if the last names are equal
			return x.distToEye < y.distToEye;
		});

	size_t maxQuads = 25;
	if (quads.size() < maxQuads)
		maxQuads = quads.size();

	for (size_t i = 0; i < maxQuads; i++)
	{
		if (i >= m_sampleGrassQuads->getNumChildren())
			break;

		auto grassQuad = (osg::Group*)m_sampleGrassQuads->getChild(i);
		//grassQuad->getChild(0)->setNodeMask(0x0);
		grassQuad->getOrCreateStateSet()->getOrCreateUniform("u_translate", osg::Uniform::FLOAT_VEC3)->set(quads[i].center);
		grassQuad->getChild(1)->getOrCreateStateSet()->getOrCreateUniform("u_translate", osg::Uniform::FLOAT_VEC3)->set(quads[i].center);

		grassQuad->setNodeMask(0x01);
	}

	return true;
}

osg::Node* loadModel(osg::Node* node, osg::Vec3 position, float scaledToHeight = NAN)
{
	osg::MatrixTransform* transform = new osg::MatrixTransform();
	transform->addChild(node);
	osg::ComputeBoundsVisitor cbv;
	transform->accept(cbv);
	osg::BoundingBox bb = cbv.getBoundingBox();
	osg::Matrix translate = osg::Matrix::translate(-bb.center().x() + position.x(), -bb.center().x() + position.y(), -bb.zMin() + position.z());
	float scaleFactor = isnan(scaledToHeight) ? 1.0f : scaledToHeight / (bb.zMax() - bb.zMin());
	osg::Matrix scale = osg::Matrix::scale(scaleFactor, scaleFactor, scaleFactor);
	transform->setMatrix(translate * scale);
	return transform;
}

osg::Node* loadModel(std::string filepath, osg::Vec3 position, float scaledToHeight = NAN)
{
	return loadModel(osgDB::readNodeFile(filepath), position, scaledToHeight);
}

void writeBoston()
{
	osg::Group* boston = (osg::Group*)osgDB::readNodeFile("./data/boston.ive");
	osg::ComputeBoundsVisitor vis;
	boston->accept(vis);
	auto bb = vis.getBoundingBox();
	osg::MatrixTransform* newGroup = new osg::MatrixTransform;
	osg::Vec3 center(bb.center().x() - 1000, bb.center().y() + 500, 0);
	double maxH = -1;
	int buildings = boston->getNumChildren();
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
	osg::BoundingBox blockBB(center.x() - 100, center.y() - 100, -100, center.x() + 100, center.y() + 100, 500);
	for (int i = 0; i < buildings; i++)
	{
		osg::Node* building = boston->getChild(i);
		osg::ComputeBoundsVisitor vis;
		building->accept(vis);
		auto bb2 = vis.getBoundingBox();
		if (!blockBB.contains(bb2.center()))
			continue;
		newGroup->addChild(building);
	}
	newGroup->setMatrix(osg::Matrix::translate(-center.x(), -center.y(), 0));
	osgDB::writeNodeFile(*newGroup, "./data/cityblock.ive");
}

void BlendImage(std::string imgFile1, std::string imgFile2, std::string outImgFile, int width, int height)
{
	osg::ref_ptr<osg::Image> img1 = osgDB::readImageFile(imgFile1);
	img1->scaleImage(width, height, 1);
	osg::ref_ptr<osg::Image> img2 = osgDB::readImageFile(imgFile2);
	img2->scaleImage(width, height, 1);

	osg::ref_ptr<osg::Image> img = new osg::Image;
	img->allocateImage(width, height, 1, GL_RGBA, GL_BYTE);
	size_t numPixels = (size_t)width * (size_t)height;
	RGBA* pData = (RGBA*)img->data();
	unsigned char* pData1 = img1->data();
	unsigned char* pData2 = img2->data();
	unsigned int pixelSize1 = img1->getPixelSizeInBits() / 8;
	unsigned int pixelSize2 = img2->getPixelSizeInBits() / 8;
	for (size_t i = 0; i < numPixels; i++)
	{
		pData->r = pData1[0];
		pData->g = pData1[1];
		pData->b = pData1[2];
		pData->a = pData2[0];

		pData++;
		pData1 += pixelSize1;
		pData2 += pixelSize2;
	}
	osgDB::writeImageFile(*img, outImgFile);
}


void TestMatrix()
{
	osg::Matrix view = osg::Matrix::lookAt(osg::Vec3(0, 0, 1), osg::Vec3(0, 0, 0), osg::Vec3(0, 1, 0));
	osg::Matrix proj = osg::Matrix::ortho(-0.5, 0.5, -0.5, 0.5, 1, 10);
	osg::Vec4 vertex(0, 0, 0, 1);
	osg::Vec4 projCoords = vertex * view * proj;
	projCoords = vertex * view * proj;
}

float UpdateLOD(float eyeAlt)
{
	eyeAlt = 10;
	float resol = eyeAlt > 5 ? m_lowResol : m_highResol;

	m_grassland->getOrCreateStateSet()->setTextureAttributeAndModes(0, m_renderGrassCamera->Texture().get(), osg::StateAttribute::ON);
	m_renderGrassCamera->getOrCreateStateSet()->getOrCreateUniform("u_resolution", osg::Uniform::FLOAT)->set(resol);
	m_grassland->getOrCreateStateSet()->getOrCreateUniform("u_resolution", osg::Uniform::FLOAT)->set(resol);
	if (eyeAlt > 5)
	{
		//m_resampleTex->getOrCreateStateSet()->addUniform(new osg::Uniform("u_reps", eyeAlt));
		//m_renderGrassCamera->getOrCreateStateSet()->getOrCreateUniform("u_resolution", osg::Uniform::FLOAT)->set(resol);
		//m_grassland->getOrCreateStateSet()->getOrCreateUniform("u_resolution", osg::Uniform::FLOAT)->set(resol * eyeAlt);
		//m_grassland->getOrCreateStateSet()->getOrCreateUniform("u_resolution", osg::Uniform::FLOAT)->set(resol);
		//m_grassland->getOrCreateStateSet()->setTextureAttributeAndModes(0, m_resampleTex->Texture().get(), osg::StateAttribute::ON);
		//m_grassland->getOrCreateStateSet()->setTextureAttributeAndModes(0, m_renderGrassCamera->Texture().get(), osg::StateAttribute::ON);
		//m_grassland->getOrCreateStateSet()->getOrCreateUniform("u_rotateTex", osg::Uniform::BOOL)->set(false);
		m_grassland->getOrCreateStateSet()->getOrCreateUniform("u_rotateTex", osg::Uniform::BOOL)->set(true);
		m_sampleGrassLowResol->setNodeMask(0x01);
		m_sampleGrassHighResol->setNodeMask(0x0);
	}
	else
	{
		m_grassland->getOrCreateStateSet()->getOrCreateUniform("u_rotateTex", osg::Uniform::BOOL)->set(true);
		m_sampleGrassLowResol->setNodeMask(0x0);
		m_sampleGrassHighResol->setNodeMask(0x01);
	}
	m_grassBB = osg::BoundingBox(-0.5 * resol, -0.5 * resol, 0, 0.5 * resol, 0.5 * resol, 0.5 * resol);
	return resol;
}


int main(int argc, char** argv)
{
	
	osgViewer::Viewer viewer;

	viewer.setUpViewInWindow(50, 50, 1024, 768);

	viewer.setThreadingModel(osgViewer::ViewerBase::ThreadingModel::ThreadPerContext);
	// add the state manipulator

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

	//viewer.addEventHandler(new ScreenRezieEventHandler);

	viewer.getCamera()->setClearColor(osg::Vec4(0, 0, 0, 0));
	

	osg::ref_ptr<osg::Group> root = new osg::Group;

	osg::Node* bigQuad = CreateColorQuad(-10, -10, 10, 10, 0, osg::Vec4(0, 1, 0, 1));
	osg::Node* smallQuadStencil = CreateColorQuad(-8.5, -8.5, -7.5, -7.5, 0, osg::Vec4(0, 0, 1, 1));
	osg::Node* smallQuad = CreateColorQuad(-0.5, -0.5, 0.5, 0.5, 0, osg::Vec4(1, 0, 0, 1));

	//##############################################################################################
	//Draw stencil
	osg::Stencil* stencil = new osg::Stencil;
	stencil->setFunction(osg::Stencil::ALWAYS, 1, ~0u);
	stencil->setOperation(osg::Stencil::KEEP, osg::Stencil::KEEP, osg::Stencil::REPLACE);
	stencil->setWriteMask(~0u);

	//// switch off the writing to the color bit planes.
	osg::ColorMask* colorMask = new osg::ColorMask;
	colorMask->setMask(false, false, false, false);
	osg::Depth* depth = new osg::Depth;
	//depth->setFunction(osg::Depth::NEVER);

	osg::StateSet* stateset = smallQuadStencil->getOrCreateStateSet();
	stateset->setRenderBinDetails(1, "RenderBin");
	stateset->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
	stateset->setAttributeAndModes(stencil, osg::StateAttribute::ON);
	//stateset->setAttribute(colorMask);
	//stateset->setAttribute(depth);
	root->addChild(smallQuadStencil);

	//##############################################################################################
	//Draw big quad
	stencil = new osg::Stencil;
	stencil->setFunction(osg::Stencil::NOTEQUAL, 1, ~0u);
	stencil->setOperation(osg::Stencil::KEEP, osg::Stencil::KEEP, osg::Stencil::KEEP);
	stencil->setWriteMask(~0u);

	// switch off the writing to the color bit planes.
	//colorMask = new osg::ColorMask;
	//colorMask->setMask(true, true, true, true);

	stateset = bigQuad->getOrCreateStateSet();
	stateset->setRenderBinDetails(2, "RenderBin");
	//stateset->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
	stateset->setAttributeAndModes(stencil, osg::StateAttribute::ON);
	//tateset->setAttribute(colorMask);
	root->addChild(bigQuad);

	//##############################################################################################
    //Draw small quad
	//stencil = new osg::Stencil;
	//stencil->setFunction(osg::Stencil::ALWAYS, 1, ~0u);
	//stencil->setOperation(osg::Stencil::KEEP, osg::Stencil::KEEP, osg::Stencil::KEEP);
	//stencil->setWriteMask(~0u);

	//// switch off the writing to the color bit planes.
	////colorMask = new osg::ColorMask;
	////colorMask->setMask(false, false, false, false);
	////depth = new osg::Depth;
	////depth->setFunction(osg::Depth::NEVER);

	//stateset = smallQuad->getOrCreateStateSet();
	//stateset->setRenderBinDetails(3, "RenderBin");
	////stateset->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
	//stateset->setAttributeAndModes(stencil, osg::StateAttribute::ON);
	////stateset->setAttribute(colorMask);
	//stateset->setAttribute(depth);
	//root->addChild(smallQuad);

	//root->addChild(CreateColorQuad(-0.5, -0.5, 0.5, 0.5, 0, osg::Vec4(1, 0, 0, 1)));

	root->addChild(osgDB::readNodeFile("C:/Code/OSGGrass/bin/data/cow.osg"));

	viewer.init();
	viewer.realize();
	viewer.setThreadingModel(osgViewer::ViewerBase::ThreadingModel::ThreadPerContext);
	// add the state manipulator
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	viewer.setCameraManipulator(new osgGA::TrackballManipulator);

	unsigned int clearMask = viewer.getCamera()->getClearMask();
	viewer.getCamera()->setClearMask(clearMask | GL_STENCIL_BUFFER_BIT);
	viewer.getCamera()->setClearColor(osg::Vec4(0, 0, 0, 0));
	viewer.getCamera()->setClearStencil(0);

	stencil = new osg::Stencil;
	stencil->setFunction(osg::Stencil::ALWAYS, 1, ~0u);
	stencil->setOperation(osg::Stencil::KEEP, osg::Stencil::KEEP, osg::Stencil::KEEP);
	viewer.getCamera()->getOrCreateStateSet()->setAttributeAndModes(stencil, osg::StateAttribute::ON);

	viewer.setSceneData(root);


	//viewer.getCameraManipulator()->setHomePosition(osg::Vec3(5.62419844, -7.12781620, 9.89352894) + (osg::Vec3(5.62785816, -7.73593903, 10.6873636) - osg::Vec3(5.62419844, -7.12781620, 9.89352894)) * 10, osg::Vec3(5.62419844, -7.12781620, 9.89352894), osg::Vec3(0.122764252, 0.788108468, 0.603169918));
	//viewer.getCameraManipulator()->setHomePosition(osg::Vec3(0, 0, 5.5), osg::Vec3(0, 0, 0), osg::Vec3(0, 1, 0));
	//viewer.getCameraManipulator()->setHomePosition(osg::Vec3(m_grasslandBB.xMin(), m_grasslandBB.yMin(), 0.5), osg::Vec3(m_grasslandBB.xMin(), m_grasslandBB.yMin(), 0), osg::Vec3(0, 1, 0));

	//viewer.getCameraManipulator()->home(0);

	while (!viewer.done())
	{
		viewer.frame();
	}

	return viewer.run();
}