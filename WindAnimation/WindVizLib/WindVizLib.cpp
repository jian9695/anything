#include "pch.h"
#include "WindVizLib.h"
#include <time.h>
#include <math.h>

using namespace WindVizLib;
template<typename T>
T bilinear(
	const double& tx,
	const double& ty,
	const T& c00,
	const T& c10,
	const T& c01,
	const T& c11)
{

	//T a = c00 * (1.f - tx) + c10 * tx;
	//T b = c01 * (1.f - tx) + c11 * tx;
	//return a * (1.f - ty) + b * ty;
	return (1 - tx) * (1 - ty) * c00 +
		tx * (1 - ty) * c10 +
		(1.f - tx) * ty * c01 +
		tx * ty * c11;
}

template<typename T>
Vec2<T> operator+(const Vec2<T>& one, const Vec2<T>& other)
{
	return Vec2<T>(one.m_x + other.m_x, one.m_y + other.m_y);
}

FRGBA operator*(const FRGBA& one, const FRGBA& other)
{
	return FRGBA(one.m_r * other.m_r, one.m_g * other.m_g, one.m_b * other.m_b, one.m_a * other.m_a);
}


void WindMap::Initialize(array<System::Byte>^ windMap, int width, int height, int components)
{
	m_width = width;
	m_height = height;
	m_stride = width * 4;
	m_map.clear();
	int index = 0;
	SetVelocityScale(1.0);
	for (int row = 0; row < height; row++)
	{
		for (int col = 0; col < width; col++)
		{
			Vec2<double> p = Vec2<double>((double)windMap[index + 2] / 255.0f, windMap[index + 1] / 255.0f);
			p = m_min + p * (m_max - m_min);
			m_map.push_back(p);
			index += components;
		}
	}
}

Vec2<double> WindMap::GetSpeed(Vec2<int> pos)
{
	return m_map[pos.m_y * m_width + pos.m_x] * m_velocityScale;
}

Vec2<double> WindMap::GetSpeed(Vec2<double> pos, bool bilinear)
{
	Vec2<double> size(m_width, m_height);
	Vec2<double> resol(1.0 / m_width, 1.0 / m_height);
	Vec2<int> index = Vec2<int>((int)(pos.m_x * size.m_x), (int)(pos.m_y * size.m_y));

	if (!bilinear)
	{
		if (index.m_x > m_width - 1)
			index.m_x = m_width - 1;
		if (index.m_y > m_height - 1)
			index.m_y = m_height - 1;
		return GetSpeed(index);
	}

	if (index.m_x >= m_width - 1)
		index.m_x = m_width - 2;
	if (index.m_y >= m_height - 1)
		index.m_y = m_height - 2;
	Vec2<double> frac = pos - Vec2<double>(index.m_x, index.m_y) / size;
	int left = index.m_x;
	int top = index.m_y;
	int right = left + 1;
	int bottom = top + 1;
	Vec2<double> c00 = GetSpeed(Vec2<int>(left, top));
	Vec2<double> c10 = GetSpeed(Vec2<int>(right, top));
	Vec2<double> c01 = GetSpeed(Vec2<int>(left, bottom));
	Vec2<double> c11 = GetSpeed(Vec2<int>(right, bottom));
	c00 = c00 * ((1.0 - frac.m_x) * (1.0 - frac.m_y));
	c10 = c10 * (frac.m_x * (1.0 - frac.m_y));
	c01 = c01 * ((1.0 - frac.m_x) * frac.m_y);
	c11 = c11 * (frac.m_x * frac.m_y);
	return c00 + c10 + c01 + c11;
}

//vec2 lookup_wind(const vec2 uv) {
//	// return texture2D(u_wind, uv).rg; // lower-res hardware filtering
//	vec2 px = 1.0 / u_wind_res;
//	vec2 vc = (floor(uv * u_wind_res)) * px;
//	vec2 f = fract(uv * u_wind_res);
//	vec2 tl = texture2D(u_wind, vc).rg;
//	vec2 tr = texture2D(u_wind, vc + vec2(px.x, 0)).rg;
//	vec2 bl = texture2D(u_wind, vc + vec2(0, px.y)).rg;
//	vec2 br = texture2D(u_wind, vc + px).rg;
//	return mix(mix(tl, tr, f.x), mix(bl, br, f.x), f.y);
//}

WindParticleEmitter::WindParticleEmitter(array<System::Byte>^ windMap, int width, int height, int components)
{
	m_particles = nullptr;
	srand(time(NULL));
	m_windMap = new WindMap();
	m_windMap->Initialize(windMap, width, height, components);
	m_worldXRange = 40075;
	m_worldYRange = m_worldXRange / 2.0;
	m_worldMaxX = m_worldXRange * 0.5;
	m_worldMaxY = m_worldYRange * 0.5;
	m_worldMinX = -m_worldMaxX;
	m_worldMinY = -m_worldMaxY;
	m_colorBuffer = nullptr;
  m_colorBufferWidth = -1;
  m_colorBufferHeight = -1;
}

WindParticleEmitter::~WindParticleEmitter()
{
	if (m_particles)
		delete[] m_particles;
	if (m_windMap)
		delete m_windMap;
	if (m_colorBuffer)
		delete[] m_colorBuffer;
}

void WindParticleEmitter::Initialize(int numParticles, double velocity)
{
	if (m_particles)
		delete[] m_particles;
	m_numParticles = numParticles;
	m_particles = new Vec2<double>[m_numParticles];
	m_windMap->SetVelocityScale(velocity);
	ResetAll();
}

void WindParticleEmitter::Render(int targetImageWidth, int targetImageHeight, array<System::Byte>^% pixels)
{
	RenderOnTopOfBackground(targetImageWidth, targetImageHeight, pixels);
	//RenderBlendedWithBackground(targetImageWidth, targetImageHeight, pixels);
}

void WindParticleEmitter::RenderBlendedWithBackground(int targetImageWidth, int targetImageHeight, array<System::Byte>^% pixels)
{
	int numPixels = targetImageWidth * targetImageHeight;
	int components = 4;
	//array<System::Byte>^ pixels = gcnew array<System::Byte>(numPixels * 4);
	m_colorBufferWidth = targetImageWidth;
	m_colorBufferHeight = targetImageHeight;
	RGBA* colorBuffer = new RGBA[numPixels];
	double alphaFade = 0.9;

	Vec2<double>* pParticles = m_particles;
	double maxSpeedLengh = m_windMap->MaxSpeed().Length();
	while (pParticles < m_particles + m_numParticles)
	{
		Vec2<double> pos = *pParticles++;
		if (pos.m_x > 1.0 || pos.m_y > 1.0)
			continue;

		int col = (int)(pos.m_x * targetImageWidth);
		int row = (int)(pos.m_y * targetImageHeight);
		if (col > targetImageWidth - 1)
			col = targetImageWidth - 1;
		if (row > targetImageHeight - 1)
			row = targetImageHeight - 1;
		int index = row * targetImageWidth + col;

		Vec2<double> speed = m_windMap->GetSpeed(pos);
		double frac = pow(speed.Length() / maxSpeedLengh, 0.5);
		if (frac > 1)
			frac = 1;
		RGB color2 = ColorRamp::Incandescent().Sample(frac);
		color2 = (FRGBA(color2) * 2.5).ToRGB();
		Vec2<double> speedFrac = (speed - m_windMap->MinSpeed()) / (m_windMap->MaxSpeed() - m_windMap->MinSpeed());
		RGB color = RGB(speedFrac.m_x * 255, speedFrac.m_y * 255, 0);
		//color = ((FRGBA(color) * 0.5 + FRGBA(color2) * 0.5) * 0.5).ToRGB();
		colorBuffer[index].Reset(color.m_r, color.m_g, color.m_b, 255);
	}

	if (m_colorBuffer)
	{
		for (int i = 0; i < numPixels; i++)
		{
			RGBA& color0 = m_colorBuffer[i];
			RGBA& color = colorBuffer[i];
			if (color.m_a > 0)
			{
				color0.m_a = (unsigned char)(255 * alphaFade);
			}
			else if (color0.m_a > 0)
			{
				color0.m_a = (unsigned char)(color0.m_a * alphaFade);
				color = color0;
			}
		}
	}

	for (int i = 0; i < numPixels; i++)
	{
		int index = i * components;
		RGBA& color = colorBuffer[i];
		pixels[index] = color.m_b;
		pixels[index + 1] = color.m_g;
		pixels[index + 2] = color.m_r;
		Byte a = color.m_a > 0 ? 255 : 0;
		pixels[index + 3] = a;
		if (color.m_a > 0)
		{
			if (m_colorBuffer)
				color.m_a = m_colorBuffer[i].m_a;
			else
				color.m_a = (unsigned char)(255 * alphaFade);
		}
	}

	if (m_colorBuffer)
		delete[] m_colorBuffer;
	m_colorBuffer = colorBuffer;
}

void WindParticleEmitter::RenderOnTopOfBackground(int targetImageWidth, int targetImageHeight, array<System::Byte>^% pixels)
{
	int numPixels = targetImageWidth * targetImageHeight;
	int components = 4;
	//array<System::Byte>^ pixels = gcnew array<System::Byte>(numPixels * 4);
	m_colorBufferWidth = targetImageWidth;
	m_colorBufferHeight = targetImageHeight;
	double alphaFade = 0.996;
	if (m_colorBuffer)
	{
		for (int i = 0; i < numPixels; i++)
		{
			RGBA& color = m_colorBuffer[i];
			// a hack to guarantee opacity fade out even with a value close to 1.0
			color.m_r = (unsigned char)floor(color.m_r * alphaFade);
			color.m_g = (unsigned char)floor(color.m_g * alphaFade);
			color.m_b = (unsigned char)floor(color.m_b * alphaFade);
			color.m_a = (unsigned char)floor(color.m_a * alphaFade);
		}
	}
	else
	{
		m_colorBuffer = new RGBA[numPixels];
	}

	Vec2<double>* pParticles = m_particles;
	double maxSpeedLengh = m_windMap->MaxSpeed().Length();
	while (pParticles < m_particles + m_numParticles)
	{
		Vec2<double> pos = *pParticles++;
		if (pos.m_x > 1.0 || pos.m_y > 1.0)
			continue;

		int col = (int)(pos.m_x * targetImageWidth);
		int row = (int)(pos.m_y * targetImageHeight);
		if (col > targetImageWidth - 1)
			col = targetImageWidth - 1;
		if (row > targetImageHeight - 1)
			row = targetImageHeight - 1;
		int index = row * targetImageWidth + col;

		Vec2<double> speed = m_windMap->GetSpeed(pos);
		double frac = pow(speed.Length() / maxSpeedLengh, 0.5);
		if (frac > 1)
			frac = 1;
		RGB color2 = ColorRamp::Incandescent().Sample(frac);
		color2 = (FRGBA(color2) * 2.5).ToRGB();
		Vec2<double> speedFrac = (speed - m_windMap->MinSpeed()) / (m_windMap->MaxSpeed() - m_windMap->MinSpeed());
		RGB color = RGB(speedFrac.m_x * 255, speedFrac.m_y * 255, 0);
		//color = ((FRGBA(color) * 0.5 + FRGBA(color2) * 0.5) * 0.5).ToRGB();
		m_colorBuffer[index].Reset(color.m_r, color.m_g, color.m_b, 255);
	}

	for (int i = 0; i < numPixels; i++)
	{
		int index = i * components;
		RGBA& color = m_colorBuffer[i];
		pixels[index] = color.m_b;
		pixels[index + 1] = color.m_g;
		pixels[index + 2] = color.m_r;
		Byte a = color.m_a > 0 ? 255 : 0;
		pixels[index + 3] = a;
	}
}

double WindParticleEmitter::GetRand()
{
	return (rand() / (double)RAND_MAX);
}

size_t FrameCount = 0;

void WindParticleEmitter::Update()
{
	FrameCount++;
	//reset one in every five particles
	if (FrameCount % 3 == 0)
		Reset(10);
	for (size_t i = 0; i < m_numParticles; i++)
	{
		Vec2<double> pos = m_particles[i];
		Vec2<double> speed = m_windMap->GetSpeed(pos);
		pos = Local2World(pos);
		pos = pos + speed;
		pos = World2Local(pos);
		if (pos.m_x < 0 || pos.m_y < 0 || pos.m_x > 1 || pos.m_y > 1)
		{
			pos.m_x = GetRand();
			pos.m_y = GetRand();
		}
		m_particles[i] = pos;
	}
}

void WindParticleEmitter::Reset(int stride)
{
	int start = rand() % stride;
	int index = start;
	while (index < m_numParticles)
	{
		m_particles[index].m_x = GetRand();
		m_particles[index].m_y = GetRand();
		index += stride;
	}
}

void WindParticleEmitter::ResetAll()
{
	for (size_t i = 0; i < m_numParticles; i++)
	{
		m_particles[i].m_x = GetRand();
		m_particles[i].m_y = GetRand();
	}
}

RGB ColorRamp::Sample(double fraction)
{
	if (fraction == 0)
		return m_parts[0].second;
	else if (fraction == 1)
		return m_parts[m_parts.size() - 1].second;

	for (int i = m_parts.size() - 1; i >= 0; i--)
	{
		if (fraction >= m_parts[i].first)
		{
			FRGBA start = m_parts[i].second;
			FRGBA end = m_parts[i + 1].second;
			double frac = (fraction - m_parts[i].first) / (m_parts[i + 1].first - m_parts[i].first);
			return (start + (end - start) * frac).ToRGB();
		}
	}

	return m_parts[0].second;
}

void ColorRamp::InsertPart(const double& frac, const RGB& rgb)
{
	std::pair<double, RGB> part(frac, rgb);
	if (m_parts.size() == 0 || frac >= m_parts[m_parts.size() - 1].first)
	{
		m_parts.push_back(part);
		return;
	}

	std::vector<std::pair<double, RGB>> newParts;
	bool found = false;
	for (int i = 0; i < m_parts.size(); i++)
	{
		if (!found && frac >= m_parts[i].first)
		{
			m_parts.push_back(part);
			found = true;
		}
		m_parts.push_back(m_parts[i]);
	}
}
