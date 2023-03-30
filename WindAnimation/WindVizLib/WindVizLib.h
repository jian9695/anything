#pragma once
#include <vector>
#include <list>
#include <math.h>
#include <algorithm>

using namespace System;

namespace WindVizLib {

	template<class T>
	public struct Vec2
	{
	public:
		T m_x;
		T m_y;
		Vec2()
		{

		}
		Vec2(T x, T y)
		{
			m_x = x;
			m_y = y;
		}

		Vec2 operator*(const T& multiplier) const
		{
			return Vec2(m_x * multiplier, m_y * multiplier);
		}

		Vec2 operator*(const Vec2& other) const
		{
			return Vec2(m_x * other.m_x, m_y * other.m_y);
		}

		Vec2 operator/(const Vec2& other) const
		{
			return Vec2(m_x / other.m_x, m_y / other.m_y);
		}

		Vec2 operator+(const T& other) const
		{
			return Vec2(m_x + other.m_x, m_y + other.m_y);
		}

		Vec2 operator-(const Vec2& other) const
		{
			return Vec2(m_x - other.m_x, m_y - other.m_y);
		}

		Vec2 Floor()
		{
			return Vec2((floor)m_x, (floor)m_y);
		}

		Vec2 Reciprocal(const T& multiplier) const
		{
			return Vec2(m_x / multiplier, m_y / multiplier);
		}

		double Length() const
		{
			return m_x * m_x + m_y * m_y;
		}
	};

	public struct RGB
	{
	public:
		unsigned char m_r, m_g, m_b;
		RGB() { m_r = 0; m_g = 0; m_b = 0; };
		RGB(unsigned char r, unsigned char g, unsigned char b) { m_r = r; m_g = g; m_b = b; };
		RGB operator*(const double& multiplier) const
		{
			return RGB((unsigned char)(m_r * multiplier), (unsigned char)(m_g * multiplier), (unsigned char)(m_b * multiplier));
		}
		void Reset(unsigned char r, unsigned char g, unsigned char b) { m_r = r; m_g = g; m_b = b;}
		static RGB Red() { return RGB(255, 0, 0); }
		static RGB DarkRed() { return RGB(139, 0, 0); }
		static RGB Green() { return RGB(0, 255, 0); }
		static RGB Blue() { return RGB(0, 0, 255); }
		static RGB Yellow() { return RGB(255, 255, 0); }
		static RGB Navy() { return RGB(0, 0, 128); }
		static RGB Fuchsia() { return RGB(255, 0, 255); }
		static RGB Cyan() { return RGB(0, 255, 255); }
		static RGB Purple() { return RGB(128, 0, 128); }
		static RGB Black() { return RGB(0, 0, 0); }
		static RGB White() { return RGB(255, 255, 255); }
	};

	public struct RGBA : public RGB
	{
	public:
		unsigned char m_a;
		RGBA() { m_r = 0; m_g = 0; m_b = 0; m_a = 0; }
		RGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a) { m_r = r; m_g = g; m_b = b; m_a = a; };
		void Reset(unsigned char r, unsigned char g, unsigned char b, unsigned char a) { m_r = r; m_g = g; m_b = b; m_a = a; }
	};

	public struct FRGBA
	{
	public:
		double m_r, m_g, m_b, m_a;
		FRGBA() { m_r = 0; m_g = 0; m_b = 0; m_a = 0; };
		FRGBA(double r, double g, double b) { m_r = r; m_g = g; m_b = b; };
		FRGBA(double r, double g, double b, double a) { m_r = r; m_g = g; m_b = b; m_a = a; };
		FRGBA(RGB rgb) { m_r = rgb.m_r; m_g = rgb.m_g; m_b = rgb.m_b; };
		void Zero() { m_r = 0; m_g = 0; m_b = 0; m_a = 0; };
		void Reset(FRGBA rgba) { m_r = rgba.m_r; m_g = rgba.m_g; m_b = rgba.m_b; m_a = rgba.m_a; }
		FRGBA operator*(const double& multiplier) const
		{
			return FRGBA(m_r * multiplier, m_g * multiplier, m_b * multiplier);
		}

		FRGBA operator/(const double& multiplier) const
		{
			return FRGBA(m_r / multiplier, m_g / multiplier, m_b / multiplier);
		}

		FRGBA operator+(const FRGBA& other) const
		{
			return FRGBA(m_r + other.m_r, m_g + other.m_g, m_b + other.m_b);
		}
		FRGBA operator-(const FRGBA& other) const
		{
			return FRGBA(m_r - other.m_r, m_g - other.m_g, m_b - other.m_b);
		}
		RGB ToRGB() { return RGB((unsigned char)(std::min(m_r,255.0)), (unsigned char)(std::min(m_g, 255.0)), (unsigned char)(std::min(m_b, 255.0))); }
		RGBA ToRGBA() { return RGBA((unsigned char)(std::min(m_r, 255.0)), (unsigned char)(std::min(m_g, 255.0)), (unsigned char)(std::min(m_b, 255.0)), (unsigned char)(std::min(m_a, 255.0))); }
		double Length() { return sqrt(m_r * m_r + m_g * m_g + m_b * m_b); }
		FRGBA Normalize()
		{
			double len = Length();
			return FRGBA(m_r / len, m_g / len, m_b / len);
		}
	};

	public class ColorRamp
	{
	private:
		std::vector<std::pair<double, RGB>> m_parts;
	public:
		int NumParts() { return m_parts.size(); }
		void InsertPart(const double& frac, const RGB& rgb);
	  RGB Sample(double fraction);

		static ColorRamp VisibleSpectrum()
		{
			static ColorRamp visibleSpectrum;
			if (visibleSpectrum.NumParts() < 1)
			{
				visibleSpectrum.InsertPart(0.00, RGB::Fuchsia());
				visibleSpectrum.InsertPart(0.25, RGB::Blue());
				visibleSpectrum.InsertPart(0.50, RGB::Green());
				visibleSpectrum.InsertPart(0.75, RGB::Yellow());
				visibleSpectrum.InsertPart(1.00, RGB::Red());
			}
			return visibleSpectrum;
		}

		static ColorRamp ColorSpectrum()
		{
			static ColorRamp colorSpectrum;
			if (colorSpectrum.NumParts() < 1)
			{
				colorSpectrum.InsertPart(0.00, RGB::Navy());
				colorSpectrum.InsertPart(0.25, RGB::Blue());
				colorSpectrum.InsertPart(0.50, RGB::Green());
				colorSpectrum.InsertPart(0.75, RGB::Yellow());
				colorSpectrum.InsertPart(1.00, RGB::Red());
			}
			return colorSpectrum;
		}

		static ColorRamp SteppedColors()
		{
			static ColorRamp steppedColors;
			if (steppedColors.NumParts() < 1)
			{
				steppedColors.InsertPart(0.00, RGB::Navy());
				steppedColors.InsertPart(0.26, RGB::Green());
				steppedColors.InsertPart(0.50, RGB::Green());
				steppedColors.InsertPart(0.51, RGB::Yellow());
				steppedColors.InsertPart(0.75, RGB::Yellow());
				steppedColors.InsertPart(0.76, RGB::Red());
				steppedColors.InsertPart(1.00, RGB::Red());
			}
			return steppedColors;
		}

		static ColorRamp HeatedMetal()
		{
			static ColorRamp heatedMetal;
			if (heatedMetal.NumParts() < 1)
			{
				heatedMetal.InsertPart(0.00, RGB::Black());
				heatedMetal.InsertPart(0.40, RGB::Purple());
				heatedMetal.InsertPart(0.60, RGB::Red());
				heatedMetal.InsertPart(0.80, RGB::Yellow());
				heatedMetal.InsertPart(1.00, RGB::White());
			}
			return heatedMetal;
		}	

		static ColorRamp Incandescent()
		{
			static ColorRamp incandescent;
			if (incandescent.NumParts() < 1)
			{
				incandescent.InsertPart(0.00, RGB::Black());
				incandescent.InsertPart(0.33, RGB::DarkRed());
				incandescent.InsertPart(0.66, RGB::Yellow());
				incandescent.InsertPart(1.00, RGB::White());
			}
			return incandescent;
		}

		static ColorRamp BlueRed()
		{
			static ColorRamp blueRed;
			if (blueRed.NumParts() < 1)
			{
				blueRed.InsertPart(0.00, RGB::Blue());
				blueRed.InsertPart(1.00, RGB::Red());
			}
			return blueRed;
		}
	};


	public class WindMap
	{
	public:
		WindMap() {};
		void Initialize(array<System::Byte>^ windMap, int width, int height, int components);
		Vec2<double> GetSpeed(Vec2<int> pos);
		Vec2<double> GetSpeed(Vec2<double> pos, bool bilinear = true);
		Vec2<double> MaxSpeed() { return m_max; }
		Vec2<double> MinSpeed() { return m_min; }
		double VelocityScale() { return m_velocityScale; }
		void SetVelocityScale (double scale) 
		{ 
			m_velocityScale = scale;
			m_min = Vec2<double>(-scale, -scale);
			m_max = Vec2<double>(scale, scale);
		}
		double m_velocityScale;
	private:
		int m_width;
		int m_height;
		int m_stride;
		Vec2<double> m_min;
		Vec2<double> m_max;
		std::vector<Vec2<double>> m_map;
	};

	public ref class WindParticleEmitter
	{
	public:
		// TODO: Add your methods for this class here.
		 WindParticleEmitter(array<System::Byte>^ windMap, int width, int height, int components);
		 ~WindParticleEmitter();
		 void Initialize(int numParticles, double velocity);
		 void Render(int targetImageWidth, int targetImageHeight, array<System::Byte>^% pixels);
		 void Update();
	private:
		double m_velocity;
		int m_numParticles;
		Vec2<double>* m_particles;
		WindMap* m_windMap;
		double m_worldMinX;
		double m_worldMinY;
		double m_worldMaxX;
		double m_worldMaxY;
		double m_worldXRange;
		double m_worldYRange;
		RGBA* m_colorBuffer;
		int m_colorBufferWidth;
		int m_colorBufferHeight;
	private:
		double GetRand();
		void Reset(int stride);
		void ResetAll();
		Vec2<double> Local2World(const Vec2<double>& local) { return Vec2<double>(m_worldMinX + local.m_x * m_worldXRange, m_worldMaxY - local.m_y * m_worldYRange); }
		Vec2<double> World2Local(const Vec2<double>& world) { return Vec2<double>((world.m_x - m_worldMinX) / m_worldXRange, (m_worldMaxY - world.m_y) / m_worldYRange); }
		void RenderBlendedWithBackground(int targetImageWidth, int targetImageHeight, array<System::Byte>^% pixels);
		void RenderOnTopOfBackground(int targetImageWidth, int targetImageHeight, array<System::Byte>^% pixels);
	};
}
