#include "frustum.h"


Ray::Ray() : startPoint(Vector4f(0.f)),direct(Vector4f(0.f,.8f,-.6f)),distTotal(0.f),begin(1e-8f),end(1.0e6f), intersectFace(NULL),intersected(false){}

Ray::Ray(Vector4f start, Vector4f drct, float dist) : startPoint(start), direct(drct), distTotal(dist), begin(1e-6f), end(1.0e6f), intersectFace(NULL), intersected(false)
{
}

Ray::~Ray()
{
}

void Ray::Move(Vector4f displacement)
{
	startPoint += displacement;
	ResetInterset();
}

const float Frustum::angleLim = 1.e-2;

Frustum::Frustum()
{
	vector<Vector4f> tmp = {Vector4f(1,1,1),Vector4f(-1,1,1),Vector4f(-1,-1,1),Vector4f(1,-1,1)};
	Frustum(Vector4f(0), tmp);
}

Frustum::Frustum(Vector4f vert, vector<Vector4f> verts, float amp) : vertex(vert), verts(verts), amp(amp)
{
	Init();
}

vector<Frustum> Frustum::DivideFrustum()
{
	vector<Frustum> vecFsm;
	float l1, l2, lambda;
	Vector4f mp[5];
	int vertNum = verts.size();
	for (int i = 0; i < vertNum; i++)
	{
		int ii = (i + 1) % vertNum;
		l1 = corner[i].GetBegin();
		l2 = corner[ii].GetBegin();
		lambda = l2 / (l1 + l2);
		mp[i] = verts[i] * lambda + verts[ii] * (1 - lambda);
	}
	l1 = (mp[0] - vertex).GetLenght();
	l2 = (mp[2] - vertex).GetLenght();
	lambda = l2 / (l1 + l2);
	mp[4] = mp[0] * lambda + mp[2] * (1 - lambda);
	for (int i = 0; i < vertNum; i++)
	{
		int ii = (i + vertNum - 1) % vertNum;
		vector<Vector4f> vts = { verts[i],mp[i],mp[4],mp[ii] };
		vecFsm.push_back(Frustum(vertex, vts));
	}
	return vecFsm;
}

void Frustum::cutWithPlane(Vector4f pl)
{
	for (auto& ray : corner)
	{
		float proj = Vector4f::Dot3f(ray.GetDirect(), pl), t;
		if (abs(proj) > 1e-10)
			t = -(Vector4f::Dot3f(ray.GetStartPt(), pl) + pl.w) / proj;
		else
			t = 1e6;
		if (proj <= 0)
			ray.SetEnd(t<ray.GetEnd() ? t>ray.GetBegin() ? t : ray.GetBegin() : ray.GetEnd());
		else
			ray.SetEnd(t<ray.GetEnd() ? t>ray.GetBegin() ? t : ray.GetEnd() : ray.GetEnd());
	}
	farPlane = pl;
}

void Frustum::Init()
{
	//init corner rays
	for (auto vertFar : verts)
	{
		Vector4f drct = vertFar - vertex;
		float begin = drct.GetLenght();
		drct.SetLenght(1.0);
		corner.push_back(Ray(vertex, drct));
		corner.back().SetBegin(begin);
	}
	//init face normal vectors
	Vector4f d1, d2, n;
	float sign;
	d1 = corner.back().GetDirect();
	d2 = corner.front().GetDirect();
	sign = Vector4f::Dot3f(Vector4f::Cross3f(d1, d2), corner[1].GetDirect()) > 0 ? 1 : -1;
	n = Vector4f::Cross3f(d1, d2)*sign;
	n.SetLenght(1.0);
	n.w = -(Vector4f::Dot3f(n, vertex));
	norm.push_back(n);
	for (int i = 1; i < corner.size(); i++)
	{
		d1 = d2;
		d2 = corner[i].GetDirect();
		n = Vector4f::Cross3f(d1, d2)*sign;
		n.SetLenght(1.0);
		n.w = -(Vector4f::Dot3f(n, vertex));
		norm.push_back(n);
	}
	d1 = verts[1] - verts[0];
	d2 = verts[2] - verts[1];
	n = Vector4f::Cross3f(d1, d2);
	sign = Vector4f::Dot3f(n, corner.front().GetDirect()) > 0 ? 1 : -1;
	n = n*sign;
	n.SetLenght(1.0);
	n.w = -(Vector4f::Dot3f(n, verts.front()));
	norm.push_back(n);
	//calculate solidAngel
	solidAngle = 0;
	for (int i = 0; i < verts.size(); i++)
	{
		int ii = (i + 1) % verts.size();
		solidAngle -= acos(Vector4f::Dot3f(norm[i], norm[ii]));
	}
	solidAngle += 2 * PI;
}
