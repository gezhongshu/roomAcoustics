#ifndef FRUSTUM_H
#define FRUSTUM_H

#include<vector>
#include"mathdefs.h"
#include"OBB.h"

class Ray
{
public:
	Ray();
	Ray(Vector4f start, Vector4f drct, float dist = 0.f);
	Ray(Vector4f start, Vector4f drct, Vector4f normRef, float dist = 0.f);
	~Ray();
	Vector4f GetDirect() const { return direct; };
	Vector4f GetStartPt() const { return startPoint; };
	Vector4f GetBeginPt() { return startPoint + direct*double(begin); }
	float GetBegin() const { return begin; };
	void SetBegin(float bg) { begin = bg; }
	float GetEnd() const { return end; };
	void SetEnd(float ed) { end = ed; }
	float GetDist() { return distTotal; }
	faceInfo* GetFace() const { return intersectFace; }
	Vector4f GetRef() { return reflectNorm; }
	void SetFace(faceInfo* f) { intersectFace = f; }
	bool IsIntersect() const { return intersected; }
	void SetIntersect() { intersected = true; }
	void ResetInterset() { intersected = false; end = 1e6; }
	void Move(Vector4f displacement);

private:
	Vector4f startPoint;//Start point of the ray
	Vector4f direct,reflectNorm;//Indicating the direction of the ray
	float begin, end;//These two numbers represent the interval of the param of the line's param represent
	float distTotal;// The dist this ray travels
	faceInfo *intersectFace;
	bool intersected;
};

class Frustum
{
public:
	Frustum();
	~Frustum() {};
	Frustum(Vector4f vert, vector<Vector4f> verts, float amp = 1.f);
	inline vector<Vector4f>& GetNorm() { return norm; };
	inline vector<Ray>& GetCorner() { return corner; };
	inline vector<Ray> GetConstCorner() { return corner; };
	inline Vector4f GetVertex() const { return vertex; };
	inline vector<Vector4f> GetVerts() const { return verts; };
	inline Vector4f& GetRefPlane() { return farPlane; };
	inline float GetSolidAngle() { return solidAngle; };
	inline void AddObb(OBB* obb) { obbs.push_back(obb); };
	inline int GetObbNum() { return obbs.size(); };
	inline vector<OBB*> GetObbs() { return obbs; };
	inline bool Dividable() { return obbs.size() > 2 && angleLim < solidAngle; };
	vector<Frustum> DivideFrustum();
	void cutWithPlane(Vector4f pl);

private:
	//Sideface f1, f2, f3, f4;
	Vector4f vertex, farPlane;
	vector<Ray> corner;
	vector<Vector4f> verts, norm;
	vector<OBB*> obbs;
	float solidAngle;
	float amp;
	static const float angleLim;

	void Init();
};

struct FsmNode
{
	Frustum fsm;
	//vector<FsmNode> *child;
	//vector<Vector4f> points;
	vector<faceInfo*> faces;
	bool colli;
	Vector4f direct;
	float distTotal;// The dist this ray travels

	FsmNode() :colli(false), direct(Vector4f(1,0,0)), distTotal(0) {};
	FsmNode(Frustum frustum, float dist = 0, Vector4f drct = Vector4f(1, 0, 0))
	{
		fsm = frustum;
		colli = false;
		direct = drct;
		distTotal = dist;
	}
	Vector4f GetDirect() { return direct; };
	bool IsIntersect() { return colli; }
	float GetDist() { return distTotal; }
	float TravelDist()
	{
		vector<Vector4f> verts = GetRayCollides();
		Vector4f c(0);
		for (auto v : verts)
			c += v;
		return (c - fsm.GetVertex()).GetLenght();
	}
	faceInfo* GetFace()
	{
		float area = 0;
		faceInfo* face = 0;
		for (auto f:faces)
			if (f->area > area)
			{
				area = f->area;
				face = f;
			}
		return face;
	}
	vector<Vector4f> GetRayCollides()
	{
		vector<Vector4f> collis;
		for (auto r : fsm.GetCorner())
			if (r.IsIntersect())
				collis.push_back(r.GetStartPt() + r.GetDirect()*(r.GetEnd()-EPS));
		return collis;
	}
};

struct RayNode
{
	Frustum fsm;
	Ray ray;

	RayNode(Frustum frustum, Ray ray) :fsm(frustum), ray(ray) {};
	RayNode(Frustum frustum, Vector4f start, Vector4f drct, float dist = 0)
	{
		fsm = frustum;
		ray = Ray(start, drct, dist);
	}
	RayNode(Frustum frustum, Vector4f start, Vector4f drct, Vector4f normRef, float dist = 0)
	{
		fsm = frustum;
		ray = Ray(start, drct, normRef, dist);
	}
	Vector4f GetDirect() { return ray.GetDirect(); }
	bool IsIntersect() { return ray.IsIntersect(); }
	float GetDist() { return ray.GetDist(); }
	faceInfo* GetFace()	{ return ray.GetFace();	}
	void CutFsm()
	{
		if (!ray.IsIntersect())return;
		Vector4f d, vref;
		vector<Vector4f> verts;
		d = Vector4f(0) - ray.GetDirect();
		vref = ray.GetStartPt() - d * ray.GetEnd();
		d.w = -Vector4f::Dot3f(d, vref);
		fsm.cutWithPlane(d);
	}
};


template <class T> struct BiNode
{
	T data;//������
	bool isScat;//�����Ƿ���ɢ��
	BiNode<T> *left, *right;//ָ�������ӽ���ָ��

	BiNode(const T &data, BiNode<T> *left = 0, BiNode<T> *right = 0) :
		data(data), isScat(false), left(left), right(right) {};
	void relase()
	{
		if (left != NULL)left->relase();
		if (right != NULL)right->relase();
		delete this;
	};
};

#endif
