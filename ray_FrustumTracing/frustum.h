#ifndef FRUSTUM_H
#define FRUSTUM_H

#include<vector>
#include"mathdefs.h"
#include"OBB.h"

class Ray
{
public:
	Ray();
	Ray(Vector4f start, Vector4f drct, float amp = 1.f);
	~Ray();
	Vector4f GetDirect() const { return direct; };
	Vector4f GetStartPt() const { return startPoint; };
	float GetBegin() const { return begin; };
	void SetBegin(float bg) { begin = bg; }
	float GetEnd() const { return end; };
	void SetEnd(float ed) { end = ed; }
	faceInfo* GetFace() const { return intersectFace; }
	void SetFace(faceInfo* f) { intersectFace = f; }
	bool IsIntersect() const { return intersected; }
	void SetIntersect() { intersected = true; }
	void ResetInterset() { intersected = false; end = 1e6; }
	void Move(Vector4f displacement);

private:
	Vector4f startPoint;//Start point of the ray
	Vector4f direct;//Indicating the direction of the ray
	float begin, end;//These two numbers represent the interval of the param of the line's param represent
	float amp;// The energy this ray carries
	faceInfo* intersectFace;
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

	FsmNode() :colli(false) {};
	FsmNode(Frustum frustum)
	{
		fsm = frustum;
		colli = false;
	}
};
#endif
