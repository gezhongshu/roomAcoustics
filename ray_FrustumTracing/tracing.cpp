#include "tracing.h"

double Tracing::angleLim = 0.03;
double Tracing::egyCoef = 1;
int Tracing::scatCount = 0;
double Tracing::solid = 0;
int counth = 0;
double maxh = 0;
Vector4f Tracing::pos_s = Vector4f();

void Tracing::AddImpulseResponse(vector<vector<double>>& hrir, const vector<vector<double>>& sDrct, vector<int> refs, vector<int> mirs, vector<int> scats, int band, Vector4f vec, int id, double len, bool scatFlag)
{
	int nref = 0;
	for (auto r : refs)
		nref += r;
	//nref = 1 - 2 * (nref % 2);
	nref = 0;
	vector<double> bRef, hrir_s;
	vector<COMPLEX> cRef;
	double delay = id - len * FS / SOUND_SPEED;
	vector<double> phaseAdj = WallAirAbsorb::FreqMult(delay * 2 * pi / FS);
	bRef = WallAirAbsorb::Absorb(len, refs, mirs, scats, band);
	double egy = 0, eh = 0, ec = 0;
	if (wideBand < -1)
	{
		for (int i = 0; i < bRef.size(); i++)
			cRef.push_back(Mul(COMPLEX(bRef[i], 0), COMPLEX(cos(phaseAdj[i]), sin(phaseAdj[i]))));
		for (auto c : cRef)
			ec += c.Energy();
		hrir_s = WallAirAbsorb::InterpIFFT(cRef);
	}
	else
		hrir_s = bRef;
	vector<vector<double>> hrir_D;
	double max_abs = 0;
	int offset = 0;
	hrir_D = WallAirAbsorb::ConvHrir(hrir_s, sDrct);
	for (int i = 0; i < hrir_D[0].size(); i++)
		if (max_abs < abs(hrir_D[0][i] + hrir_D[1][i]))
		{
			max_abs = abs(hrir_D[0][i] + hrir_D[1][i]);
			offset = i;
		}
	double coef = scatFlag ? egyCoef : 1;
	for (int ihs = 0; ihs < hrir_D[0].size(); ihs++)
	{
		int ind = id + ihs - offset;
		if (ind >= LEN_RIR)break;
		if (ind < 0)continue;
		mu_hrir.lock();
		hrir[ind][0] += coef*hrir_D[0][ihs];
		hrir[ind][1] += coef*hrir_D[1][ihs];
		mu_hrir.unlock();
	}
}

void Tracing::TracingInRoom(vector<BiNode<Ray>*>& rays, OBBTree * tree, int ref, Vector4f s, int nCircle)
{
	//nCircle = 4;
	float dTheta = PI / nCircle;
	Vector4f source = s;
	int numRay = 0;
	for (float i = 0.5f; i < nCircle; i++)
	{
		float theta = i*dTheta;
		int m = round(2 * PI*sin(theta) / dTheta);
		int totalTask = 0;
		vector<thread> tasks;
		for (float j = 0.5f; j < m; j++)
		{
			numRay++;
			float phi = j*PI * 2 / m;
			Vector4f drct = Vector4f(sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta));
			//double amp = DirectBeam(drct);
			Ray ray = Ray(source, drct);
			OBBIntersection::CollisionTest(&ray, tree);
			rays.push_back(new BiNode<Ray>(ray));
			rays.back()->isScat = ref > 0;
			while (totalTask > maxThread) Sleep(0);
			//cout << "max threads: " << thread::hardware_concurrency() * 2 << endl;
			mu_thread.lock();
			totalTask++;
			string s;
			s = "Running: " + to_string(int(i + 1)) + " / " + to_string(nCircle)
				+ ", number of threads: " + to_string(totalTask);
			for (int i = 0; i < s.size(); i++)
				printf("\b");
			//cout << s << flush;
			printf("%s", s.c_str());
			mu_thread.unlock();
			tasks.push_back(thread(RayTracingParallel<Ray>, rays.back(), tree, ref, &totalTask));
			tasks.back().detach();
			//RayTracing(rays.back(), tree, ref);
		}
		while (totalTask > 0) Sleep(1);
	}
	egyCoef = 1 / numRay;
	cout << "\nReal number of rays: " << numRay << endl;
}

void Tracing::RefRay(BiNode<Ray>* pr, bool divide)
{
	Vector4f d0, d1, n, start;
	float dist = pr->data.GetDist();
	d0 = pr->data.GetDirect();
	n = pr->data.GetFace()->faceNorm;
	start = pr->data.GetStartPt() + pr->data.GetDirect()*(pr->data.GetEnd() - 1e-5);
	if (divide)
	{
		float elev = acos(1 - 2 * (float)rand() / RAND_MAX) / 2;
		float azim = PI * 2 * (float)rand() / RAND_MAX;
		Vector4f z = Vector4f(0.f, 0.f, 1.f);
		float theta = acos(Vector4f::Dot3f(z, n));
		Vector4f c = Vector4f::Cross3f(z, n);
		if (c.GetLenght() < EPS)
			c = Vector4f(0.f, 1.f, 0.f);
		else
			c.SetLenght(1.f);
		Matrix4x4f w = Matrix4x4f();
		w.r0 = Vector4f(0.f, -c[2], c[1]);
		w.r1 = Vector4f(c[2], 0.f, -c[0]);
		w.r2 = Vector4f(-c[1], c[0], 0.f);
		Matrix4x4f eye = Matrix4x4f();
		eye.LoadIdentity();
		Matrix4x4f R = eye + w*sin(theta) + w * w * (1 - cos(theta));
		d1 = R * Vector4f(sin(elev)*cos(azim), sin(elev)*sin(azim), cos(elev));
		d1.SetLenght(1.0f);
		pr->left = new BiNode<Ray>(Ray(start, d1, n, dist + pr->data.GetEnd()));
		pr->left->isScat = divide;

	}
	float proj = Vector4f::Dot3f(d0, n);
	d1 = d0 - n*proj * 2;
	d1.SetLenght(1.0f);
	pr->right = new BiNode<Ray>(Ray(start, d1, n, dist + pr->data.GetEnd()));
	pr->right->isScat = divide;
}

void Tracing::ColliRay(Ray * ray, OBBTree * tree)
{
	OBBIntersection::CollisionTest(ray, tree);
}

void Tracing::PassReceiver(BiNode<Ray>* ray, Orient& rec, const vector<double>& sDrct, vector<vector<double>>& hrir, vector<int>& refs, vector<int>& mirs, vector<int>& scats, int band)
{
	int id, n = hrir.size();
	double chordLen, proj;
	Vector4f vec;
	vector<double> bRef, hrir_s;
	vector<COMPLEX> cRef;
	Ray& r = ray->data;

	vec = r.GetStartPt() - rec.GetPos();
	proj = -Vector4f::Dot3f(vec, r.GetDirect());
	chordLen = Vector4f::Cross3f(vec, r.GetDirect()).GetLenght();
	//HRIR::JudgeDirection(vec, front, up, hrir_s);
	double len = ray->data.GetDist() + vec.GetLenght();
	vec.SetLenght(len);
	id = (int)round(len * FS / SOUND_SPEED);
	if (id >= n+256 || Vector4f::Dot3f(rec.GetPos() - r.GetStartPt(), r.GetRef()) + r.GetRef().w < 0)return;
	vector<vector<double>> hr = HRIR::EvalAmp(rec.LocalPolar(r.GetStartPt() - rec.GetPos()));
	vector<vector<double>> sDHrir = WallAirAbsorb::ConvHrir(sDrct, hr);
	int n_r = 0;
	for (auto r : refs)
		n_r += r;
	if (n_r)
		AddImpulseResponse(hrir, sDHrir, refs, vector<int>(refs.size(), 0), scats, band, vec, id, len, true);
	if(chordLen / len <= Tracing::angleLim && proj <= r.GetEnd())
		AddImpulseResponse(hrir, sDHrir, refs, mirs, vector<int>(refs.size(), 0), band, vec, id, len);
}


void Tracing::TracingInRoom(vector<BiNode<FsmNode>*>& rays, OBBTree * tree, int ref, Vector4f s)
{
	Vector4f source = s;
	int numRay = 0;
	float delta = 2;

	for (float elev = -90 + 2 * delta; 90 - elev - delta> EPS; elev += 2 * delta)
	{
		float theta[] = { elev - delta, elev + delta };
		int totalTask = 0;
		vector<thread> tasks;
		for (float azim = 0; 360 - azim - delta > EPS; azim += 2 * delta)
		{
			numRay++;
			float phi[] = { azim - delta, azim + delta };
			Vector4f drct = Vector4f(cos(elev*pi / 180)*cos(azim*pi / 180), cos(elev*pi / 180)*sin(azim*pi / 180), sin(elev*pi / 180));
			vector<Vector4f> verts;
			for (int i = 0; i < 2; i++)
				for (int j = 0; j < 2; j++)
					verts.push_back(Vector4f(s.x + cos(theta[i] * pi / 180)*cos(phi[j^i] * pi / 180) * 0.1,
						s.y + cos(theta[i] * pi / 180)*sin(phi[j^i] * pi / 180) * 0.1, s.z + sin(theta[i] * pi / 180) * 0.1));
			Frustum ray = Frustum(s, verts);
			OBBIntersection::CollisionTest(&ray, tree);
			rays.push_back(new BiNode<FsmNode>(FsmNode(ray, 0, drct)));
			ColliFace(rays.back()->data, tree);
			rays.back()->isScat = hasScat;
			solid += rays.back()->data.fsm.GetSolidAngle();
			while (totalTask > maxThread) Sleep(0);
			mu_thread.lock();
			totalTask++;
			string s;
			s = "Running: " + to_string(int(90 + elev)) + " / " + to_string(int(180-2*delta))
				+ ", number of threads: " + to_string(totalTask);
			for (int i = 0; i < s.size(); i++)
				printf("\b");
			//cout << s << flush;
			printf("%s", s.c_str());
			mu_thread.unlock();
			tasks.push_back(thread(RayTracingParallel<FsmNode>, rays.back(), tree, ref, &totalTask));
			tasks.back().detach();
			//RayTracing(rays.back(), tree, ref);
		}
		while (totalTask > 0) Sleep(1);
	}
	cout << "\nReal number of rays: " << numRay << endl;
	cout << "Total Solid Angle: " << solid << endl;
}

void Tracing::RefRay(BiNode<FsmNode>* pr, bool divide)
{
	if (!pr->data.colli)return;
	Vector4f d0, d1, n, vertex;
	vector<Vector4f> verts;
	float dist;
	n = pr->data.fsm.GetRefPlane();
	vertex = pr->data.fsm.GetVertex();

	for (auto r : pr->data.fsm.GetCorner())
		verts.push_back(r.GetStartPt() + r.GetDirect()*(r.GetEnd() - 1e-5));
	vertex = vertex - n * 2 * (Vector4f::Dot3f(vertex, n) + n.w);
	pr->right = new BiNode<FsmNode>(FsmNode(Frustum(vertex, verts), pr->data.TravelDist()));
	pr->right->isScat = pr->isScat;

	/*if (divide)
	{
		float elev = acos(1 - 2 * (float)rand() / RAND_MAX) / 2;
		float azim = PI * 2 * (float)rand() / RAND_MAX;
		Vector4f z = Vector4f(0.f, 0.f, 1.f);
		float theta = acos(Vector4f::Dot3f(z, n));
		Vector4f c = Vector4f::Cross3f(z, n);
		if (c.GetLenght() < EPS)
			c = Vector4f(0.f, 1.f, 0.f);
		else
			c.SetLenght(1.f);
		Matrix4x4f w = Matrix4x4f();
		w.r0 = Vector4f(0.f, -c[2], c[1]);
		w.r1 = Vector4f(c[2], 0.f, -c[0]);
		w.r2 = Vector4f(-c[1], c[0], 0.f);
		Matrix4x4f eye = Matrix4x4f();
		eye.LoadIdentity();
		Matrix4x4f R = eye + w*sin(theta) + w * w * (1 - cos(theta));
		d1 = R * Vector4f(sin(elev)*cos(azim), sin(elev)*sin(azim), cos(elev));
		d1.SetLenght(1.0f);
		pr->left = new BiNode<FsmNode>(Ray(vertex, d1, dist + pr->data.GetEnd()));
	}*/
}

void Tracing::ColliRay(FsmNode * ray, OBBTree * tree)
{
	OBBIntersection::CollisionTest(ray, tree);
	ColliFace(*ray, tree);
}

void Tracing::PassReceiver(BiNode<FsmNode>* ray, Orient& rec, const vector<double>& sDrct, vector<vector<double>>& hrir, vector<int>& refs, vector<int>& mirs, vector<int>& scats, int band)
{
	int id, n = hrir.size();
	double len;
	Vector4f vec;
	vector<double> bRef, hrir_s;
	vector<COMPLEX> cRef;
	bool colli = true;
	for (auto norm : ray->data.fsm.GetNorm())
		if (Vector4f::Dot3f(norm, rec.GetPos()) + norm.w <= 0)
		{
			colli = false;
			break;
		}
	//if (!colli)return;
	Vector4f farPl = ray->data.fsm.GetRefPlane();
	if (Vector4f::Dot3f(farPl, rec.GetPos()) + farPl.w <= 0)colli = false;
	vec = ray->data.fsm.GetVertex() - rec.GetPos();
	len = vec.GetLenght();
	id = (int)round(len * FS / SOUND_SPEED);
	Vector4f norm = ray->data.fsm.GetNorm().back();
	if (id >= n || Vector4f::Dot3f(norm, rec.GetPos()) + norm.w <= 0)return;
	double len_s = ray->data.fsm.GetCorner().front().GetBegin() + (ray->data.fsm.GetCorner().back().GetBeginPt() - rec.GetPos()).GetLenght();
	vector<vector<double>> hr = HRIR::EvalAmp(rec.LocalPolar(ray->data.fsm.GetVertex() - rec.GetPos()));
	vector<vector<double>> sDHrir = WallAirAbsorb::ConvHrir(sDrct, hr);
	int n_r = 0;
	for (auto r : refs)
		n_r += r;
	mu_egy.lock();
	egyCoef = ray->data.fsm.GetSolidAngle() / pi / 4;
	if (n_r == 1)scatCount++;
	if (n_r == 1)solid += ray->data.fsm.GetSolidAngle();
	if (n_r)
		AddImpulseResponse(hrir, sDHrir, refs, vector<int>(refs.size(), 0), scats, band, vec, id, len_s, true);
	mu_egy.unlock();
	if (colli)
		AddImpulseResponse(hrir, sDHrir, refs, mirs, scats, band, vec, id, len);
}


void Tracing::TracingInRoom(vector<BiNode<RayNode>*>& rays, OBBTree * tree, int ref, Vector4f s)
{
	int numRay = 0;
	//float delta = 1;

	for (float elev = -90 + 2 * delta; 90 - elev - 2 * delta >= 0; elev += 2 * delta)
	{
		float theta[] = { elev - delta, elev + delta };
		int totalTask = 0;
		vector<thread> tasks;
		for (float azim = 0; 360 - azim - delta > EPS; azim += 2 * delta)
		{
			numRay++;
			float phi[] = { azim - delta, azim + delta };
			Vector4f drct = Vector4f(cos(elev*pi / 180)*cos(azim*pi / 180), cos(elev*pi / 180)*sin(azim*pi / 180), sin(elev*pi / 180));
			vector<Vector4f> verts;
			for (int i = 0; i < 2; i++)
				for (int j = 0; j < 2; j++)
					verts.push_back(Vector4f(s.x + cos(theta[i] * pi / 180)*cos(phi[j^i] * pi / 180) * 0.1,
						s.y + cos(theta[i] * pi / 180)*sin(phi[j^i] * pi / 180) * 0.1, s.z + sin(theta[i] * pi / 180) * 0.1));
			Frustum fsm = Frustum(s, verts);
			Ray ray = Ray(s, drct);
			OBBIntersection::CollisionTest(&ray, tree);
			rays.push_back(new BiNode<RayNode>(RayNode(fsm, ray)));
			rays.back()->data.CutFsm();
			rays.back()->isScat = hasScat;
			solid += rays.back()->data.fsm.GetSolidAngle();
			while (totalTask > maxThread) Sleep(0);
			mu_thread.lock();
			totalTask++;
			string s;
			s = "Running: " + to_string(int(90 + elev)) + " / " + to_string(int(180 - 2 * delta))
				+ ", number of threads: " + to_string(totalTask);
			for (int i = 0; i < s.size(); i++)
				printf("\b");
			//cout << s << flush;
			printf("%s", s.c_str());
			mu_thread.unlock();
			tasks.push_back(thread(RayTracingParallel<RayNode>, rays.back(), tree, ref, &totalTask));
			tasks.back().detach();
			//RayTracing(rays.back(), tree, ref);
		}
		while (totalTask > 0) Sleep(1);
	}
	cout << "\nReal number of rays: " << numRay << endl;
	cout << "Total Solid Angle: " << solid << endl;
}

void Tracing::RefRay(BiNode<RayNode>* pr, bool divide)
{
	if (!pr->data.ray.IsIntersect())return;
	Vector4f d0, d1, dr, n, vertex, start;
	vector<Vector4f> verts;
	float dist, dnorm;
	dist = pr->data.GetDist();
	d0 = pr->data.GetDirect();
	n = pr->data.GetFace()->faceNorm;
	vertex = pr->data.fsm.GetVertex();
	start = pr->data.ray.GetStartPt() + pr->data.GetDirect()*(pr->data.ray.GetEnd() - EPS);

	float proj = Vector4f::Dot3f(d0, n);
	d1 = d0 - n*proj * 2;
	d1.SetLenght(1.0f);
	vertex = vertex - n * 2 * (Vector4f::Dot3f(vertex, n) + n.w);
	dnorm = (start - vertex).GetLenght();

	for (auto r : pr->data.fsm.GetCorner())
	{
		dr = r.GetDirect();
		dr = dr - n * Vector4f::Dot3f(dr, n) * 2;
		verts.push_back(vertex + dr * dnorm / Vector4f::Dot3f(d1, dr));
	}
	pr->right = new BiNode<RayNode>(RayNode(Frustum(vertex, verts), start, d1, n, dist + pr->data.ray.GetEnd()));
	pr->right->isScat = pr->isScat;

	/*if (divide)
	{
	float elev = acos(1 - 2 * (float)rand() / RAND_MAX) / 2;
	float azim = PI * 2 * (float)rand() / RAND_MAX;
	Vector4f z = Vector4f(0.f, 0.f, 1.f);
	float theta = acos(Vector4f::Dot3f(z, n));
	Vector4f c = Vector4f::Cross3f(z, n);
	if (c.GetLenght() < EPS)
	c = Vector4f(0.f, 1.f, 0.f);
	else
	c.SetLenght(1.f);
	Matrix4x4f w = Matrix4x4f();
	w.r0 = Vector4f(0.f, -c[2], c[1]);
	w.r1 = Vector4f(c[2], 0.f, -c[0]);
	w.r2 = Vector4f(-c[1], c[0], 0.f);
	Matrix4x4f eye = Matrix4x4f();
	eye.LoadIdentity();
	Matrix4x4f R = eye + w*sin(theta) + w * w * (1 - cos(theta));
	d1 = R * Vector4f(sin(elev)*cos(azim), sin(elev)*sin(azim), cos(elev));
	d1.SetLenght(1.0f);
	pr->left = new BiNode<RayNode>(Ray(vertex, d1, dist + pr->data.GetEnd()));
	}*/
}

void Tracing::ColliRay(RayNode * ray, OBBTree * tree)
{
	OBBIntersection::CollisionTest(&ray->ray, tree);
	ray->CutFsm();
}

void Tracing::PassReceiver(BiNode<RayNode>* ray, Orient& rec, const vector<double>& sDrct, vector<vector<double>>& hrir, vector<int>& refs, vector<int>& mirs, vector<int>& scats, int band)
{
	int id, n = hrir.size();
	double len, len_s;
	Vector4f vec;
	bool colli = true;
	for (auto norm : ray->data.fsm.GetNorm())
		if (Vector4f::Dot3f(norm, rec.GetPos()) + norm.w <= 0)
		{
			colli = false;
			break;
		}
	//if (!colli)return;
	Vector4f farPl = ray->data.fsm.GetRefPlane();
	if (Vector4f::Dot3f(farPl, rec.GetPos()) + farPl.w <= 0)colli = false;
	vec = ray->data.fsm.GetVertex() - rec.GetPos();
	Ray r = ray->data.ray;
	len = r.GetDist() + (r.GetStartPt() - rec.GetPos()).GetLenght(); //vec.GetLenght();
	id = (int)round(len * FS / SOUND_SPEED);
	if (id >= n+256 || Vector4f::Dot3f(rec.GetPos() - r.GetStartPt(), r.GetRef()) + r.GetRef().w < 0)return;
	len_s = r.GetDist();
	vector<vector<double>> filter;
	if (hasDrct)
		filter.push_back(sDrct);
	else
		filter = vector<vector<double>>(1, vector<double>(1, 1));
	vector<vector<double>> hr = HRIR::EvalAmp(rec.LocalPolar(r.GetStartPt() - rec.GetPos()));
	vector<vector<double>> sDHrir = WallAirAbsorb::ConvHrir(filter[0], hr);
	//id = id + rand() % 5 - 2;
	int n_r = 0;
	for (auto r : refs)
		n_r += r;
	if (hasScat)
	{
		double agl = ray->data.fsm.GetSolidAngle(), rd = len - len_s;
		assert(agl > 0);
		agl = agl*Vector4f::Dot3f(r.GetRef(), vec) / vec.GetLenght();
		mu_egy.lock();
		egyCoef = 1 / rd * sqrt(len * abs(agl / pi));
		//cout << "\nmod: " << egyCoef << " ori: " << ray->data.fsm.GetSolidAngle()*len/4/pi*rd << endl;
		if (n_r == 1)scatCount++;
		if (n_r == 1)solid += ray->data.fsm.GetSolidAngle();
		if (n_r)
			AddImpulseResponse(hrir, sDHrir, refs, vector<int>(refs.size(), 0), scats, band, vec, id, len, true);
		mu_egy.unlock();
	}
	else
		mirs = vector<int>(refs.size(), 0);
	if (choose && id - (pos_s - rec.GetPos()).GetLenght()*FS / SOUND_SPEED < 100)
		id = (int)round((pos_s - rec.GetPos()).GetLenght()*FS / SOUND_SPEED);
	if (colli)
		AddImpulseResponse(hrir, sDHrir, refs, mirs, vector<int>(refs.size(), 0), band, vec, id, len);
}


void Tracing::RayTracing(vector<Ray>& ray, OBBTree* tree, int ref)
{
	while (ray.back().IsIntersect() && ray.size()<=ref)
	{
		Ray r = Tracing::RefRay(ray.back(), *ray.back().GetFace());
		OBBIntersection::CollisionTest(&r, tree);
		ray.push_back(r);
	}
}

void Tracing::RayTracing(vector<Ray>& ray, OBBTree* tree, float lim)
{
	float len = ray[0].GetEnd();
	while (ray.back().IsIntersect() && len<lim)
	{
		Ray r = Tracing::RefRay(ray.back(), *ray.back().GetFace());
		OBBIntersection::CollisionTest(&r, tree);
		len += r.GetEnd();
		ray.push_back(r);
	}
}

Ray Tracing::RefRay(Ray & r, faceInfo & f)
{
	Vector4f d0, d1, n;
	d0 = r.GetDirect();
	//d0.SetLenght(1.0f);
	n = f.faceNorm;
	//n.SetLenght(1.0f);
	/*float scatter = Mtllib::GetScatter(f.m_id);
	if (scatter > ((float)rand()/RAND_MAX)) {*/
		float elev = acos(1 - 2 * (float)rand() / RAND_MAX) / 2;
		float azim = PI * 2 * (float)rand() / RAND_MAX;
		Vector4f z = Vector4f(0.f, 0.f, 1.f);
		float theta = acos(Vector4f::Dot3f(z, n));
		Vector4f c = Vector4f::Cross3f(z, n);
		if (c.GetLenght() < EPS)
			c = Vector4f(0.f, 1.f, 0.f);
		else
			c.SetLenght(1.f);
		Matrix4x4f w = Matrix4x4f();
		w.r0 = Vector4f(0.f, -c[2], c[1]);
		w.r1 = Vector4f(c[2], 0.f, -c[0]);
		w.r2 = Vector4f(-c[1], c[0], 0.f);
		Matrix4x4f eye = Matrix4x4f();
		eye.LoadIdentity();
		Matrix4x4f R = eye + w*sin(theta) + w * w * (1 - cos(theta));
		d1 = R * Vector4f(sin(elev)*cos(azim), sin(elev)*sin(azim), cos(elev));
		//float variance = Vector4f::Dot3f(d1, n) - Vector4f::Dot3f(z, Vector4f(sin(elev)*cos(azim), sin(elev)*sin(azim), cos(elev)));
	/*}
	else {*/
		float proj = Vector4f::Dot3f(d0, n);
		d1 = d0 - n*proj * 2;
	//}
	d1.SetLenght(1.0f);
	return Ray(r.GetStartPt() + r.GetDirect()*(r.GetEnd() - 1e-5), d1);
}

void Tracing::ColliReceiver(vector<vector<Ray>>& rays, Vector4f rec, Vector4f front, Vector4f up, vector<vector<double>>& hrir, ofstream& fout)
{
	int id, n = hrir.size();
	double chordLen, proj, Amp, length, angleLim = 0.05;//0.05 means a diameter of 5 cm at 1 meter distance
	Vector4f vec;
	vector<vector<double>> hrir_s;
	for (int i = 0; i < rays.size(); i++)
	{
		Amp = 1.f;
		length = 0.f;
		for (int j = 0; j < rays[i].size(); j++)
		{
			vec = rays[i][j].GetStartPt() - rec;
			proj = -Vector4f::Dot3f(vec, rays[i][j].GetDirect());
			chordLen = Vector4f::Cross3f(vec, rays[i][j].GetDirect()).GetLenght();
			HRIR::JudgeDirection(vec, front, up, hrir_s);
			int len = length + vec.GetLenght();
			id = (int)round(len * FS / SOUND_SPEED);
			if (id >= n)continue;
			if (chordLen / vec.GetLenght() <= angleLim && proj > 0 && proj <= rays[i][j].GetEnd())
			{
				for (int ihs = 0; ihs < hrir_s.size(); ihs++)
				{
					if (id + ihs >= n)break;
					hrir[id + ihs][0] += double(hrir_s[ihs][0]) * Amp / len;
					hrir[id + ihs][1] += double(hrir_s[ihs][1]) * Amp / len;
				}
			}
			Amp *= 0.8;
			length += rays[i][j].GetEnd();
		}
	}
}

FsmNode & Tracing::ColliFace(FsmNode & fNode, OBBTree* tree)
{
	if (!fNode.fsm.GetObbNum())
		return fNode;
	Frustum & fsm = fNode.fsm;
	Vector4f vert = fsm.GetVertex(), drct;
	vector<Vector4f> vertsCheck, vertsFace;
	vector<OBB*> obbs;
	vector<faceInfo*> & faces = fNode.faces;
	vector<int> cornerFaces;
	vector<vector<bool>> vertsSides;
	bool colli = false;
	for (auto& ray : fsm.GetCorner())
	{
		/*if (Vector4f::Dot3f(ray.GetDirect(), Vector4f(0, -1, 0)) > 0.8)
		{
			ray.GetDirect();
		}*/
		OBBIntersection::CollisionTest(&ray, tree);
		if (ray.GetFace())cornerFaces.push_back(ray.GetFace()->id);
	}
	obbs = fsm.GetObbs();
	for (auto obb : obbs)
	{
		for (auto tmpFace : obb->GetFaces())
		{
			colli = false;
			drct = Vector4f(tmpFace->center) - vert;
			if (Vector4f::Dot3f(drct, tmpFace->faceNorm) >= 0)continue;
			for (int fId : cornerFaces)
			{
				if (fId == tmpFace->id)
				{
					faces.push_back(tmpFace);
					colli = true; 
					fNode.colli = true;
					break;
				}
			}
			if (colli)continue;
			vertsCheck = tmpFace->verts;
			vertsSides.clear();
			JudgeVertsSides(fsm, vertsCheck, vertsSides);
			vertsFace = vertsCheck;
			for (auto vS : vertsSides)
				if (vS[0] && vS[1] && vS[2])continue;
			vertsCheck = fsm.GetVerts();
			if (TestVertsTwoSides(vertsFace, vert, vertsCheck, vertsSides))continue;
			if (TestShading(vert, vertsFace, tree))continue;
			faces.push_back(tmpFace);
			fNode.colli = true;//
		}
	}
	//if(FaceUnionAndCull(fNode, tree))fNode.colli = true;
	CalcuRefPlane(fNode);
	return fNode;
}

void Tracing::JudgeVertsSides(Frustum & fsm, vector<Vector4f>& verts, vector<vector<bool>>& vertsSides)
{
	for (auto norm : fsm.GetNorm())
	{
		vertsSides.push_back(vector<bool>());
		for (auto vert : verts)
			vertsSides.back().push_back(Vector4f::Dot3f(norm, vert) + norm.w < 0);
	}
}

bool Tracing::TestVertsOneSide(Vector4f plane, vector<Vector4f> verts)
{
	for (auto v : verts)
		if (Vector4f::Dot3f(plane, v) + plane.w > 0)return false;
	return true;
}

bool Tracing::TestVertsTwoSides(vector<Vector4f> vertsFace, Vector4f vert, vector<Vector4f> vertsCheck, vector<vector<bool>>& vertsSides)
{
	Vector4f v1 = vertsFace[2], v2 = vertsFace[0], v3 = vertsFace[1];
	for (int i = 0; i < 3; i++)
	{
		Vector4f d1 = v1 - v3, d2 = v2 - v1, d3 = Vector4f::Cross3f(d1, d2);
		d1.SetLenght(1.0f); d2.SetLenght(1.0f); d3; d3.SetLenght(1.0f);
		if (TestVertsOneSide(CrossPlane(v1 - vert, d2, v1, v3), vertsCheck))
			return true;
		int iL = vertsSides.size() - 1, iC = vertsCheck.size();
		for (int j = 0; j < vertsCheck.size(); j++)
			if ((vertsSides[j][0] || vertsSides[iL][0]) && (vertsSides[j][1] || vertsSides[iL][1]) &&
				(vertsSides[j][2] || vertsSides[iL][2]) && TestVertsOneSide(CrossPlane(d1, d2, v1, vert), vertsCheck) &&
				TestVertsOneSide(CrossPlane(vertsCheck[j] - vertsCheck[(j + iC - 1) % iC], d2, v1, v3), vertsCheck))
				return true;
		if (TestVertsOneSide(CrossPlane(d1, d2, v1, v1 + d3), vertsCheck))return true;
		v3 = v1; v1 = v2; v2 = vertsFace[(i+1)%3];
	}
	return false;
}

bool Tracing::TestShading(Vector4f vert, vector<Vector4f> vertsFace, OBBTree * tree)
{
	for (auto v : vertsFace)
	{
		Vector4f drct = v - vert;
		float len = drct.GetLenght();
		drct.SetLenght(1.0f);
		Ray ray = Ray(vert, drct);
		OBBIntersection::CollisionTest(&ray, tree);
		if (abs(len - ray.GetEnd() < 1e-5))return false;
	}
	return true;
}

bool Tracing::FaceUnionAndCull(FsmNode & fNode, OBBTree * tree)
{
	if (fNode.faces.empty())return false;
	vector<vector<Edge>>& edges = tree->edges;
	unordered_map<int, pair<int, faceInfo*>> faceLink;
	unordered_map<int, float> faceI2A;
	unordered_map<int, vector<int>> faceChildren;
	set<pair<float, int>, greater<pair<float, int>>> facesArea;
	for (auto f : fNode.faces)
	{
		faceLink[f->id] = pair<int, faceInfo*>(f->id, f);
		facesArea.insert(pair<float, int>(f->area, f->id));
		faceI2A[f->id] =  f->area;
		faceChildren[f->id].push_back(f->id);
	}
	for (auto facePair : faceLink)
	{
		vector<int>& vertsId = facePair.second.second->vertsId;
		int v1 = vertsId.back(), pairId, p1 = FindParent(faceLink, facePair.first), p2;
		for (auto v2 : vertsId)
		{
			if (FindFacePair(edges, v1, v2, p1, pairId))
				if (faceLink.find(pairId) != faceLink.end()
					&& p1 != FindParent(faceLink, pairId))
				{
					p2 = faceLink[pairId].first;
					if (faceI2A[p1] < faceI2A[p2])swap(p1, p2);
					float area = faceI2A[p2];
					facesArea.erase(pair<float, int>(area, p2));
					area += faceI2A[p1];
					facesArea.erase(pair<float, int>(faceI2A[p1], p1));
					facesArea.insert(pair<float, int>(area, p1));
					faceLink[p2].first = p1;
					faceI2A[p1] = area;
					faceI2A[p2] = 0;
					for (auto fId : faceChildren[p2])
						faceChildren[p1].push_back(fId);
					faceChildren[p2].clear();
				}
			v1 = v2;
		}
	}
	fNode.faces.clear();
	int largest = facesArea.begin()->second;
	for (auto fId : faceChildren[largest])
		fNode.faces.push_back(faceLink[fId].second);
	CalcuRefPlane(fNode);
	return true;
}

bool Tracing::FindFacePair(vector<vector<Edge>>& edges, int v1, int v2, int fId, int & pairId)
{
	if (v1 > v2)swap(v1, v2);
	vector<Edge> edge = edges[v1];
	for (auto e : edge)
		if (v2 == e.vertId)
			for (int i = 0; i < 2; i++)
				if (e.faceId[i] >= 0 && e.faceId[i] != fId)
				{
					pairId = e.faceId[i];
					return true;
				}
	pairId = -1;
	return false;
}

int Tracing::FindParent(unordered_map<int, pair<int, faceInfo*>>& fLink, int fId)
{
	if (fLink[fId].first != fId)
		fLink[fId].first = FindParent(fLink, fLink[fId].first);
	return fLink[fId].first;
}

void Tracing::CalcuRefPlane(FsmNode & fNode)
{
	Vector4f drct(0), dtemp(0), v1, v2;
	vector<Vector4f> verts = fNode.GetRayCollides();
	float areaSum = 0;
	if (verts.size() <= 2)
	{
		fNode.colli = false;
		return;
	}
	v1 = verts[2] - verts[0];
	v2 = verts.back() - verts[1];
	dtemp = Vector4f::Cross3f(v1, v2);
	dtemp.SetLenght(1.0);

	for (auto face : fNode.faces)
	{
		drct += face->faceNorm*face->area;
		areaSum += face->area;
	}
	drct.SetLenght(1.0);
	if (Vector4f::Dot3f(drct, dtemp) < 0)
		dtemp = Vector4f(0) - dtemp;
	double lambda = 0.4;
	drct = drct*lambda + dtemp*(1-lambda);
	drct.SetLenght(1.0);
	drct.w = 0;
	
	//for (auto face : fNode.faces)
	//	//for (auto v : face->verts)
	//	drct.w -= Vector4f::Dot3f(drct, face->center)*face->area / areaSum;
	for (auto v : fNode.GetRayCollides())
		//for (auto v : face->verts)
		drct.w -= Vector4f::Dot3f(drct, v);
	drct.w /= fNode.GetRayCollides().size();
	fNode.fsm.cutWithPlane(drct);
}

Vector4f Tracing::CrossPlane(Vector4f d1, Vector4f d2, Vector4f vp, Vector4f vt)
{
	Vector4f plane = Vector4f::Cross3f(d1, d2);
	plane.w = -Vector4f::Dot3f(plane, vp);
	if (Vector4f::Dot3f(plane, vt) + plane.w < 0)
		plane.Set(-plane.x, -plane.y, -plane.z, -plane.w);
	return plane;
}

void Tracing::ReflectFrustum(queue<FsmNode>& que, vector<FsmNode>& vec)
{
	for (auto fNode : vec)
	{
		if (!fNode.colli)continue;
		vector<Ray> rays = fNode.fsm.GetCorner();
		vector<Vector4f> verts;
		for (auto r : rays)
			verts.push_back(r.GetStartPt() + r.GetDirect()*(r.GetEnd()-1e-5));
		Vector4f v = fNode.fsm.GetVertex(), n = fNode.fsm.GetRefPlane(), vertex;
		vertex = v - n * 2 * (Vector4f::Dot3f(v, n) + n.w);
		que.push(FsmNode(Frustum(vertex, verts)));
	}
}

void Tracing::FrustumTracing(queue<FsmNode>& queFNode, vector<FsmNode>& vecFNode, OBBTree * tree, int ref)
{
	if (!queFNode.empty())
	{
		FsmNode fNode = queFNode.front();
		queFNode.pop();
		Frustum & fsm = fNode.fsm;
		OBBIntersection::CollisionTest(&fsm, tree);
		if (fsm.Dividable())
		{
			vector<Frustum> tmp = fsm.DivideFrustum();
			for (auto frustrm : tmp)
			{
				queFNode.push(FsmNode(frustrm));
				FrustumTracing(queFNode, vecFNode, tree, ref);
			}
		}
		else
			vecFNode.push_back(ColliFace(fNode, tree));
	}
}

void Tracing::TracingInRoom(vector<vector<FsmNode>>& vecFsms, OBBTree * tree, int ref, Vector4f s)
{
	queue<FsmNode> queFN;
	vecFsms.push_back(vector<FsmNode>());
	float cube[][3] = { {1e-3,1e-3,1e-3 },{-1e-3,1e-3,1e-3 },{-1e-3,-1e-3,1e-3 },{1e-3,-1e-3,1e-3 } };
	for (int sig = -1; sig <= 1; sig += 2)
	{
		for (int i = 0; i < 3; i++)
		{
			int ind[] = { 0,1,2,0,1 };
			int x = ind[i], y = ind[i + 1], z = ind[i + 2];
			vector<Vector4f> verts;
			for (int j = 0; j < 4; j++)
			{
				verts.push_back(Vector4f(s.x + cube[j][x] * sig, 
					s.y + cube[j][y] * sig, s.z + cube[j][z] * sig));
			}
			queFN.push(FsmNode(Frustum(s, verts)));
			//int tmpref = ref;
			//while (--tmpref>=0)
			//{
				//vecFsms.push_back(vector<FsmNode>());
				FrustumTracing(queFN, vecFsms.back(), tree, ref);
				//if (tmpref > 0)ReflectFrustum(queFN, vecFsms.back());
			//}
		}
	}
	while (--ref >= 0)
	{
		vector<FsmNode> vecFsm, tmpback = vecFsms.back();
		vecFsms.push_back(vecFsm);
		for (auto fNode : tmpback)
		{
			vecFsm.push_back(fNode);
			ReflectFrustum(queFN, vecFsm);
			FrustumTracing(queFN, vecFsms.back(), tree, ref);
			vecFsm.clear();
		}
	}
}

void Tracing::ColliReceiver(vector<vector<FsmNode>>& fNodes, Vector4f rec, Vector4f front, Vector4f up, vector<vector<double>>& hrir, ofstream& fout)
{
	int id, n = hrir.size();
	double len, tlim = n / FS;
	Vector4f vec, farPl;
	bool colli = true;
	for (int i = 0; i < fNodes.size(); i++)
	{
		for (int j = 0; j < fNodes[i].size(); j++)
		{
			colli = true;
			for (auto norm : fNodes[i][j].fsm.GetNorm())
				if (Vector4f::Dot3f(norm, rec) + norm.w <= 0)
				{
					colli = false;
					break;
				}
			if (!colli)continue;
			farPl = fNodes[i][j].fsm.GetRefPlane();
			if (Vector4f::Dot3f(farPl, rec) + farPl.w <= 0)continue;
			vec = fNodes[i][j].fsm.GetVertex() - rec;
			len = vec.GetLenght();
			/*double time = len / SOUND_SPEED, di = i;
			if (time > tlim)continue;
			fout.write((char*)&time, sizeof(double));
			for (int iv = 0; iv < 3; iv++)
			{
				double v = vec[iv];
				fout.write((char*)&v, sizeof(double));
			}
			fout.write((char*)&di, sizeof(double));*/
			//HRIR::JudgeDirection(vec, front, up, hrir_s);
			id = (int)round(len * FS / SOUND_SPEED);
			if (id >= n)continue;
			vector<double> bRef, hrir_s;
			vector<COMPLEX> cRef;
			bRef = WallAirAbsorb::Absorb(len, vector<int>(1,i), vector<int>(1, 0), vector<int>(1,0), -3);
			//cout << i << "\t" << bRef.size() << endl;
			double delay = id - len * FS / SOUND_SPEED;
			vector<double> phaseAdj = WallAirAbsorb::FreqMult(delay * 2 * pi / FS);
			for (int i = 0; i < bRef.size(); i++)
				cRef.push_back(COMPLEX(bRef[i] * cos(phaseAdj[i]), bRef[i] * sin(phaseAdj[i])));
			hrir_s = WallAirAbsorb::InterpIFFT(cRef);
			for (int ihs = 0; ihs < hrir_s.size(); ihs++)
			{
				int ind = id + ihs - hrir_s.size() / 2;
				if (ind >= n)break;
				if (ind < 0)continue;
				//hrir[id + ihs][0] += double(hrir_s[ihs][0])*Amp / len;
				//rir[id + ihs][1] += double(hrir_s[ihs][1])*Amp / len;
				hrir[ind][0] += hrir_s[ihs];
				//hrir[id + ihs][1] += double(hrir_s[ihs][1]);
			}
		}
	}
	fout.flush();
}

