#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
// Needed on MsWindows
#include <windows.h>
#endif // Win32 platform

#include <GL/gl.h>
#include <GL/glu.h>
// Download glut from: http://www.opengl.org/resources/libraries/glut/
#include <GL/glut.h>

#include "float2.h"
#include "float3.h"
#include "Mesh.h"
#include <vector>
#include <map>






class LightSource
{
public:
	virtual float3 getRadianceAt(float3 x) = 0;
	virtual float3 getLightDirAt(float3 x) = 0;
	virtual float  getDistanceFrom(float3 x) = 0;
	virtual void   apply(GLenum openglLightName) = 0;
};

class DirectionalLight : public LightSource
{
	float3 dir;
	float3 radiance;
public:
	DirectionalLight(float3 dir, float3 radiance)
		:dir(dir), radiance(radiance) {}
	float3 getRadianceAt(float3 x) { return radiance; }
	float3 getLightDirAt(float3 x) { return dir; }
	float  getDistanceFrom(float3 x) { return 900000000; }
	void   apply(GLenum openglLightName)
	{
		float aglPos[] = { dir.x, dir.y, dir.z, 0.0f };
		glLightfv(openglLightName, GL_POSITION, aglPos);
		float aglZero[] = { 0.0f, 0.0f, 0.0f, 0.0f };
		glLightfv(openglLightName, GL_AMBIENT, aglZero);
		float aglIntensity[] = { radiance.x, radiance.y, radiance.z, 1.0f };
		glLightfv(openglLightName, GL_DIFFUSE, aglIntensity);
		glLightfv(openglLightName, GL_SPECULAR, aglIntensity);
		glLightf(openglLightName, GL_CONSTANT_ATTENUATION, 1.0f);
		glLightf(openglLightName, GL_LINEAR_ATTENUATION, 0.0f);
		glLightf(openglLightName, GL_QUADRATIC_ATTENUATION, 0.0f);
	}
};

class PointLight : public LightSource
{
	float3 pos;
	float3 power;
public:
	PointLight(float3 pos, float3 power)
		:pos(pos), power(power) {}
	float3 getRadianceAt(float3 x) { return power*(1 / (x - pos).norm2() * 4 * 3.14); }
	float3 getLightDirAt(float3 x) { return (pos - x).normalize(); }
	float  getDistanceFrom(float3 x) { return (pos - x).norm(); }
	void   apply(GLenum openglLightName)
	{
		float aglPos[] = { pos.x, pos.y, pos.z, 1.0f };
		glLightfv(openglLightName, GL_POSITION, aglPos);
		float aglZero[] = { 0.0f, 0.0f, 0.0f, 0.0f };
		glLightfv(openglLightName, GL_AMBIENT, aglZero);
		float aglIntensity[] = { power.x, power.y, power.z, 1.0f };
		glLightfv(openglLightName, GL_DIFFUSE, aglIntensity);
		glLightfv(openglLightName, GL_SPECULAR, aglIntensity);
		glLightf(openglLightName, GL_CONSTANT_ATTENUATION, 0.0f);
		glLightf(openglLightName, GL_LINEAR_ATTENUATION, 0.0f);
		glLightf(openglLightName, GL_QUADRATIC_ATTENUATION, 0.25f / 3.14f);
	}
};

class Material
{
public:
	float3 kd;			// diffuse reflection coefficient
	float3 ks;			// specular reflection coefficient
	float shininess;	// specular exponent
	Material()
	{
		kd = float3(0.5, 0.5, 0.5) + float3::random() * 0.5;
		ks = float3(1, 1, 1);
		shininess = 15;
	}
	virtual void apply()
	{
		glDisable(GL_TEXTURE_2D);
		float aglDiffuse[] = { kd.x, kd.y, kd.z, 1.0f };
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, aglDiffuse);
		float aglSpecular[] = { kd.x, kd.y, kd.z, 1.0f };
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, aglSpecular);
		if (shininess <= 128)
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
		else
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 128.0f);
	}
};

extern "C" unsigned char* stbi_load(char const *filename, int *x, int *y, int *comp, int req_comp);

class TexturedMaterial : public Material
{
unsigned int textureId;
GLint filtering;
public : 
	
	TexturedMaterial(const char* filename,
		GLint filtering = GL_LINEAR_MIPMAP_LINEAR
		) {
		unsigned char* data;
		int width;
		int height;
		int nComponents = 4;
		this->filtering = filtering;

		data = stbi_load(filename, &width, &height, &nComponents, 0);

		if (data == NULL) 
			return;
		glGenTextures(1, &textureId);  // id generation
		glBindTexture(GL_TEXTURE_2D, textureId);      // binding



		if (nComponents == 4)
			gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA, width, height, GL_RGBA, GL_UNSIGNED_BYTE, data);
		else if (nComponents == 3)
			gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, width, height, GL_RGB, GL_UNSIGNED_BYTE, data);


		delete data;
	}

	void apply() {
		Material::apply();

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, textureId);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filtering);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filtering);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	}

};
class Camera
{
	float3 eye;

	float3 ahead;
	float3 lookAt;
	float3 right;
	float3 up;

	float fov;
	float aspect;

	float2 lastMousePos;
	float2 mouseDelta;

public:
	float3 getEye()
	{
		return eye;
	}
	Camera()
	{
		eye = float3(0, 0, -5);
		lookAt = float3(0, 0, 0);
		right = float3(1, 0, 0);
		up = float3(0, 1, 0);

		fov = 1.1;
		aspect = 1;
	}

	void apply()
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(fov / 3.14 * 180, aspect, 0.1, 500);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(eye.x, eye.y, eye.z, lookAt.x, lookAt.y, lookAt.z, 0.0, 1.0, 0.0);
	}

	void setAspectRatio(float ar) { aspect = ar; }

	void move(float dt, std::vector<bool>& keysPressed)
	{
		if (keysPressed.at('w'))
			eye += ahead * dt * 20;
		if (keysPressed.at('s'))
			eye -= ahead * dt * 20;
		if (keysPressed.at('a'))
			eye -= right * dt * 20;
		if (keysPressed.at('d'))
			eye += right * dt * 20;
		if (keysPressed.at('q'))
			eye -= float3(0, 1, 0) * dt * 20;
		if (keysPressed.at('e'))
			eye += float3(0, 1, 0) * dt * 20;

		float yaw = atan2f(ahead.x, ahead.z);
		float pitch = -atan2f(ahead.y, sqrtf(ahead.x * ahead.x + ahead.z * ahead.z));

		yaw -= mouseDelta.x * 0.02f;
		pitch += mouseDelta.y * 0.02f;
		if (pitch > 3.14 / 2) pitch = 3.14 / 2;
		if (pitch < -3.14 / 2) pitch = -3.14 / 2;

		mouseDelta = float2(0, 0);

		ahead = float3(sin(yaw)*cos(pitch), -sin(pitch), cos(yaw)*cos(pitch));
		right = ahead.cross(float3(0, 1, 0)).normalize();
		up = right.cross(ahead);

		lookAt = eye + ahead;
	}

	void startDrag(int x, int y)
	{
		lastMousePos = float2(x, y);
	}
	void drag(int x, int y)
	{
		float2 mousePos(x, y);
		mouseDelta = mousePos - lastMousePos;
		lastMousePos = mousePos;
	}
	void endDrag()
	{
		mouseDelta = float2(0, 0);
	}

};
class Helicam
{
	float3 eye;

	float3 ahead;
	float3 lookAt;
	float3 right;
	float3 up;

	float fov;
	float aspect;

	float2 lastMousePos;
	float2 mouseDelta;

public:
	float3 getEye()
	{
		return eye;
	}
	Helicam()
	{
		eye = float3(0, 100, -100);
		lookAt = float3(0,0,0);
		right = float3(1, 0, 0);
		up = float3(0, 1, 0);

		fov = 1.1;
		aspect = 1;
	}

	void apply(float3 position, float orientationAngle)
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(fov / 3.14 * 180, aspect, 0.1, 500);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		float distance = 50;
		orientationAngle = (orientationAngle / 180) * 3.1415;
		eye.x = distance * cos(orientationAngle) + position.x;
		eye.z = distance * -sin(orientationAngle) + position.z;
		eye.y = 75;
		lookAt.x = distance * -cos(orientationAngle) + position.x;
		lookAt.z = distance * sin(orientationAngle) + position.z;
		lookAt.y = 15;
		gluLookAt(eye.x, eye.y, eye.z, lookAt.x, lookAt.y, lookAt.z, 0.0, 1.0, 0.0);
	}

	void setAspectRatio(float ar) { aspect = ar; }

	void move(float3 position /*std::vector<bool>& keysPressed, float3 velocity*/)
	{
		
		eye.x = position.x;
		eye.y = 100;
		eye.z = position.z - 100;
		/*if (keysPressed.at('w'))
			eye += ahead * dt * 20;
		if (keysPressed.at('s'))
			eye -= ahead * dt * 20;
		if (keysPressed.at('a'))
			eye -= right * dt * 20;
		if (keysPressed.at('d'))
			eye += right * dt * 20;
		if (keysPressed.at('q'))
			eye -= float3(0, 1, 0) * dt * 20;
		if (keysPressed.at('e'))
			eye += float3(0, 1, 0) * dt * 20;

		float yaw = atan2f(ahead.x, ahead.z);
		float pitch = -atan2f(ahead.y, sqrtf(ahead.x * ahead.x + ahead.z * ahead.z));

		yaw -= mouseDelta.x * 0.02f;
		pitch += mouseDelta.y * 0.02f;
		if (pitch > 3.14 / 2) pitch = 3.14 / 2;
		if (pitch < -3.14 / 2) pitch = -3.14 / 2;

		mouseDelta = float2(0, 0);

		ahead = float3(sin(yaw)*cos(pitch), -sin(pitch), cos(yaw)*cos(pitch));
		right = ahead.cross(float3(0, 1, 0)).normalize();
		up = right.cross(ahead);

		lookAt = eye + ahead;*/
	}

	void startDrag(int x, int y)
	{
		lastMousePos = float2(x, y);
	}
	void drag(int x, int y)
	{
		float2 mousePos(x, y);
		mouseDelta = mousePos - lastMousePos;
		lastMousePos = mousePos;
	}
	void endDrag()
	{
		mouseDelta = float2(0, 0);
	}

};

class Object
{
protected:
	Material* material;
	float3 scaleFactor;
	float3 position;
	float3 orientationAxis;
	float bubble_rad;
	float orientationAngle;
	float mass;
	bool collected;
	bool collided = 0;
	
	float3 velocity;
	float3 accel;
	
public:
	int collision_count = 0;
	bool dead = 0;
	int collected_count = 0;
	bool victory = 0;
	Object(Material* material) :material(material), orientationAngle(0.0f), scaleFactor(1.0, 1.0, 1.0), orientationAxis(0.0, 1.0, 0.0), bubble_rad(150.0), collected(0), mass(10), velocity(float3(0,0,0)) {}
	virtual ~Object() {}
	Object* translate(float3 offset) {
		position += offset; return this;
	}
	Object* scale(float3 factor) {
		scaleFactor *= factor; return this;
	}
	Object* rotate(float angle) {
		orientationAngle += angle; return this;
	}

	float3 getPosition() {
		return position;
	}
	float3 getVelocity() {
		return velocity;
	}
	float3 getAccel() {
		return accel;
	}

	virtual void changeVel(float3 newVel)
	{
		velocity = newVel;
	}

	virtual void changePos(float3 newPos)
	{
		position = float3(0, 0, 0);
	}
	virtual void changeAccel(float3 newAccel)
	{
		accel = newAccel;
	}
	float getOrientation()
	{
		return orientationAngle;
	}
	float getBubble_rad() {
		return bubble_rad;
	}

	bool getCollected() {
		return collected;
	}
	bool getCollided() {
		return collided;
	}
	void hasCollided() {
		collided = 1;
	}

	float getMass() {
		return mass;
	}
	virtual void wasCollected() {
		collected = 1;
	}

	virtual bool collection(float3 location, float size)
	{
		float d_srd = pow((position.x - location.x), 2) + pow((position.y - location.y), 2) + pow((position.z - location.z), 2);
		if (d_srd < (bubble_rad + size)) {
			return 1;
		}
		else {
			return 0;
		}
	}

	virtual float3 collision(Object* obj)
	{
		return float3(0, 0, 0);
	}

	virtual void draw()
	{
		material->apply();
		// apply scaling, translation and orientation
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glTranslatef(position.x, position.y, position.z);
		glRotatef(orientationAngle, orientationAxis.x, orientationAxis.y, orientationAxis.z);
		glScalef(scaleFactor.x, scaleFactor.y, scaleFactor.z);
		drawModel();
		glPopMatrix();
	}
	virtual void drawShadow(float3 lightDir) {
		if (!collected)
		{
			float projectToPlane[] = {
				1, 0, 0, 0,
				(-lightDir.x / lightDir.y) , 0, (-lightDir.z / lightDir.y) , 0,
				0, 0, 1, 0,
				0, .1, 0, 1 };
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glMultMatrixf(projectToPlane);
			glTranslatef(position.x, position.y, position.z);
			glRotatef(orientationAngle, orientationAxis.x, orientationAxis.y, orientationAxis.z);
			glScalef(scaleFactor.x, scaleFactor.y, scaleFactor.z);




			drawModel();
			glPopMatrix();
		}
		
	
	};

	virtual void drawModel() = 0;
	virtual void move(double t, double dt) {}
	virtual bool control(std::vector<bool>& keysPressed, std::vector<Object*>& spawn, std::vector<Object*>& objects) { return false; }
};



class Teapot : public Object
{
float angularVelocity;
float angularAcceleration;
float restitution;
public:
	
	Teapot(Material* material) :Object(material) 
	{ 
	mass = 75;
	velocity.x = (rand() % 100 - rand() % 100) * 1.4;
	velocity.z = (rand() % 100 - rand() % 100) * 1.4;
	angularVelocity = rand() % 150;
	}
	void drawModel()
	{
		glutSolidTeapot(1.0f);
	}
	void move(double t, double dt) {

		restitution = .2;
		velocity += accel*dt;
		position += velocity*dt;
		angularVelocity += angularAcceleration*dt;
		float maxSpeed = 20000;
		float horizVel = (velocity.x * velocity.x) + (velocity.z * velocity.z);
		if (horizVel > maxSpeed) {
			velocity.x = (velocity.x / horizVel) * maxSpeed;
			velocity.z = (velocity.z / horizVel) * maxSpeed;
		}
		orientationAngle += angularVelocity*dt;
		if (position.y < 0) {
			position.y = .1;
			velocity.y *= -restitution;
		}
		if (position.x > 190)
		{
			position.x = 188;
			velocity.x = -velocity.x * 1.2;
		}
		if (position.x < -190)
		{
			position.x = -188;
			velocity.x = -velocity.x * 1.2;
		}
		if (position.z > 190)
		{
			position.z = 188;
			velocity.z = -velocity.z * 1.2;
		}
		if (position.z < -190)
		{
			position.z = -188;
			velocity.z = -velocity.z * 1.2;
		}
	}
	float3 collision(Object* obj)
	{
		float3 location = obj->getPosition();
		float size = obj->getBubble_rad();
		float mass2 = obj->getMass();
		
		float d_srd = pow((position.x - location.x), 2) + pow((position.y - location.y), 2) + pow((position.z - location.z), 2);
		if (d_srd < (bubble_rad + size))
		{
			
			////Move first object///
			float3 transfer = float3(velocity.x, 0, velocity.z) * (mass / mass2);
			float2 normal = float2(position.x - location.x, position.z - location.z);
			position.x += normal.x * .08;
			position.z += normal.y * .08;
			float2 vel2d = float2(velocity.x, velocity.z);
			float magV = sqrt(pow(vel2d.x, 2) + pow(vel2d.y, 2));
			if (!magV == 0)
			{
				float magN = sqrt(pow(normal.x, 2) + pow(normal.y, 2));
				float cosine = (vel2d.dot(normal)) / (magV * magN);
				float sine = sqrt(1 - pow(cosine, 2));
				float2 vNormal = vel2d * cosine;
				float2 vPerp = vel2d * sine;
				velocity.x = vNormal.x + vPerp.x;
				velocity.z = vNormal.y + vPerp.y;
			}
			
		

		

			///Move hit object///
			float3 vel = obj->getVelocity();
			float3 newVel = vel + transfer;
			
			obj->changeVel(newVel);
			return float3(0,0,0);

		}
	}

};
class Wallx : public Object
{
public:
	float3 placement;
	Wallx(Material* material, float3 placement) : Object(material), placement(placement) {}
	void drawModel()
	{
		glBegin(GL_QUAD_STRIP);
		glVertex3d(placement.x, placement.y, placement.z);
		glVertex3d(-placement.x, placement.y, placement.z);
		glVertex3d(placement.x, 0, placement.z);
		glVertex3d(-placement.x, 0, placement.z);
		glEnd();
		glBegin(GL_QUAD_STRIP);
		glVertex3d(placement.x, placement.y, -placement.z);
		glVertex3d(-placement.x, placement.y, -placement.z);
		glVertex3d(placement.x, 0, -placement.z);
		glVertex3d(-placement.x, 0, -placement.z);
		glEnd();

	}
	void drawShadow(float3 lightDir) {}
};

class Wallz : public Object
{
public:
	float3 placement;
	Wallz(Material* material, float3 placement) : Object(material), placement(placement) {}
	void drawModel()
	{
		glBegin(GL_QUAD_STRIP);
		glVertex3d(placement.x, placement.y, placement.z);
		glVertex3d(placement.x, placement.y, -placement.z);
		glVertex3d(placement.x, 0, placement.z);
		glVertex3d(placement.x, 0, -placement.z);
		glEnd();
		glBegin(GL_QUAD_STRIP);
		glVertex3d(-placement.x, placement.y, placement.z);
		glVertex3d(-placement.x, placement.y, -placement.z);
		glVertex3d(-placement.x, 0, placement.z);
		glVertex3d(-placement.x, 0, -placement.z);
		glEnd();

	}
	void drawShadow(float3 lightDir) {}
};
class Ground : public Object
{
public:
	Ground(Material* material) : Object(material) {}
	void drawModel()
	{
		glBegin(GL_QUAD_STRIP);
		glVertex3d(1000, 0, 1000);
		glVertex3d(1000, 0, -1000);
		glVertex3d(-1000, 0, 1000);
		glVertex3d(-1000, 0, -1000);
		glEnd();

	}
	void drawShadow(float3 lightDir) {}
};

class MeshInstance : public Object
{
Mesh* mesh;

public:
	MeshInstance(Mesh* mesh, Material* material) : mesh(mesh), Object(material) {}
	void drawModel()
	{
		mesh->draw();
	}
};

class Bouncer : public MeshInstance
{
	float angularVelocity;
	float angularAcceleration;
	float restitution;
	Helicam helicam;
	
public:
	Bouncer(Mesh* mesh, Material* material, Helicam helicam) :
		MeshInstance(mesh, material), helicam(helicam) {
		bubble_rad = 100;
		mass = 100;
	}

	
	void move(double t, double dt) {

		restitution = .2;
		velocity += accel*dt;
		position += velocity*dt;
		angularVelocity += angularAcceleration*dt;
		angularVelocity *= pow(0.1, dt);
		velocity.x *= pow(0.4, dt);
		velocity.z *= pow(0.4, dt);
		float maxSpeed = 20000;
		float horizVel = (velocity.x * velocity.x) + (velocity.z * velocity.z);
		if (horizVel > maxSpeed) {
			velocity.x = (velocity.x / horizVel) * maxSpeed;
			velocity.z = (velocity.z / horizVel) * maxSpeed;
		}
		if (angularVelocity > 250) {
			angularVelocity = 250;
			angularAcceleration = 0;
		}
		else if (angularVelocity < -250) {
			angularVelocity = -250;
			angularAcceleration = 0;
		}
		orientationAngle += angularVelocity*dt;
		if (position.y < 0) {
			position.y = .01;
			velocity.y *= -restitution;
		}
		if (position.x > 195)
		{
			position.x = 190;
			velocity.x = -velocity.x * 1.2;
		}
		if (position.x < -195)
		{
			position.x = -190;
			velocity.x = -velocity.x * 1.2;
		}
		if (position.z > 195)
		{
			position.z = 190;
			velocity.z = -velocity.z * 1.2;
		}
		if (position.z < -195)
		{
			position.z = -190;
			velocity.z = -velocity.z * 1.2;
		}
		helicam.apply(position, orientationAngle);

	}

	float3 collision(Object* obj)
	{
		float3 location = obj->getPosition();
		float size = obj->getBubble_rad();
		float mass2 = obj->getMass();
		
		float d_srd = pow((position.x - location.x), 2) + pow((position.y - location.y), 2) + pow((position.z - location.z), 2);
		if (d_srd < (bubble_rad + size))
		{

			////Move first object///
			collision_count++;
			if (!victory) {
				this->scale(float3(.9, .9, .9));
			}
			
			float3 transfer = float3(velocity.x, 0, velocity.z) * (mass / mass2);
			float2 normal = float2(position.x - location.x, position.z - location.z);
			position.x += normal.x * .08;
			position.z += normal.y * .08;
			float2 vel2d = float2(velocity.x, velocity.z);
			float magV = sqrt(pow(velocity.x, 2) + pow(vel2d.y, 2));
			if (!magV == 0) {
				float magN = sqrt(pow(normal.x, 2) + pow(normal.y, 2));
				float cosine = (vel2d.dot(normal)) / (magV * magN);
				float sine = 1 - pow(cosine, 2);
				float2 vNormal = vel2d * cosine;
				float2 vPerp = vel2d * sine;
				velocity.x = vNormal.x + vPerp.x;
				velocity.z = vNormal.y + vPerp.y;

			}
	

			///Move hit object///
			float3 vel = obj->getVelocity();
			float3 newVel = vel + transfer;
			obj->changeVel((newVel));
			return float3(0,0,0);

		}
	}
	void changeVel(float3 newVel)
	{
		velocity = newVel;
		collision_count++;
	}
	bool control(std::vector<bool>& keysPressed, std::vector<Object*>& spawn, std::vector<Object*>& objects) {
		
		if (keysPressed.at('h')) {
			angularAcceleration += 100;
		}
		else if (keysPressed.at('k')) {
			angularAcceleration += -100;
		}
		
		else {
			angularAcceleration = 0;
		}
		if (keysPressed.at('u')) {
			accel = float3(-cos(orientationAngle / 180 * 3.1415) * 100, 0, sin(orientationAngle / 180 * 3.1415) * 100);
		}
		else if (keysPressed.at('j')) {
			accel = float3(cos(orientationAngle / 180 * 3.1415) * 100, 0, -sin(orientationAngle / 180 * 3.1415) * 100);
		}
		else {
			accel = float3(0, 0, 0);
		}
		return false;
	}
};

class Moveable : public MeshInstance
{
public:
	float angularVelocity;
	float angularAcceleration;
	float restitution;

	Moveable(Mesh* mesh, Material* material) :
		MeshInstance(mesh, material)  {
		bubble_rad = 400;
		accel = float3(0, 0, 0);
		mass = 100;
	}

	void move(double t, double dt) {

		restitution = .2;
		velocity += accel*dt;
		position += velocity*dt;
		angularVelocity += angularAcceleration*dt;
		angularVelocity *= pow(0.1, dt);
		velocity.x *= pow(0.2, dt);
		velocity.z *= pow(0.2, dt);
		float maxSpeed = 2000;
		float horizVel = (velocity.x * velocity.x) + (velocity.z * velocity.z);
		if (horizVel > maxSpeed) {
			velocity.x = (velocity.x / horizVel) * maxSpeed;
			velocity.z = (velocity.z / horizVel) * maxSpeed;
		}
		if (angularVelocity > 250) {
			angularVelocity = 250;
			angularAcceleration = 0;
		}
		else if (angularVelocity < -250) {
			angularVelocity = -250;
			angularAcceleration = 0;
		}
		orientationAngle += angularVelocity*dt;
		if (position.y < 0) {
			position.y = .01;
			velocity.y *= -restitution;
		}
		if (position.x > 195)
		{
			position.x = 190;
			velocity.x = -velocity.x;
		}
		if (position.x < -195)
		{
			position.x = -190;
			velocity.x = -velocity.x;
		}
		if (position.z > 195)
		{
			position.z = 190;
			velocity.z = -velocity.z;
		}
		if (position.z < -195)
		{
			position.z = -190;
			velocity.z = -velocity.z;
		}
	}

	float3 collision(Object* obj)
	{
		float3 location = obj->getPosition();
		float size = obj->getBubble_rad();
		float mass2 = obj->getMass();
		
		float d_srd = pow((position.x - location.x), 2) + pow((position.y - location.y), 2) + pow((position.z - location.z), 2);
		if (d_srd < (bubble_rad + size))
		{
			////Move first object///
			float3 vel = obj->getVelocity();
			float3 transfer = float3(velocity.x, 0, velocity.z) * (mass / mass2);
			float2 normal = float2(position.x - location.x, position.z - location.z);
			float2 vel2d = float2(velocity.x, velocity.z);
			float magV = sqrt(pow(velocity.x, 2) + pow(vel2d.y, 2));
			if (!magV == 0)
			{
				float magN = sqrt(pow(normal.x, 2) + pow(normal.y, 2));
				float cosine = (vel2d.dot(normal)) / (magV * magN);
				float sine = sqrt(1 - pow(cosine, 2));
				float2 vNormal = vel2d * cosine;
				float2 vPerp = vel2d * sine;
				velocity.x = vNormal.x + vPerp.x;
				velocity.z = vNormal.y + vPerp.y;
			}

			
			
			
			///Move hit object///
			float3 newVel = vel + transfer;
			obj->changeVel(newVel);
			return float3(0,0,0);
		}
	}
};




class Scene
{
	Camera camera;
	Helicam helicam;
	std::vector<LightSource*> lightSources;
	std::vector<Object*> objects;
	std::vector<Object*> collidables;
	std::vector<Object*> collectables;
	std::vector<Material*> materials;
	std::vector<Object*> player;
public:
	void initialize()
	{
		//adding a mesh//
	
		// BUILD YOUR SCENE HERE
		lightSources.push_back(
			new DirectionalLight(
				float3(.2, 1, .5),
				float3(1, 0.5, 1)));
		lightSources.push_back(
			new PointLight(
				float3(-1, -1, 1),
				float3(0.2, 0.1, 0.1)));
		Material* greenDiffuseMaterial = new Material();
	
		materials.push_back(greenDiffuseMaterial);
		greenDiffuseMaterial->kd = float3(0, .5, 0);
		materials.push_back(new Material());
		materials.push_back(new Material());
		materials.push_back(new Material());
		materials.push_back(new Material());
		materials.push_back(new Material());
		materials.push_back(new Material());
		TexturedMaterial* tigger_mat = new TexturedMaterial("C:/Users/Elliot/Documents/Visual Studio 2015/Projects/game/game/tigger.png", GL_LINEAR_MIPMAP_LINEAR);
		TexturedMaterial* chevy_mat = new TexturedMaterial("C:/Users/Elliot/Documents/Visual Studio 2015/Projects/game/game/chevy.png", GL_LINEAR_MIPMAP_LINEAR);
		TexturedMaterial* tree_mat = new TexturedMaterial("C:/Users/Elliot/Documents/Visual Studio 2015/Projects/game/game/tree.png", GL_LINEAR_MIPMAP_LINEAR);

		Mesh* tigger = new Mesh("C:/Users/Elliot/Documents/Visual Studio 2015/Projects/game/game/tigger.obj");
		Mesh* chevy = new Mesh("C:/Users/Elliot/Documents/Visual Studio 2015/Projects/game/game/chevy.obj");
		Mesh* tree = new Mesh("C:/Users/Elliot/Documents/Visual Studio 2015/Projects/game/game/tree.obj");
		Bouncer* tigger_mesh = new Bouncer(tigger, tigger_mat, helicam);
	
		objects.push_back(tigger_mesh);
		objects.push_back((new Teapot(materials.at(3)))->translate(float3(100, 10, 100))->scale(float3(10,10,10)));
		objects.push_back((new Teapot(materials.at(2)))->translate(float3(100, 10, 0))->scale(float3(10, 10, 10)));
		objects.push_back((new Teapot(materials.at(1)))->translate(float3(0,10,100))->scale(float3(10, 10, 10)));
		objects.push_back((new Teapot(materials.at(3)))->translate(float3(-100, 10, 100))->scale(float3(10, 10, 10)));
		objects.push_back((new Teapot(materials.at(2)))->translate(float3(-100, 10, 0))->scale(float3(10, 10, 10)));



		objects.push_back((new Teapot(materials.at(3)))->translate(float3(50, 10, 50))->scale(float3(10, 10, 10)));
		objects.push_back((new Teapot(materials.at(2)))->translate(float3(50, 10, 0))->scale(float3(10, 10, 10)));
		objects.push_back((new Teapot(materials.at(1)))->translate(float3(0, 10, -50))->scale(float3(10, 10, 10)));
		objects.push_back((new Teapot(materials.at(3)))->translate(float3(-50, 10, -50))->scale(float3(10, 10, 10)));
		objects.push_back((new Teapot(materials.at(2)))->translate(float3(-50, 10, 0))->scale(float3(10, 10, 10)));
		objects.push_back((new Teapot(materials.at(1)))->translate(float3(-75, 10, 50))->scale(float3(10, 10, 10)));
		objects.push_back((new Teapot(materials.at(3)))->translate(float3(-75, 10, 0))->scale(float3(10, 10, 10)));

		///collectables///
		MeshInstance* car_mesh = new MeshInstance(chevy, chevy_mat);
		car_mesh->translate(float3(75, 5, 75))->scale(float3(.3, .3, .3));
		MeshInstance* car_mesh1 = new MeshInstance(chevy, chevy_mat);
		car_mesh1->translate(float3(-75, 5, -75))->scale(float3(.3, .3, .3));
		MeshInstance* car_mesh2 = new MeshInstance(chevy, chevy_mat);
		car_mesh2->translate(float3(75, 5, -75))->scale(float3(.3, .3, .3));
		MeshInstance* car_mesh3 = new MeshInstance(chevy, chevy_mat);
		car_mesh3->translate(float3(-75, 5, 75))->scale(float3(.3, .3, .3));
		MeshInstance* car_mesh4 = new MeshInstance(chevy, chevy_mat);
		car_mesh4->translate(float3(50, 5, 0))->scale(float3(.3, .3, .3));
		objects.push_back(car_mesh);
		collectables.push_back(car_mesh);
		objects.push_back(car_mesh1);
		collectables.push_back(car_mesh1);
		objects.push_back(car_mesh2);
		collectables.push_back(car_mesh2);
		objects.push_back(car_mesh3);
		collectables.push_back(car_mesh3);
		objects.push_back(car_mesh4);
		collectables.push_back(car_mesh4);
		
		
		//objects.push_back(car_mesh);
		objects.push_back((new Ground(materials.at(0))));
		objects.push_back((new Wallx(materials.at(1), float3(200, 50, 200))));
		objects.push_back((new Wallz(materials.at(1), float3(200, 50, 200))));
		//collectables.push_back(car_mesh);
		player.push_back(tigger_mesh);
		collidables.push_back(tigger_mesh);
		collidables.push_back(objects.at(1));
		collidables.push_back(objects.at(2));
		collidables.push_back(objects.at(3));
		collidables.push_back(objects.at(4));
		collidables.push_back(objects.at(5));
		collidables.push_back(objects.at(6));
		collidables.push_back(objects.at(7));
		collidables.push_back(objects.at(8));
		collidables.push_back(objects.at(9));
		collidables.push_back(objects.at(10));
		collidables.push_back(objects.at(11));
		collidables.push_back(objects.at(12));
		

	}
	void control(std::vector<bool>& keysPressed) {
		std::vector<Object*> spawn;

		for (int i = 0; i < objects.size(); i++) {
			objects.at(i)->control(keysPressed, spawn, objects);
		}
	}

	void collect() {
		for (int i = 0; i < collectables.size(); i++)
		{
			if (!collectables.at(i)->getCollected()) {

				float3 location = collectables.at(i)->getPosition();
				float size = collectables.at(i)->getBubble_rad();
				if (player.at(0)->collection(location, size))
				{
					player.at(0)->collected_count++;
					collectables.at(i)->wasCollected();
					if (player.at(0)->collected_count >= collectables.size())
					{
						player.at(0)->victory = 1;
						printf("You Win!");
					}
				}
			
			}
		}

	}

	void collide() {
		for (int j = 0; j < collidables.size(); j++)
		{
			for (int i = 0; i < collidables.size(); i++)
			{
				if (i != j) {
					float3 location = collidables.at(i)->getPosition();
					float size = collidables.at(i)->getBubble_rad();
					float mass = collidables.at(i)->getMass();
					collidables.at(j)->collision(collidables.at(i));
				
				}
			

			}
		}

	}

	void move(double t, double dt) {
		if (player.at(0)->collision_count > 7)
		{
			player.at(0)->dead = 1;
		}
		if (player.at(0)->victory)
		{
			player.at(0)->dead = 0;
			for (int i = 1; i < collidables.size(); i++)
			{
				//collidables.at(i)->scale(float3(.99, .99, .99));
			}
		}
		if (player.at(0)->dead)
		{	
			player.at(0)->scale(float3(.98, .98, .98));
			
		}
		else {
			for (int i = 0; i < objects.size(); i++) {
				objects.at(i)->move(t, dt);
			}
		}
		
	}
	
	~Scene()
	{
		for (std::vector<LightSource*>::iterator iLightSource = lightSources.begin(); iLightSource != lightSources.end(); ++iLightSource)
			delete *iLightSource;
		for (std::vector<Material*>::iterator iMaterial = materials.begin(); iMaterial != materials.end(); ++iMaterial)
			delete *iMaterial;
		for (std::vector<Object*>::iterator iObject = objects.begin(); iObject != objects.end(); ++iObject)
			delete *iObject;
	}

public:
	Camera& getCamera()
	{
		return camera;
	}
	Helicam& getHelicam()
	{
		return helicam;
	}

	void draw()
	{

		unsigned int iLightSource = 0;
		for (; iLightSource < lightSources.size(); iLightSource++)
		{
			glEnable(GL_LIGHT0 + iLightSource);
			lightSources.at(iLightSource)->apply(GL_LIGHT0 + iLightSource);
		}
		for (; iLightSource < GL_MAX_LIGHTS; iLightSource++)
			glDisable(GL_LIGHT0 + iLightSource);

		for (unsigned int iObject = 0; iObject < objects.size(); iObject++)
		{
			if (!objects.at(iObject)->getCollected()) {
				objects.at(iObject)->draw();
			}
		}
		


		glDisable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);
		glColor3d(0, 0, 0);

		for (int i = 0; i < objects.size(); i++) {
			float3 lightDir = lightSources.at(0)->getLightDirAt(objects.at(i)->getPosition());
			objects.at(i)->drawShadow(lightDir);
		}
		glEnable(GL_LIGHTING);
	}

};

Scene scene;
std::vector<bool> keysPressed;

void onDisplay() {
	glClearColor(0.1f, 0.2f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear screen

	scene.draw();

	glutSwapBuffers(); // drawing finished
}

void onIdle()
{
	double t = glutGet(GLUT_ELAPSED_TIME) * 0.001;        	// time elapsed since starting this program in msec 
	static double lastTime = 0.0;
	double dt = t - lastTime;
	lastTime = t;

	scene.getCamera().move(dt, keysPressed);
	scene.control(keysPressed);
	scene.collect();
	scene.collide();
	scene.move(t, dt);

	glutPostRedisplay();
}

void onKeyboard(unsigned char key, int x, int y)
{
	keysPressed.at(key) = true;
}

void onKeyboardUp(unsigned char key, int x, int y)
{
	keysPressed.at(key) = false;
}

void onMouse(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON)
		if (state == GLUT_DOWN)
			scene.getCamera().startDrag(x, y);
		else
			scene.getCamera().endDrag();
}

void onMouseMotion(int x, int y)
{
	scene.getCamera().drag(x, y);
}

void onReshape(int winWidth, int winHeight)
{
	glViewport(0, 0, winWidth, winHeight);
	scene.getCamera().setAspectRatio((float)winWidth / winHeight);
}

int main(int argc, char **argv) {
	glutInit(&argc, argv);						// initialize GLUT
	glutInitWindowSize(600, 600);				// startup window size 
	glutInitWindowPosition(100, 100);           // where to put window on screen
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);    // 8 bit R,G,B,A + double buffer + depth buffer

	glutCreateWindow("Attack of the Spinning Teapots");				// application window is created and displayed

	glViewport(0, 0, 600, 600);

	glutDisplayFunc(onDisplay);					// register callback
	glutIdleFunc(onIdle);						// register callback
	glutReshapeFunc(onReshape);
	glutKeyboardFunc(onKeyboard);
	glutKeyboardUpFunc(onKeyboardUp);
	glutMouseFunc(onMouse);
	glutMotionFunc(onMouseMotion);

	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);

	scene.initialize();
	for (int i = 0; i<256; i++)
		keysPressed.push_back(false);

	glutMainLoop();								// launch event handling loop

	return 0;
}
