/*
Title: MassSpringLagrangian
File Name: main.cpp
Copyright © 2015
Original authors: Gabriel Ortega
Written under the supervision of David I. Schwartz, Ph.D., and
supported by a professional development seed grant from the B. Thomas
Golisano College of Computing & Information Sciences
(https://www.rit.edu/gccis) at the Rochester Institute of Technology.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Description:
This is a simulation of simple mass spring system using Langrangian dynamics. We have 3 point masses (m0, m1, m2), connected by 2 springs with constants (c0, c1).
We'll treat m0 as if it is fixed, but m1 and m2 are constrained by the springs and under the force of gravity.

=========O========== - m0
|
c0 - |           - L1
|
O  ---------- m1
|
c1 - |           - L2
|
O  ---------- m2

The force due to gravity on mass indexed by i (mI):

GravityI = -mI * g * k where g is the gravitational constant and k is the basis up vector.

Due to Hook's law the spring force at each mass is :

F = (y - yL) * c, where y is displacement, yL is resting displacement, and c is the spring constant;

The kinetic energy for the system is:

T = 1/2 * m * y'^2 for masses i = 1, 2, where y is position and y' is velocity.

NOTE: Technically this is a summation.

The relevant derivatives:

dT/dy = 0

dT/dy' = m * y'

d/dt(dT/dy') = m * y"

The equations of motion are m * y" = F * k, for each mass i

m1 * y1" = (-c1 * (y1 - y2 - L1)) + (c0 * (y0 - y1 - L0)) - (m3 * g)
m2 * y2" = (c1 * (y1 - y2 - L1)) - (m2 * g)

Solve each equation for y" to get generalized acceleration and plug back into position equation!
*/

#include <GL\glew.h>
#include <GLFW\glfw3.h>
#include <glm\common.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <chrono>
#include <vector>

#define PI															3.141592653589793238463f
#define PI_2														3.141592653589793238463f * 2.0f
#define GRAVITY														0.0000098196f
#define SPHERE_STACK_COUNT											10
#define SPHERE_SLICE_COUNT											10

#define M0															0.2f
#define M1															0.2f
#define M2															0.2f

#define IM0															0.0f
#define IM1															5.0f
#define IM2															5.0f

#define C0															0.00002f
#define C1															0.00002f

#define L0															1.5f
#define L1															1.5f

using namespace glm;

typedef std::chrono::high_resolution_clock::time_point				ClockTime;
typedef std::chrono::duration<double, std::milli>					Milliseconds;
typedef std::vector<vec3>											VertexBuffer;
typedef std::vector<vec3>											NormalBuffer;

GLFWwindow*	gWindow;
GLFWmonitor* gMonitor;
const GLFWvidmode* gVideoMode;
bool gShouldExit = 0;

GLuint gSphereVertexBufferID = 0;
GLuint gSphereNormalBufferID = 0;
GLuint gSpringVertexBufferID = 0;
GLuint gTransformBufferID = 0;
GLuint gSphereVertexArrayID = 0;
GLuint gSpringVertexArrayID = 0;
GLuint gLineShaderID = 0;
GLuint gSphereShaderID = 0;
GLuint gLineTransformID = 0;
GLuint gLineColorID = 0;
GLuint gSphereTransformID = 0;
GLuint gSphereColorID = 0;

mat4 gProjection;
mat4 gView;
mat4 gModel;
mat4 gTransformM0;
mat4 gTransformM1;
mat4 gTransformM2;
mat4 gTransformS;

VertexBuffer gSphereVertexBuffer;
VertexBuffer gSphereNormalBuffer;
VertexBuffer gSpringVertexBuffer;

vec3 gEyePosition = vec3(0.0f, 0.0f, 20.f);
vec3 gEyeDirection = vec3(0.0f, 0.0f, -1.0);
vec3 gEyeUp = vec3(0.0f, 1.0f, 0.0f);

vec3 gSpringPosition = vec3(0.0f, 5.0f, 0.0f);

void InitializeSimulation();
void InitializeOpenGL();
void InitializeGeometry();
void InitializeShaders();
void InitializeProjectViewMatrices();
void BeginScene();
void BindGeometryAndShaders();
void UpdateScene(double millisecondsElapsed);
void RenderScene(double millisecondsElapsed);
void HandleInput();

void BuildSphere(VertexBuffer& vertexBuffer, NormalBuffer& normalBuffer, int stackCount, int sliceCount);


// Main Loop
int main(int argc, int* argv[])
{
	// Program Structure
	InitializeSimulation();
	InitializeOpenGL();
	InitializeGeometry();
	InitializeShaders();
	InitializeProjectViewMatrices();
	BeginScene();

	return 0;
}

void InitializeSimulation()
{


}

void InitializeOpenGL()
{
	// Graphics API setup.
	int glfwSuccess = glfwInit();
	if (!glfwSuccess) {
		exit(1);
	}

	// Create Window
	gMonitor = glfwGetPrimaryMonitor();
	gVideoMode = glfwGetVideoMode(gMonitor);

	//GLFWwindow* window = glfwCreateWindow(videoMode->width, videoMode->height, "Sphere", NULL, NULL);
	gWindow = glfwCreateWindow(1200, 900, "Mass Spring Lagrangian 2D", NULL, NULL);

	if (!gWindow) {
		glfwTerminate();
	}

	glfwMakeContextCurrent(gWindow);
	glewInit();

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
}

void InitializeGeometry()
{
	BuildSphere(gSphereVertexBuffer, gSphereNormalBuffer, SPHERE_STACK_COUNT, SPHERE_SLICE_COUNT);

	glGenBuffers(1, &gSphereVertexBufferID);
	glBindBuffer(GL_ARRAY_BUFFER, gSphereVertexBufferID); // OpenGL.GL_Array_Buffer = buffer with ID(vertexBufferID)
	glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * gSphereVertexBuffer.size(), &gSphereVertexBuffer[0], GL_STATIC_DRAW);

	glGenBuffers(1, &gSphereNormalBufferID);
	glBindBuffer(GL_ARRAY_BUFFER, gSphereNormalBufferID); // OpenGL.GL_Array_Buffer = buffer with ID(vertexBufferID)
	glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * gSphereNormalBuffer.size(), &gSphereNormalBuffer[0], GL_STATIC_DRAW);

	vec3 initial = { 0.0f, 0.0f, 0.0f };
	gSpringVertexBuffer.push_back(initial);
	gSpringVertexBuffer.push_back(initial);
	gSpringVertexBuffer.push_back(initial);
	gSpringVertexBuffer.push_back(initial);

	glGenBuffers(1, &gSpringVertexBufferID);
	glBindBuffer(GL_ARRAY_BUFFER, gSpringVertexBufferID);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vec3) * gSpringVertexBuffer.size(), nullptr, GL_DYNAMIC_DRAW);
}

void InitializeShaders()
{
	// Line Shader
	{
		// Extremely simple vertex and fragment shaders
		const char* lineVertexShader =
			"#version 400\n"
			"uniform mat4 transform;"
			"in vec3 vp;"
			"void main () {"
			"  gl_Position = transform * vec4 (vp, 1.0);"
			"}";

		const char* lineFragmentShader =
			"#version 400\n"
			"uniform vec4 color;"
			"out vec4 frag_colour;"
			"void main () {"
			"  frag_colour = color;"
			"}";

		GLuint lineVertexShaderID = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(lineVertexShaderID, 1, &lineVertexShader, NULL);
		glCompileShader(lineVertexShaderID);

		GLuint lineFragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(lineFragmentShaderID, 1, &lineFragmentShader, NULL);
		glCompileShader(lineFragmentShaderID);

		gLineShaderID = glCreateProgram();
		glAttachShader(gLineShaderID, lineVertexShaderID);
		glAttachShader(gLineShaderID, lineFragmentShaderID);
		glLinkProgram(gLineShaderID);

		GLuint attributeID = glGetAttribLocation(gLineShaderID, "vp");

		glGenVertexArrays(1, &gSpringVertexArrayID);
		glBindVertexArray(gSpringVertexArrayID);

		glBindBuffer(GL_ARRAY_BUFFER, gSpringVertexBufferID);
		glVertexAttribPointer(attributeID, 3, GL_FLOAT, GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(attributeID);

		gLineTransformID = glGetUniformLocation(gLineShaderID, "transform");
		gLineColorID = glGetUniformLocation(gLineShaderID, "color");
	}

	// Sphere Shader
	{
		const char* sphereVertexShader =
			"#version 400\n"
			"uniform mat4 transform;"
			"in vec3 vp;"
			"in vec3 vn;"
			"out vec3 pn;"
			"void main () {"
			"  pn = mat3(transform) * vn;"
			"  gl_Position = transform * vec4 (vp, 1.0);"
			"}";

		const char* sphereFragmentShader =
			"#version 400\n"
			"uniform vec4 color;"
			"in vec3 pn;"
			"out vec4 frag_colour;"
			"void main () {"
			"  vec3 lightDir = vec3(1.0f, -1.0f, 0.0f);"
			"  frag_colour = color * dot(normalize(lightDir), normalize(pn));"
			"}";

		GLuint sphereVertexShaderID = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(sphereVertexShaderID, 1, &sphereVertexShader, NULL);
		glCompileShader(sphereVertexShaderID);

		GLuint sphereFragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(sphereFragmentShaderID, 1, &sphereFragmentShader, NULL);
		glCompileShader(sphereFragmentShaderID);

		gSphereShaderID = glCreateProgram();
		glAttachShader(gSphereShaderID, sphereVertexShaderID);
		glAttachShader(gSphereShaderID, sphereFragmentShaderID);
		glLinkProgram(gSphereShaderID);

		glGenVertexArrays(1, &gSphereVertexArrayID);
		glBindVertexArray(gSphereVertexArrayID);

		GLuint vertexID = glGetAttribLocation(gSphereShaderID, "vp");
		glBindBuffer(GL_ARRAY_BUFFER, gSphereVertexBufferID);
		glVertexAttribPointer(vertexID, 3, GL_FLOAT, GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(vertexID);

		GLuint normalID = glGetAttribLocation(gSphereShaderID, "vn");
		glBindBuffer(GL_ARRAY_BUFFER, gSphereNormalBufferID);
		glVertexAttribPointer(normalID, 3, GL_FLOAT, GL_FALSE, 0, NULL);
		glEnableVertexAttribArray(normalID);

		gSphereTransformID = glGetUniformLocation(gSphereShaderID, "transform");
		gSphereColorID = glGetUniformLocation(gSphereShaderID, "color");
	}
}

void InitializeProjectViewMatrices()
{
	// Camera is static so only calculate projection and view matrix once.
	gProjection = perspective(45.0f, 1200.0f / 900.0f, 0.1f, 100.0f);
	gView = lookAt(gEyePosition, gEyePosition + gEyeDirection, gEyeUp);
}

void BeginScene()
{
	// Loop setup. 
	ClockTime currentTime = std::chrono::high_resolution_clock::now();
	while (!gShouldExit)
	{
		ClockTime systemTime = std::chrono::high_resolution_clock::now();
		double deltaTime = Milliseconds(systemTime - currentTime).count() + DBL_EPSILON;
		currentTime = systemTime;

		UpdateScene(deltaTime);
		RenderScene(deltaTime);
		glfwSwapBuffers(gWindow);
	}
}

void UpdateScene(double millisecondsElapsed)
{
	if (millisecondsElapsed > 16.67f)
	{
		millisecondsElapsed = 16.67f;
	}

	float accumulator = static_cast<float>(millisecondsElapsed);
	float deltaTime = static_cast<float>(millisecondsElapsed) / 5.0f;
	float time = 0.0f;

	static float qDot1, qDot2, qDotDot1, qDotDot2;
	static float q0 = 5.0f;
	static float q1 = 0.0f;
	static float q2 = -5.0f;

	while (accumulator >= time)
	{
		qDotDot1 = ((-C1 * (q1 - q2 - L1)) * IM1) + ((C0 * (q0 - q1 - L0)) * IM1) - GRAVITY;
		qDotDot2 = (C1 * (q1 - q2 - L1) * IM2) - GRAVITY;

		qDot1 += qDotDot1 * deltaTime;
		qDot2 += qDotDot2 * deltaTime;

		q1 += qDot1 * deltaTime;
		q2 += qDot2 * deltaTime;

		time += deltaTime;
		accumulator -= deltaTime;
	}

	// Update Graphics Data 
	mat4 identity = mat4(1.0f);
	gTransformM0 = gProjection * gView * translate(identity, vec3(0.0f, q0, 0.0f));
	gTransformM1 = gProjection * gView * translate(identity, vec3(0.0f, q1, 0.0f));
	gTransformM2 = gProjection * gView * translate(identity, vec3(0.0f, q2, 0.0));
	gTransformS = gProjection * gView * identity;

	glBindBuffer(GL_ARRAY_BUFFER, gSpringVertexBufferID);
	float q[] = { q0, q1, q1, q2 };
	vec3* vertices = reinterpret_cast<vec3*>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY));
	for (int i = 0; i < gSpringVertexBuffer.size(); i++)
	{
		vertices[i].y = q[i];
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// Update Title
	char title[100];
	float fps = 1000.0f / static_cast<float>(millisecondsElapsed);
	sprintf_s(title, "Mass Spring Lagrangian FPS: %f", fps);
	glfwSetWindowTitle(gWindow, title);
}

void RenderScene(double millisecondsElapsed)
{
	// Clear buffers. Set shader.
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);


	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glUseProgram(gSphereShaderID);

	glBindVertexArray(gSphereVertexArrayID);

	glUniformMatrix4fv(gSphereTransformID, 1, GL_FALSE, &gTransformM0[0][0]);
	glDrawArrays(GL_TRIANGLES, 0, gSphereVertexBuffer.size());

	glUniformMatrix4fv(gSphereTransformID, 1, GL_FALSE, &gTransformM1[0][0]);
	glDrawArrays(GL_TRIANGLES, 0, gSphereVertexBuffer.size());

	glUniformMatrix4fv(gSphereTransformID, 1, GL_FALSE, &gTransformM2[0][0]);
	glDrawArrays(GL_TRIANGLES, 0, gSphereVertexBuffer.size());

	glUseProgram(gLineShaderID);

	glBindVertexArray(gSpringVertexArrayID);

	glUniformMatrix4fv(gLineTransformID, 1, GL_FALSE, &gTransformS[0][0]);
	glDrawArrays(GL_LINES, 0, gSpringVertexBuffer.size());
}

void BuildSphere(VertexBuffer& vertexBuffer, NormalBuffer& normalBuffer, int stackCount, int sliceCount)
{
	std::vector<float> phiCoordinates;

	float phiStep = PI / stackCount;
	for (int i = 0; i < stackCount + 1; i++)
	{
		phiCoordinates.push_back((-PI_2) + (i * phiStep));
	}

	std::vector<float> thetaCoordinates;
	float thetaStep = (PI_2) / (sliceCount * 2);
	for (int i = 0; i < (sliceCount * 2) + 1; i++)
	{
		thetaCoordinates.push_back(0 + (i * thetaStep));
	}

	float radius = 0.5f;
	for (int i = 0; i < phiCoordinates.size() - 1; i++)
	{
		for (int j = 0; j < thetaCoordinates.size() - 1; j++)
		{
			vec3 vertex1 = vec3(radius * cosf(phiCoordinates[i]) * sinf(thetaCoordinates[j]), radius * sinf(phiCoordinates[i]) * sinf(thetaCoordinates[j]), radius * cosf(thetaCoordinates[j]));
			vec3 vertex2 = vec3(radius * cosf(phiCoordinates[i]) * sinf(thetaCoordinates[j + 1]), radius * sinf(phiCoordinates[i]) * sinf(thetaCoordinates[j + 1]), radius * cosf(thetaCoordinates[j + 1]));
			vec3 vertex3 = vec3(radius * cosf(phiCoordinates[i + 1]) * sinf(thetaCoordinates[j + 1]), radius * sinf(phiCoordinates[i + 1]) * sinf(thetaCoordinates[j + 1]), radius * cosf(thetaCoordinates[j + 1]));
			vec3 vertex4 = vec3(radius * cosf(phiCoordinates[i + 1]) * sinf(thetaCoordinates[j]), radius * sinf(phiCoordinates[i + 1]) * sinf(thetaCoordinates[j]), radius * cosf(thetaCoordinates[j]));

			vec3 vn = normalize(cross(vertex2 - vertex1, vertex3 - vertex1));
			if (thetaCoordinates[j] <= PI)
			{
				vertexBuffer.push_back(vertex1);
				vertexBuffer.push_back(vertex2);
				vertexBuffer.push_back(vertex3);

				vertexBuffer.push_back(vertex1);
				vertexBuffer.push_back(vertex3);
				vertexBuffer.push_back(vertex4);

				normalBuffer.push_back(vn);
				normalBuffer.push_back(vn);
				normalBuffer.push_back(vn);

				normalBuffer.push_back(vn);
				normalBuffer.push_back(vn);
				normalBuffer.push_back(vn);
			}

			if (thetaCoordinates[j] >= PI)
			{
				vertexBuffer.push_back(vertex1);
				vertexBuffer.push_back(vertex3);
				vertexBuffer.push_back(vertex2);

				vertexBuffer.push_back(vertex1);
				vertexBuffer.push_back(vertex4);
				vertexBuffer.push_back(vertex3);

				vn = -vn;
				normalBuffer.push_back(vn);
				normalBuffer.push_back(vn);
				normalBuffer.push_back(vn);

				normalBuffer.push_back(vn);
				normalBuffer.push_back(vn);
				normalBuffer.push_back(vn);
			}
		}
	}
}
