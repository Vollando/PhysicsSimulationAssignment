#pragma once

#include "PhysicsEngine.h"
#include "BasicActors.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	class CompoundShape : public DynamicActor
	{
	public:
		CompoundShape(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.5f, 0.5f, 0.5f), PxReal density = 1.0f)
			: DynamicActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
			CreateShape(PxBoxGeometry(dimensions), density);
		}
	};

	class CapsuleWheel : public DynamicActor
	{
	public:
		CapsuleWheel(const PxTransform& pose = PxTransform(PxIdentity), PxVec2 dimensions = PxVec2(0.5f, 1.0f), PxReal density = 1.0f)
			: DynamicActor(pose)
		{
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
		}
	};

	class Wedge
	{
	public:
		vector<PxVec3> verts = { PxVec3(0.5,1,0), PxVec3(0.5,1,1), PxVec3(1,0,0), PxVec3(0,0,0), PxVec3(0,0,1), PxVec3(1,0,1) };
		ConvexMesh* mesh = new ConvexMesh(vector<PxVec3>(begin(verts), end(verts)));

		Wedge(float h = 1.0f, float w = 1.0f, float l = 1.0f, PxTransform pose = PxTransform(PxIdentity), PxReal density = 1.0f)
		{
			verts[0].y *= h;
			verts[0].x *= w;
			verts[0].z *= l;
			verts[1].y *= h;
			verts[1].x *= w;
			verts[1].z *= l;
			verts[2].x *= w;
			verts[4].z *= l;
			verts[5].x *= w;
			verts[5].z *= l;
			mesh = new ConvexMesh(vector<PxVec3>(begin(verts), end(verts)), pose, density);
		}
	};

	class Dimond
	{
	public:
		vector<PxVec3> verts = { PxVec3(0.5,1,0), PxVec3(0.5,1,1), PxVec3(1,0,0), PxVec3(0,0,0), PxVec3(0,0,1), PxVec3(1,0,1), 
								 PxVec3(0.5,-1,0), PxVec3(0.5,-1,1) };
		ConvexMesh* mesh = new ConvexMesh(vector<PxVec3>(begin(verts), end(verts)));

		Dimond(float h = 1.0f, float w = 1.0f, float l = 1.0f, PxTransform pose = PxTransform(PxIdentity), PxReal density = 1.0f)
		{
			verts[0].y *= h;
			verts[0].x *= w;
			verts[0].z *= l;
			verts[1].y *= h;
			verts[1].x *= w;
			verts[1].z *= l;

			verts[2].x *= w;
			verts[4].z *= l;
			verts[5].x *= w;
			verts[5].z *= l;

			verts[6].y *= h;
			verts[6].x *= w;
			verts[6].z *= l;
			verts[7].y *= h;
			verts[7].x *= w;
			verts[7].z *= l;
			mesh = new ConvexMesh(vector<PxVec3>(begin(verts), end(verts)), pose, density);
		}
	};

	class Walls : public DynamicActor
	{
	public:
		Walls(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.5f, 0.5f, 0.5f), PxReal density = 1.0f)
			: DynamicActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
			CreateShape(PxBoxGeometry(dimensions), density);
			CreateShape(PxBoxGeometry(PxVec3(dimensions.z, dimensions.y, 2.0f+dimensions.x/2)), density);
			CreateShape(PxBoxGeometry(PxVec3(dimensions.z, dimensions.y, 2.0f+dimensions.x/2)), density);
		}
	};

	class Hexagon
	{
	public:
		vector<PxVec3> verts{ PxVec3(0,1,0), PxVec3(0,1,0), PxVec3(-1,0,0), PxVec3(0,0,1), PxVec3(0,0,-1), PxVec3(0,0,-1) };
		ConvexMesh* mesh = new ConvexMesh(vector<PxVec3>(begin(verts), end(verts)));

		Hexagon(float h, float edge, PxTransform pose = PxTransform(PxIdentity), PxReal density = 1.0f)
		{
			float theta = 0.0f;

			for (int i = 0; i < 6; i++)
			{
				theta = i * 2.0f * PxPi / 6;
				verts[i] = PxVec3(edge * cosf(theta), 0.0f, edge * sinf(theta));
			}
			for (int i = 0; i < 6; i++)
			{
				verts.push_back(PxVec3(verts[i].x, verts[i].y + h, verts[i].z));
			}
			mesh = new ConvexMesh(vector<PxVec3>(begin(verts), end(verts)), pose, density);
		}
	};

	class CurvedWall
	{
	public:
		vector<PxVec3> verts{ PxVec3(1,0,0), PxVec3(0,0,1), PxVec3(0,0,0) };
		ConvexMesh* mesh = new ConvexMesh(vector<PxVec3>(begin(verts), end(verts)));

		CurvedWall(float l, int divisions, PxTransform pose = PxTransform(PxIdentity), PxReal density = 1.0f)
		{
			float theta = 0.0f;

			verts[0] *= l;
			verts[1] *= l;
			for (int i = 0; i < divisions; i++)
			{
				theta = i * 2.0f * PxPi / (divisions*4);
				verts.push_back(PxVec3((l * cosf(theta)), 0.0f, (l * sinf(theta))));
			}
			for (int i = 0; i < divisions+3; i++)
			{
				verts.push_back(PxVec3(verts[i].x, verts[i].y + 1.0f, verts[i].z));
			}
			mesh = new ConvexMesh(vector<PxVec3>(begin(verts), end(verts)), pose, density);
		}
	};
}