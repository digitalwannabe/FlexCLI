// FlexCLI.h
#pragma once

////dw: fix for slimdx compatibility, preventing std::errc namespace already defined error
//#define errc Errc
//#define io_errc Io_errc

#include "NvFlex.h"
#include "NvFlexExt.h"
#include <vector>
#include <map>
#include <iostream>
#include <stdio.h>


//added by digitalwannabe
#pragma comment(lib, "d3d11.lib")
#include <d3d11.h>
#include <DirectXMath.h>
using namespace DirectX;
using namespace FeralTic::DX11::Resources;



using namespace System;
using namespace System::Collections::Generic;
using namespace System::Runtime::InteropServices;

typedef XMFLOAT3 float3;
typedef XMFLOAT4 float4;

namespace FlexCLI {

	//Always declare all classes first here. Otherwise the compiler won't find anything.
	ref class Flex;
	ref class FlexCollisionGeometry;
	ref class FlexParams;
	ref class FlexScene;
	ref class FlexParticle;
	ref struct FlexSolverOptions;
	ref class FlexUtils;
	ref class FlexForceField;


	//dw
	typedef DirectX::XMFLOAT3 float3;
	typedef DirectX::XMFLOAT4 float4;

	struct nativeBufferStructInt {

		int value;
	};

	struct nativeBufferStructFloat3 {

		float3 value;
	};

	struct nativeBufferStructFloat4 {

		float4 value;
	};

	class DX11NativePointer
	{
	public:

		DX11NativePointer();

		/// \brief destructor
		~DX11NativePointer();

		ID3D11Device* device;
		ID3D11DeviceContext* deviceContext;
		ID3D11Buffer* particles;
		ID3D11ShaderResourceView*	pSRV;
		ID3D11UnorderedAccessView*	pUAV;

	};
	
	
	//dw

	public ref class Flex : IDX11Resource  //note: this interface provides support for vvvv render pipeline (only requires to implement IDisposable)
	{
		// public: Everything accessible from FlexHopper
	public:
		
		Flex();		
		FlexScene^ Scene;
		void SetCollisionGeometry(FlexCollisionGeometry^ flexCollisionGeometry);
		void SetParams(FlexParams^ flexParams);
		void SetScene(FlexScene^ flexScene);
		void SetSolverOptions(FlexSolverOptions^ flexSolverOptions);
		void SetForceFields(List<FlexForceField^>^ flexForceFields);
		bool IsReady();
		void UpdateSolver();
		void Destroy();

		//dw
		//alternative constructor, which accepts a dx11 context, creates a buffer on the context and registers the buffer with flex
		Flex(IntPtr dx11Device, IntPtr dx11DeviceContext); 
		void GetParticlesBuffer();
		IntPtr GetDX11Pointer(int index);
		DX11NativePointer* nativePointers;
		~Flex();
	internal:
		void SetParticles(List<FlexParticle^>^ flexParticles);
		void SetRigids(List<int>^ offsets, List<int>^ indices, List<float>^ restPositions, List<float>^ restNormals, List<float>^ stiffnesses, List<float>^ plasticThresholds, List<float>^ plasticCreeps, List<float>^ rotations, List<float>^ translations);
		void SetSprings(List<int>^ springPairIndices, List<float>^ springLengths, List<float>^ springCoefficients);
		void SetDynamicTriangles(List<int>^ triangleIndices, List<float>^ normals);
		void SetInflatables(List<int>^ startIndices, List<int>^ numTriangles, List<float>^ restVolumes, List<float>^ overPressures, List<float>^ constraintScales);

		static void DecomposePhase(int phase, int %groupIndex, bool %selfCollision, bool %fluid);

		//called in each update cycle
		List<FlexParticle^>^ GetParticles();
		List<FlexForceField^>^ FlexForceFields;
		//dw: changed to GetRigids
		void GetRigids(List<float>^ %translations, List<float>^ %rotations, List<float>^ %restPositions, List<float>^ %restNormals);

		//dw: buffer versions
		/*void SetParticlesBuffer(List<FlexParticle^>^ flexParticles);*/
	};

	// Structs as they is presented to .Net
	public ref class FlexParams {
	public:
#pragma region fields
		int NumIterations;					//!< Number of solver iterations to perform per-substep

		float GravityX;						//!< Constant acceleration applied to all particles
		float GravityY;
		float GravityZ;
		float Radius;						//!< The maximum interaction radius for particles
		float SolidRestDistance;			//!< The distance non-fluid particles attempt to maintain from each other, must be in the range (0, radius]
		float FluidRestDistance;			//!< The distance fluid particles are spaced at the rest density, must be in the range (0, radius], for fluids this should generally be 50-70% of mRadius, for rigids this can simply be the same as the particle radius

											// common params
		float DynamicFriction;				//!< Coefficient of friction used when colliding against shapes
		float StaticFriction;				//!< Coefficient of static friction used when colliding against shapes
		float ParticleFriction;				//!< Coefficient of friction used when colliding particles
		float Restitution;					//!< Coefficient of restitution used when colliding against shapes, particle collisions are always inelastic
		float Adhesion;						//!< Controls how strongly particles stick to surfaces they hit, default 0.0, range [0.0, +inf]
		float SleepThreshold;				//!< Particles with a velocity magnitude < this threshold will be considered fixed

		float MaxSpeed;						//!< The magnitude of particle velocity will be clamped to this value at the end of each step
		float MaxAcceleration;				//!< The magnitude of particle acceleration will be clamped to this value at the end of each step (limits max velocity change per-second), useful to avoid popping due to large interpenetrations

		float ShockPropagation;				//!< Artificially decrease the mass of particles based on height from a fixed reference point, this makes stacks and piles converge faster
		float Dissipation;					//!< Damps particle velocity based on how many particle contacts it has
		float Damping;						//!< Viscous drag force, applies a force proportional, and opposite to the particle velocity

											// cloth params
		float WindX;							//!< Constant acceleration applied to particles that belong to dynamic triangles, drag needs to be > 0 for wind to affect triangles
		float WindY;
		float WindZ;
		float Drag;							//!< Drag force applied to particles belonging to dynamic triangles, proportional to velocity^2*area in the negative velocity direction
		float Lift;							//!< Lift force applied to particles belonging to dynamic triangles, proportional to velocity^2*area in the direction perpendicular to velocity and (if possible), parallel to the plane normal

											// fluid params
/*		bool Fluid;		*/					//!< If true then particles with phase 0 are considered fluid particles and interact using the position based fluids method
		float Cohesion;						//!< Control how strongly particles hold each other together, default: 0.025, range [0.0, +inf]
		float SurfaceTension;				//!< Controls how strongly particles attempt to minimize surface area, default: 0.0, range: [0.0, +inf]    
		float Viscosity;					//!< Smoothes particle velocities using XSPH viscosity
		float VorticityConfinement;			//!< Increases vorticity by applying rotational forces to particles
		float AnisotropyScale;				//!< Control how much anisotropy is present in resulting ellipsoids for rendering, if zero then anisotropy will not be calculated, see NvFlexGetAnisotropy()
		float AnisotropyMin;				//!< Clamp the anisotropy scale to this fraction of the radius
		float AnisotropyMax;				//!< Clamp the anisotropy scale to this fraction of the radius
		float Smoothing;					//!< Control the strength of Laplacian smoothing in particles for rendering, if zero then smoothed positions will not be calculated, see NvFlexGetSmoothParticles()
		float SolidPressure;				//!< Add pressure from solid surfaces to particles
		float FreeSurfaceDrag;				//!< Drag force applied to boundary fluid particles
		float Buoyancy;						//!< Gravity is scaled by this value for fluid particles

											// diffuse params
		float DiffuseThreshold;				//!< Particles with kinetic energy + divergence above this threshold will spawn new diffuse particles
		float DiffuseBuoyancy;				//!< Scales force opposing gravity that diffuse particles receive
		float DiffuseDrag;					//!< Scales force diffuse particles receive in direction of neighbor fluid particles
		int DiffuseBallistic;				//!< The number of neighbors below which a diffuse particle is considered ballistic
		//float DiffuseSortAxisX;				//!< Diffuse particles will be sorted by depth along this axis if non-zero
		//float DiffuseSortAxisY;
		//float DiffuseSortAxisZ;
		float DiffuseLifetime;				//!< Time in seconds that a diffuse particle will live for after being spawned, particles will be spawned with a random lifetime in the range [0, diffuseLifetime]

											// rigid params
		//float PlasticThreshold;				//!< Particles belonging to rigid shapes that move with a position delta magnitude > threshold will be permanently deformed in the rest pose
		//float PlasticCreep;					//!< Controls the rate at which particles in the rest pose are deformed for particles passing the deformation threshold 

											// collision params
		float CollisionDistance;			//!< Distance particles maintain against shapes, note that for robust collision against triangle meshes this distance should be greater than zero
		float ParticleCollisionMargin;		//!< Increases the radius used during neighbor finding, this is useful if particles are expected to move significantly during a single step to ensure contacts aren't missed on subsequent iterations
		float ShapeCollisionMargin;			//!< Increases the radius used during contact finding against kinematic shapes

		float* Planes;						//!< Collision planes in the form ax + by + cz + d = 0
		int NumPlanes;						//!< Num collision planes

		int RelaxationMode;					//!< How the relaxation is applied inside the solver; 0: global, 1: local
		float RelaxationFactor;				//!< Control the convergence rate of the parallel solver, default: 1, values greater than 1 may lead to instability
#pragma endregion
		FlexParams();
		bool IsValid();
		String^ ToString() override;
		int TimeStamp;
	};

	public ref class FlexCollisionGeometry {
	public:
		FlexCollisionGeometry();

		void AddPlane(float A, float B, float C, float D);
		void AddSphere(array<float>^ centerXYZ, float radius);
		void AddBox(array<float>^ halfHeightsXYZ, array<float>^ centerXYZ, array<float>^ rotationABCD);
		void AddCapsule(float halfHeightX, float radius, array<float>^ centerXYZ, array<float>^ rotationABCD);
		void AddMesh(array<float>^ vertices, array<int>^ faces);
		void AddConvexShape(array<float>^ planes, array<float>^ upperLimit, array<float>^ lowerLimit);

		int TimeStamp;
	internal:
		//Plane properties
		array<float>^ Planes;
		int NumPlanes;
		//Sphere properties
		int NumSpheres;
		List<float>^ SphereCenters;
		List<float>^ SphereRadii;
		//Box properties
		int NumBoxes;
		List<float>^ BoxHalfHeights;
		List<float>^ BoxCenters;
		List<float>^ BoxRotations;
		//Capsule properties
		int NumCapsules;
		List<float>^ CapsuleHalfHeights;
		List<float>^ CapsuleRadii;
		List<float>^ CapsuleCenters;
		List<float>^ CapsuleRotations;
		//Mesh properties
		int NumMeshes;
		List<array<float>^>^ MeshVertices;
		List<array<int>^>^ MeshFaces;
		List<array<float>^>^ MeshLowerBounds;
		List<array<float>^>^ MeshUpperBounds;
		//ConvexShape properties
		int NumConvex;
		List<array<float>^>^ ConvexPlanes;
		List<array<float>^>^ ConvexLowerBounds;
		List<array<float>^>^ ConvexUpperBounds;
	};

	public ref class FlexScene {
	public:
		///<summary>Empty constructor</summary>
		FlexScene();

		///<summary>Number of all particles in the scene</summary>
		int NumParticles() { return Particles->Count; };
		int NumRigidBodies() { 
			return NumActualRigids; 
		};

		List<FlexParticle^>^ Particles;
		List<FlexParticle^>^ GetAllParticles();

		//Particles in general
		void RegisterParticles(array<float>^ positions, array<float>^ velocities, array<float>^ inverseMasses, bool isFluid, bool selfCollision, int groupIndex);

		//Fluids
		void RegisterFluid(array<float>^ positions, array<float>^ velocities, array<float>^ inverseMasses, int groupIndex);
		List<FlexParticle^>^ GetFluidParticles();

		//Rigids
		int NumRigids() { return RigidOffsets->Count - 1; };
		void RegisterRigidBody(array<float>^ vertices, array<float>^ vertexNormals, array<float>^ velocity, array<float>^ inverseMasses, float stiffness, float plasticThreshold, float plasticCreep, int groupIndex);
		List<FlexParticle^>^ GetRigidParticles();
		List<float>^ GetRigidRotations() { return RigidRotations; };
		List<float>^ GetRigidTranslations() { return RigidTranslations; };
		List<float>^ GetRigidRestPositions() { return RigidRestPositions; }; //dw
		List<float>^ GetRigidRestNormals() { return RigidRestNormals; }; //dw
		List<float>^ GetShapeMassCenters() { return ShapeMassCenters; }

		//Softs
		static void InitSoftBodyFromMesh(void*% asset, array<float>^ vertices, array<int>^ triangles, float particleSpacing, float volumeSampling, float surfaceSampling, float clusterSpacing, float clusterRadius, float clusterStiffness, float linkRadius, float linkStiffness, float globalStiffness, float clusterPlasticThreshold, float	clusterPlasticCreep);
		static void UnwrapSoftBody(void* asset, array<float>^% particles, array<int>^% springIndices, array<array<int>^>^% shapeIndices);
		static void DestroySoftBody(NvFlexExtAsset* asset);
		List<List<FlexParticle^>^>^ GetSoftParticles();

		//Springs
		int NumSprings() { return (int)(SpringPairIndices->Count * 0.5); };
		List<int>^ GetSpringPairIndices();
		int RegisterSpringSystem(array<float>^ positions, array<float>^ velocities, array<float>^ inverseMasses, array<int>^ springPairIndices, array<float>^ stiffnesses, array<float>^ defaultLengths, bool selfCollision, array<int>^ anchorIndices, int groupIndex);
		List<FlexParticle^>^ GetSpringParticles();

		//Cloth
		int GetNumCloths() { return NumCloths; };
		void RegisterCloth(array<float>^ positions, array<float>^ velocities, array<float>^ inverseMasses, array<int>^ triangles, array<float>^ triangleNormals, float stretchStiffness, float bendingStiffness, float preTensionFactor, array<int>^ anchorIndices, bool selfCollision, int groupIndex);
		List<FlexParticle^>^ GetClothParticles();

		//Inflatables
		int GetNumInflatables() { return NumInflatables; };
		void RegisterInflatable(array<float>^ positions, array<float>^ velocities, array<float>^ inverseMasses, array<int>^ triangles, array<float>^ triangleNormals, float stretchStiffness, float bendingStiffness, float preTensionFactor, float restVolume, float overPressure, float constraintScale, array<int>^ anchorIndices, bool selfCollision, int groupIndex);
		List<FlexParticle^>^ GetInflatableParticles();

		//Custom Constraints
		bool RegisterCustomConstraints(array<int>^ anchorIndices, array<int>^ shapeMatchingIndices, float shapeStiffness, float plasticThreshold, float plasticCreep, array<int>^ springPairIndices, array<float>^ springStiffnesses, array<float>^ springDefaultLengths, array<int>^ triangleIndices, array<float>^ triangleNormals);

		//Exposes RegisterAsset to managed code, using unsigned long as asset pointer NvFlexExtAsset*
		void RegisterAsset(unsigned long long asset, array<float>^ velocity, float invMass, int groupIndex, bool isSoftBody) { RegisterAsset((NvFlexExtAsset*)asset, velocity, invMass, groupIndex, isSoftBody); }

		bool IsValid();
		String^ ToString() override;
		int TimeStamp;

		FlexScene^ AppendScene(FlexScene^ newScene);
		FlexScene^ AlterScene(FlexScene^ alteredScene, bool includeAllParticles);

	internal:
		//reference to flex class
		Flex^ Flex;
		void RegisterAsset(NvFlexExtAsset* asset, array<float>^ velocity, float invMass, int groupIndex, bool isSoftBody);
		//Fluids
		List<int>^ FluidIndices;
		//Rigids
		List<int>^ RigidIndices;
		List<int>^ RigidOffsets;
		int NumActualRigids; //number of rigids and not soft bodies
		List<int>^ SoftBodyOffsets; //particle offset related to each soft body (not indivudal shapes, or "rigids" within a single soft body
		List<float>^ ShapeMassCenters;
		List<float>^ RigidRestPositions;
		List<float>^ RigidRestNormals;
		List<float>^ RigidStiffnesses;
		List<float>^ RigidRotations;
		List<float>^ RigidTranslations;
		List<float>^ PlasticThresholds; //dw
		List<float>^ PlasticCreeps; //dw
		List<int>^ SpringIndices;
		List<int>^ SpringPairIndices;
		List<float>^ SpringLengths;
		List<float>^ SpringStiffnesses;
		//Cloth Constraints
		int NumCloths;
		List<int>^ ClothIndices;
		//Dynamic triangles for cloth, inflatables
		List<int>^ DynamicTriangleIndices;
		List<float>^ DynamicTriangleNormals;
		//Inflatables
		int NumInflatables;
		List<int>^ InflatableIndices;
		List<int>^ InflatableStartIndices;
		List<int>^ InflatableNumTriangles;
		List<float>^ InflatableRestVolumes;
		List<float>^ InflatableOverPressures;
		List<float>^ InflatableConstraintScales;
	};

	public ref class FlexParticle {
	public:
		FlexParticle(array<float>^ position, array<float>^ velocity, float inverseMass, bool selfCollision, bool isFluid, int groupIndex, bool isActive);
		FlexParticle(array<float>^ position, array<float>^ velocity, float inverseMass, int phase, bool isActive);
		float PositionX, PositionY, PositionZ, InverseMass, VelocityX, VelocityY, VelocityZ;
		int GroupIndex;
		bool SelfCollision;
		bool IsFluid;
		int Phase;
		bool IsActive = true;
		bool IsValid();
		String^ ToString() override;
		
		//dw
		ID3D11Buffer* Position;
		ID3D11Buffer* Velocity;
	};

	//dw
	//public ref class FlexBufferedParticle {
	//public:
	//	FlexBufferedParticle(array<float>^ position, array<float>^ velocity, float inverseMass, bool selfCollision, bool isFluid, int groupIndex, bool isActive);
	//	FlexBufferedParticle(array<float>^ position, array<float>^ velocity, float inverseMass, int phase, bool isActive);
	//	/*float PositionX, PositionY, PositionZ, InverseMass, VelocityX, VelocityY, VelocityZ;*/
	//	ID3D10Buffer* Position; //+ invMass in w component
	//	ID3D11Buffer* Velocity;
	//	int GroupIndex;
	//	bool SelfCollision;
	//	bool IsFluid; //per group
	//	int Phase;
	//	bool IsActive = true;
	//	bool IsValid();
	//	String^ ToString() override;
	//};

	public ref struct FlexSolverOptions {
	public:
		FlexSolverOptions();
		FlexSolverOptions(float dt, int subSteps, int numIterations, int sceneMode, int fixedNumTotalIterations);
		int SubSteps;
		int NumIterations;
		float dT;
		int SceneMode;
		int FixedTotalIterations;
		bool IsValid();
		String^ ToString() override;
		int TimeStamp;
	};

	public ref class FlexForceField {
	public:
		FlexForceField(array<float>^ position, float radius, float strength, bool linearFallOff, int mode);
		float Radius;
		float Strength;
		bool LinearFallOff;
		array<float>^ Position;
		int Mode;
		int TimeStamp;

		String^ ToString() override;
	};

	//////////////////////
	//dw
	//helper function to create a structured buffer
	//helper function to create a structured buffer
	template <class T>
	HRESULT CreateStructuredBuffer(
		ID3D11Device*               pd3dDevice,
		const UINT                  iNumElements,
		const bool                  isCpuWritable,
		const bool                  isGpuWritable,
		ID3D11Buffer**              ppBuffer,
		ID3D11ShaderResourceView**  ppSRV,
		ID3D11UnorderedAccessView** ppUAV,
		const T*                    pInitialData = NULL)
	{
		HRESULT hr = S_OK;

		assert(pd3dDevice != NULL);
		assert(ppBuffer != NULL);
		assert(ppSRV != NULL);

		D3D11_BUFFER_DESC bufferDesc;
		ZeroMemory(&bufferDesc, sizeof(bufferDesc));
		bufferDesc.ByteWidth = iNumElements * sizeof(T);

		if ((!isCpuWritable) && (!isGpuWritable))
		{
			bufferDesc.CPUAccessFlags = 0;
			bufferDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
			bufferDesc.Usage = D3D11_USAGE_IMMUTABLE;
		}
		else if (isCpuWritable && (!isGpuWritable))
		{
			bufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
			bufferDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
			bufferDesc.Usage = D3D11_USAGE_DYNAMIC;
		}
		else if ((!isCpuWritable) && isGpuWritable)
		{
			bufferDesc.CPUAccessFlags = 0;
			bufferDesc.BindFlags = (D3D11_BIND_SHADER_RESOURCE |
				D3D11_BIND_UNORDERED_ACCESS);
			bufferDesc.Usage = D3D11_USAGE_DEFAULT;
		}
		else
		{
			assert((!(isCpuWritable && isGpuWritable)));
		}

		bufferDesc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
		bufferDesc.StructureByteStride = sizeof(T);

		D3D11_SUBRESOURCE_DATA bufferInitData;
		ZeroMemory((&bufferInitData), sizeof(bufferInitData));
		bufferInitData.pSysMem = pInitialData;
		hr = pd3dDevice->CreateBuffer((&bufferDesc),
			(pInitialData) ? (&bufferInitData) : NULL,
			ppBuffer);

		D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc;
		ZeroMemory(&srvDesc, sizeof(srvDesc));
		srvDesc.Format = DXGI_FORMAT_UNKNOWN;
		srvDesc.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
		srvDesc.Buffer.ElementWidth = iNumElements;
		hr = pd3dDevice->CreateShaderResourceView(*ppBuffer, &srvDesc, ppSRV);

		if (isGpuWritable)
		{
			assert(ppUAV != NULL);

			D3D11_UNORDERED_ACCESS_VIEW_DESC uavDesc;
			ZeroMemory((&uavDesc), sizeof(uavDesc));
			uavDesc.Format = DXGI_FORMAT_UNKNOWN;
			uavDesc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
			uavDesc.Buffer.NumElements = iNumElements;
			hr = pd3dDevice->CreateUnorderedAccessView(*ppBuffer, &uavDesc, ppUAV);
		}

		return hr;
	}
}