#ifndef DEFORMABLE_MODEL_h
#define DEFORMABLE_MODEL_h

// DXUT includes
#include <DXUT.h>
#include <DXUTcamera.h>
// DirectXTK includes
#include "Model.h"
#include "Effects.h"
#include "CommonStates.h"
#include "../DirectXTK/Src/BinaryReader.h"
#include "SDKMesh.h"
#include <SimpleMath.h>
// vector mat quat
#include "util/vectorbase.h"
#include "util/matrixbase.h"

#include "DrawingUtilitiesClass.h"

using namespace DirectX;
using namespace GamePhysics;

class DeformableModel {
public:
	// Construtors
	DeformableModel();
	DeformableModel(wchar_t* modelPath, DrawingUtilitiesClass* DUC);
	~DeformableModel();

	void draw(DrawingUtilitiesClass* DUC);


private:
	DrawingUtilitiesClass* DUC;
	Model* model;
	std::pair<SimpleMath::Vector3, SimpleMath::Vector3>* vertexList;
	SDKMESH_VERTEX_BUFFER_HEADER vertexHeader;

	void loadModel(wchar_t* modelPath);
};

#endif