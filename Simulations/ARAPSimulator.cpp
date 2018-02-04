#include "ARAPSimulator.h"
#include "KinectSensor.h"
#include "MassSpringSystemSimulator.h"
#include <vector>

KinectSensor *kinect;

ARAPSimulator::ARAPSimulator()
{
	m_iTestCase = 1;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	m_iNumSpheres = 100;
	m_fSphereSize = 0.05f;
	kinect = new KinectSensor();
	//create an array containing the vertices that correspond to each skeleton joint
	parseConfigFile();

}

const char * ARAPSimulator::getTestCasesStr() {
	return "Teapot,Random Objects,Triangle";
}

void ARAPSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	m_pMesh.reset();

	kinect->ResetKinect1();

	// TODO Release vertexNeighbours
}

void ARAPSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0: break;
	case 1:
		TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
		break;
	case 2:break;
	default:break;
	}
}

void ARAPSimulator::notifyCaseChanged(int testCase)
{
	float scale = 1.4f;
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0: {
		cout << "Draw model!\n";
		//m_pMesh = GeometricPrimitive::CreateMesh("../Butterfly.obj", DUC->g_pd3dImmediateContext, 0.05f, false);
		m_pMesh = GeometricPrimitive::CreateMesh("../trooper.obj", DUC->g_pd3dImmediateContext, scale, false);
		findNeighbours(&vertexNeighbours);
		alg.addMesh(m_pMesh.get(), &vertexNeighbours);
		alg.init();

		break;
	}
	case 1:
		cout << "Random Object!\n";
		m_iNumSpheres = 100;
		m_fSphereSize = 0.05f;
		break;
	case 2:
		cout << "Triangle !\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void ARAPSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
}


void ARAPSimulator::findNeighbours(std::map<uint16_t, vector<uint16_t>* >* neighborList) {
	VertexCollection vertices = m_pMesh->GetVertexList();
	IndexCollection indices = m_pMesh->GetIndexList();

	for (size_t idx = 0; idx < indices.size(); idx += 3) {
		insertVertexNeighbors(neighborList, indices[idx], indices[idx + 1], indices[idx + 2]);
		insertVertexNeighbors(neighborList, indices[idx + 1], indices[idx], indices[idx + 2]);
		insertVertexNeighbors(neighborList, indices[idx + 2], indices[idx + 1], indices[idx]);
	}
	//Sort neighborlists, this is later important for wij calculations
	for (auto kv : *neighborList) {
		sort(kv.second->begin(), kv.second->end());
	}
}


void ARAPSimulator::insertVertexNeighbors(std::map<uint16_t, vector<uint16_t>* >* neighborList, uint16_t vertex, uint16_t neighb1, uint16_t neighb2) {
	map<uint16_t, vector<uint16_t>* >::iterator iter = neighborList->find(vertex);

	// Not present
	if (iter == neighborList->end()) {
		vector<uint16_t>* neighbors = new vector<uint16_t>();
		neighbors->push_back(neighb1);
		neighbors->push_back(neighb2);
		neighborList->insert(pair<uint16_t, vector<uint16_t>* >(vertex, neighbors));
	}
	// Exists already
	else {
		vector<uint16_t>* neighbors = iter->second;
		if (std::find(neighbors->begin(), neighbors->end(), neighb1) == neighbors->end()) {
			neighbors->push_back(neighb1);
		}
		if (std::find(neighbors->begin(), neighbors->end(), neighb2) == neighbors->end()) {
			neighbors->push_back(neighb2);
		}
	}
}

void ARAPSimulator::parseConfigFile() {

	//std::ifstream infile("../Simulations/skeleton_config.txt");
	std::ifstream infile("../Simulations/IronMan_config.txt");
	std::string line;	
	//uint16_t skeleton_vertices[20];

	if (infile.is_open())
	{
		int i = 0;

		while (std::getline(infile, line) || i<20 ) //only 20 skeleton joints, if there are more lines do not process them
		{
			auto delimiterPos = line.find(":");
			auto name = line.substr(0, delimiterPos);
			auto value = line.substr(delimiterPos + 1);
			//std::cout << "name: " << name << std::endl;
			//value = "1,2,";
			//std::cout << "value: " << value << std::endl;
			if (value.find(",") != std::string::npos)
			{

				std::size_t delimit = value.find(",");
				while( delimit != std::string::npos)
				{
					auto val1 = value.substr(0, delimit);
					value=value.substr(delimit + 1);
					delimit = value.find(",");
					SetVertex.push_back(std::pair <int, int>(i, atoi(val1.c_str())));
					
				}

				
			}
			else
			{
					SetVertex.push_back(std::pair <int, int>(i, atoi(value.c_str()) ));
			}
			

			
			i++;
		}
		infile.close();
		std::cout << "vertices mapped to Skelton" << std::endl << std::endl;
		for (size_t ind = 0 ; ind < SetVertex.size(); ind++)
		{
			std::cout << "value: " << SetVertex[ind].first << " " << SetVertex[ind].second << std::endl;
		}
		std::getchar();
		
			
	}
	else {
		std::cout << "Unable to open config file." << '\n';
	}
	//return skeleton_vertices[20];

}


void ARAPSimulator::newskeletondata()
{
	auto vertices_list = m_pMesh->GetVertexList();


	//code to move the handles or move the wings 
	NUI_SKELETON_FRAME skeletonFrame;
	skeletonFrame = kinect->GetSkeletonframe();
	for (int i = 0; i < 6; i++) //Six times, because the Kinect has space to track six people
	{

		NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;
		//std::cout << "ProcessSkeleton" << std::endl;

		if (NUI_SKELETON_TRACKED == trackingState) {
			//Print "Right hand:"
			std::cout << "ProcessSkeleton = " << i << std::endl;
			
			for (size_t j = 0; j <SetVertex.size(); j++)
			{
				auto MapdSkelPart = SetVertex[j].first;
				if (SetVertex[j].second >= 0 ) {
					float x = skeletonFrame.SkeletonData[i].SkeletonPositions[MapdSkelPart].x * 2;
					float y = skeletonFrame.SkeletonData[i].SkeletonPositions[MapdSkelPart].y * 2;
					float z = skeletonFrame.SkeletonData[i].SkeletonPositions[MapdSkelPart].z * 2;
					
					/*auto MapdMeshVert = SetVertex[j].second;
					if ( MapdSkelPart == 7 || MapdSkelPart == 11 )
					{ 
						//alg.setHandle(MapdMeshVert, vertices_list[MapdMeshVert].position.x, y, vertices_list[MapdMeshVert].position.z);
						m_pMesh->SetVertex(SetVertex[j].second, { vertices_list[MapdMeshVert].position.x, y, vertices_list[MapdMeshVert].position.z });
					}						
					else
					{
						//alg.setHandle(MapdMeshVert, x, y, z);
						m_pMesh->SetVertex(SetVertex[j].second, { x, y, z });
					}
					*/

					std::cout << SetVertex[j].second << " = " << x << " " << y << " " << z << " " << std::endl;
					//m_pMesh->SetVertex(SetVertex[j].second, { x, y, z });
					alg.setHandle(SetVertex[j].second, x, y, z);
					
					//std::cout << j << " = " << x << " " << y << " " << z << " " << std::endl;
				}
			}
			
		}
	}
}

void ARAPSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case 0:
		alg.iteration_step();
		alg.updateMesh();

		newskeletondata();

		m_pMesh->UpdateBuffer(DUC->g_pd3dImmediateContext);
		break;
	case 1:
		break;
	default:
		break;
	}
}

void ARAPSimulator::drawMesh()
{
	DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0.97, 0.86, 1));
	drawMesh(m_vfMovableObjectPos, m_vfRotate, Vec3(0.5, 0.5, 0.5));
}

void ARAPSimulator::drawTriangle()
{
	DUC->DrawTriangleUsingShaders();
}

void ARAPSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0: drawMesh(); break;
	case 1:
		break;
	}
}

void ARAPSimulator::drawMesh(Vec3 pos, Quat rot, Vec3 scale)
{
	XMVECTOR posXM = pos.toDirectXVector();
	XMVECTOR scaleXM = scale.toDirectXVector();

	// Setup position/normal effect (per object variables)
	XMMATRIX s = XMMatrixScaling(XMVectorGetX(scaleXM), XMVectorGetY(scaleXM), XMVectorGetZ(scaleXM));
	XMMATRIX t = XMMatrixTranslation(XMVectorGetX(posXM), XMVectorGetY(posXM), XMVectorGetZ(posXM));
	XMMATRIX r = XMMatrixRotationQuaternion(rot.toDirectXQuat());

	DUC->g_pEffectPositionNormal->SetWorld(r * s * t * DUC->g_camera.GetWorldMatrix());
	m_pMesh->Draw(DUC->g_pEffectPositionNormal, DUC->g_pInputLayoutPositionNormal);
}

void ARAPSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void ARAPSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}


void ARAPSimulator::drawMesh(Vec3 pos, Vec3 rot, Vec3 scale)
{
	XMVECTOR posXM = pos.toDirectXVector();
	XMVECTOR rotXM = rot.toDirectXVector();
	XMVECTOR scaleXM = scale.toDirectXVector();

	// Setup position/normal effect (per object variables)
	XMMATRIX s = XMMatrixScaling(XMVectorGetX(scaleXM), XMVectorGetY(scaleXM), XMVectorGetZ(scaleXM));
	XMMATRIX t = XMMatrixTranslation(XMVectorGetX(posXM), XMVectorGetY(posXM), XMVectorGetZ(posXM));
	XMMATRIX r = XMMatrixRotationRollPitchYaw(XMVectorGetX(rotXM), XMVectorGetX(rotXM), XMVectorGetX(rotXM));

	DUC->g_pEffectPositionNormal->SetWorld(r * s * t * DUC->g_camera.GetWorldMatrix());
	m_pMesh->Draw(DUC->g_pEffectPositionNormal, DUC->g_pInputLayoutPositionNormal);
}



//This is just a helper function for Testing purposes!
void ARAPSimulator::handleHelper(int i, float x, float y, float z) {
	auto pos = m_pMesh->GetVertexList()[i].position;
	alg.setHandle(i, pos.x + x, pos.y + y, pos.z + z);
	m_pMesh->SetVertex(i, { pos.x + x, pos.y + y, pos.z + z });
};
