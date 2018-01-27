#include "ARAPSimulator.h"

#include "KinectSensor.h"
KinectSensor *kinect;


ARAPSimulator::ARAPSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	m_iNumSpheres = 100;
	m_fSphereSize = 0.05f;
	kinect = new KinectSensor();
	handle_vertex = 10;
}

const char * ARAPSimulator::getTestCasesStr() {
	return "Teapot,Random Objects,Triangle";
}

void ARAPSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	m_pMesh.reset();
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
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0: {
		cout << "Draw model!\n";

		m_pMesh = GeometricPrimitive::CreateMesh("../Butterfly.obj", DUC->g_pd3dImmediateContext, 0.1f, false);
		findNeighbours(&vertexNeighbours);

		//kinect = new KinectSensor();
		//kinect->Update();

		//kinect->ProcessSkeleton();

		// Test move vertex
		//uint16_t index = 0;
		//XMFLOAT3 newPosition = { 50,50,50 };
		//m_pMesh->SetVertex(index, newPosition);
		//index = 10;
		//newPosition = { 100,100,100 };
		//m_pMesh->SetVertex(index, newPosition);


		// Update vertex buffer do display changes
		//m_pMesh->UpdateBuffer(DUC->g_pd3dImmediateContext);

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

	for (uint16_t idx = 0; idx < indices.size(); idx += 3) {
		insertVertexNeighbors(neighborList, indices[idx], indices[idx + 1], indices[idx + 2]);
		insertVertexNeighbors(neighborList, indices[idx + 1], indices[idx], indices[idx + 2]);
		insertVertexNeighbors(neighborList, indices[idx + 2], indices[idx + 1], indices[idx]);
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

void ARAPSimulator::newskeletondata()
{


	auto vertices_list = m_pMesh->GetVertexList();
	std::cout << "number of vertices " << vertices_list.size() << std::endl;
	XMFLOAT3 newPosition;

	/*
	float step = .0035;
	for (int i = 0; i < vertices_list.size(); i++)
	{

	//extract the old coordinates
	newPosition = { vertices_list[i].position.x ,vertices_list[i].position.y,vertices_list[i].position.z+ step };
	//change only the z value

	//assign the vertiex back to the model

	m_pMesh->SetVertex(i, newPosition);

	}

	*/


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
			XMFLOAT3 newKinectPostion;
			newKinectPostion.x = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].x * 2;
			newKinectPostion.y = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].y * 2;
			newKinectPostion.z = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].z * 2;

			std::cout << "Right Hand: ";
			std::cout << newKinectPostion.x << " " << newKinectPostion.y << " " << newKinectPostion.z << std::endl;
			uint16_t handlearray[] = { 49,17,172 };
			int mesh_index;

			//code to move the flap the butterfly 
			for (int j = 0; j < 3; j++)
			{

				for (handle_vertex = 0; handle_vertex < 6; handle_vertex++)
				{ 
					mesh_index = handlearray[j] + handle_vertex;

					newPosition = { newKinectPostion.x ,newKinectPostion.y,newKinectPostion.z };

					//newPosition = { vertices_list[mesh_index].position.x ,newKinectPostion.y,vertices_list[mesh_index].position.z };
					m_pMesh->SetVertex(mesh_index, newPosition);
				}
			}

			////code to move the whole butterfly
			/*
			for (handle_vertex = 0; handle_vertex < 6; handle_vertex++)
			{
				mesh_index = handlearray[j] + handle_vertex;

				newPosition = { vertices_list[mesh_index].position.x ,newKinectPostion.y,vertices_list[mesh_index].position.z };
				m_pMesh->SetVertex(mesh_index, newPosition);
			}
			*/

		}
	}
	



	/*
	NUI_SKELETON_FRAME skeletonFrame;
	skeletonFrame = kinect->GetSkeletonframe();

	for (int i = 0; i < 6; i++) //Six times, because the Kinect has space to track six people
	{

	NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;
	//std::cout << "ProcessSkeleton" << std::endl;

	if (NUI_SKELETON_TRACKED == trackingState) {
	//Print "Right hand:"
	std::cout << "ProcessSkeleton = " << i << std::endl;
	XMFLOAT3 newPosition;
	newPosition.x = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].x * 2;
	newPosition.y = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].y * 2;
	newPosition.z = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT].z * 2;

	std::cout << "Right Hand: ";
	std::cout << newPosition.x << " " << newPosition.y << " " << newPosition.z << std::endl;
	uint16_t handlearray[] = { 49,17,172 };
	for (int i = 0; i < 3; i++)
	{

	for (handle_vertex = 0; handle_vertex < 6; handle_vertex++)
	m_pMesh->SetVertex(handlearray[i]+handle_vertex, newPosition);


	}

	m_pMesh->UpdateBuffer(DUC->g_pd3dImmediateContext);

	}
	}
	*/
	m_pMesh->UpdateBuffer(DUC->g_pd3dImmediateContext);
}

void ARAPSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case 0:

		newskeletondata();
		//kinect->ProcessSkeleton();
		// rotate the teapot
		/*m_vfRotate.x += timeStep;
		if (m_vfRotate.x > 2 * M_PI) m_vfRotate.x -= 2.0f * (float)M_PI;
		m_vfRotate.y += timeStep;
		if (m_vfRotate.y > 2 * M_PI) m_vfRotate.y -= 2.0f * (float)M_PI;
		m_vfRotate.z += timeStep;
		if (m_vfRotate.z > 2 * M_PI) m_vfRotate.z -= 2.0f * (float)M_PI;*/

		break;
	default:
		break;
	}
}

void ARAPSimulator::drawSomeRandomObjects()
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	for (int i = 0; i<m_iNumSpheres; i++)
	{
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(Vec3(randPos(eng), randPos(eng), randPos(eng)), Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));
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
	case 1: drawSomeRandomObjects(); break;
	case 2: drawTriangle(); break;
	}
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