#include "DeformableModel.h"

DeformableModel::DeformableModel()
{
}


DeformableModel::DeformableModel(wchar_t* modelPath, DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	this->loadModel(modelPath);
}

DeformableModel::~DeformableModel()
{
	delete(this->vertexList);
}


void DeformableModel::loadModel(wchar_t* modelPath) {
	EffectFactory fx(DUC->g_ppd3Device);
	size_t data_size = 0;
	std::unique_ptr<uint8_t[]> v_data;

	// Load byte data
	HRESULT hr = BinaryReader::ReadEntireFile(modelPath, v_data, &data_size);
	uint8_t* mesh_data = v_data.get();

	auto v_header = reinterpret_cast<const SDKMESH_HEADER*>(mesh_data);
	auto vb_array = reinterpret_cast<const SDKMESH_VERTEX_BUFFER_HEADER*>(mesh_data + v_header->VertexStreamHeadersOffset);

	if (v_header->NumVertexBuffers < 1)
		throw std::exception("Vertex Buffers less than 1");
	this->vertexHeader = vb_array[0];
	uint64_t buffer_data_offset = v_header->HeaderSize + v_header->NonBufferDataSize;
	uint8_t* buffer_data = mesh_data + buffer_data_offset;

	// Convert to vertex list
	this->vertexList = (std::pair<SimpleMath::Vector3, SimpleMath::Vector3>*) malloc(vertexHeader.NumVertices * sizeof(SimpleMath::Vector3) * 2);
	memcpy(this->vertexList, reinterpret_cast<std::pair<SimpleMath::Vector3, SimpleMath::Vector3>*>(buffer_data + (vertexHeader.DataOffset - buffer_data_offset)), vertexHeader.NumVertices * sizeof(SimpleMath::Vector3) * 2);
	//this->vertexList = reinterpret_cast<std::pair<SimpleMath::Vector3, SimpleMath::Vector3>*>(buffer_data + (vertexHeader.DataOffset - buffer_data_offset));
	//memcpy(this->vertexList, this->vertexList, vertexHeader.NumVertices * sizeof(SimpleMath::Vector3));

	// Load model
	this->model = Model::CreateFromSDKMESH(DUC->g_ppd3Device, v_data.get(), data_size, fx, false, false).release();
	//this->model = Model::CreateFromSDKMESH(DUC->g_ppd3Device, L"../benchy.sdkmesh", fx, false, false).release();
	//model->name = modelPath;
}


void DeformableModel::draw(DrawingUtilitiesClass* DUC) {
	/*XMVECTOR posDX = pos.toDirectXVector();
	XMVECTOR rotDX = rot.toDirectXVector();
	XMVECTOR scaleDX = scale.toDirectXVector();

	XMMATRIX s = XMMatrixScaling(XMVectorGetX(scaleDX), XMVectorGetY(scaleDX), XMVectorGetZ(scaleDX));
	XMMATRIX t = XMMatrixTranslation(XMVectorGetX(posDX), XMVectorGetY(posDX), XMVectorGetZ(posDX));
	XMMATRIX r = XMMatrixRotationRollPitchYaw(XMVectorGetX(rotDX), XMVectorGetX(rotDX), XMVectorGetX(rotDX));*/

	// Load model and vertices
	/*size_t data_size = 0;
	std::unique_ptr<uint8_t[]> v_data;
	HRESULT hr = BinaryReader::ReadEntireFile(L"../benchy.sdkmesh", v_data, &data_size);
	uint8_t* mesh_data = v_data.get();
	auto model = DirectX::Model::CreateFromSDKMESH(ppd3Device, v_data.get(), data_size, fx, false, false);
	model->name = L"../benchy.sdkmesh";
	auto v_header = reinterpret_cast<const SDKMESH_HEADER*>(mesh_data);
	auto vb_array = reinterpret_cast<const SDKMESH_VERTEX_BUFFER_HEADER*>(mesh_data + v_header->VertexStreamHeadersOffset);

	if (v_header->NumVertexBuffers < 1)
		throw std::exception("Vertex Buffers less than 1");
	auto& vertex_header = vb_array[0];
	uint64_t buffer_data_offset = v_header->HeaderSize + v_header->NonBufferDataSize;
	uint8_t* buffer_data = mesh_data + buffer_data_offset;
	auto verts_pairs = reinterpret_cast<std::pair<SimpleMath::Vector3, SimpleMath::Vector3>*>(buffer_data + (vertex_header.DataOffset - buffer_data_offset));
	auto verts = reinterpret_cast<const uint8_t*>(buffer_data + (vertex_header.DataOffset - buffer_data_offset));

	verts_pairs[0].first.x = 100.0f;
	verts_pairs[1].first.x = 100.0f;
	auto convertedVertices = reinterpret_cast<const uint8_t*>(verts_pairs);*/


	// New vertex buffer
	D3D11_BUFFER_DESC desc = {};
	desc.Usage = D3D11_USAGE_DEFAULT;
	desc.ByteWidth = static_cast<UINT>(this->vertexHeader.SizeBytes);
	desc.BindFlags = D3D11_BIND_VERTEX_BUFFER;

	// Replace vertex buffer
	D3D11_SUBRESOURCE_DATA initData = {};
	initData.pSysMem = reinterpret_cast<const uint8_t*>(this->vertexList);

	for (auto it = model->meshes.cbegin(); it != model->meshes.cend(); ++it)
	{
		auto mesh = it->get();
		assert(mesh != 0);

		for (auto it = mesh->meshParts.cbegin(); it != mesh->meshParts.cend(); ++it)
		{
			auto part = (*it).get();
			assert(part != 0);

			ThrowIfFailed(
				DUC->g_ppd3Device->CreateBuffer(&desc, &initData, &part->vertexBuffer)
			);
		}
	}

	// Draw mesh
	CommonStates states(DUC->g_ppd3Device);
	XMMATRIX world = DUC->g_camera.GetWorldMatrix();
	XMMATRIX view = DUC->g_camera.GetViewMatrix();
	XMMATRIX proj = DUC->g_camera.GetProjMatrix();

	model->Draw(DUC->g_pd3dImmediateContext, states, world, view, proj);
}