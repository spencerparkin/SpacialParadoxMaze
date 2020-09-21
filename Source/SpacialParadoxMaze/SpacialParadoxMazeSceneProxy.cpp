#include "SpacialParadoxMazeSceneProxy.h"
#include "SpacialParadoxMazeComponent.h"

SpacialParadoxMazeSceneProxy::SpacialParadoxMazeSceneProxy(const UPrimitiveComponent* component) : FPrimitiveSceneProxy(component)
{
	this->mazeComponent = Cast<USpacialParadoxMazeComponent>(component);
	this->vertexFactory = nullptr;
}

/*virtual*/ SpacialParadoxMazeSceneProxy::~SpacialParadoxMazeSceneProxy()
{
	this->indexBuffer.ReleaseResource();
	this->vertexBuffers.PositionVertexBuffer.ReleaseResource();
	this->vertexBuffers.StaticMeshVertexBuffer.ReleaseResource();
	this->vertexBuffers.ColorVertexBuffer.ReleaseResource();

	if (this->vertexFactory)
	{
		this->vertexFactory->ReleaseResource();
		delete this->vertexFactory;
		this->vertexFactory = nullptr;
	}
}

/*virtual*/ SIZE_T SpacialParadoxMazeSceneProxy::GetTypeHash() const
{
	static size_t UniquePointer;
	return reinterpret_cast<size_t>(&UniquePointer);
}

/*virtual*/ void SpacialParadoxMazeSceneProxy::GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily, uint32 VisibilityMap, FMeshElementCollector& Collector) const
{
	if (!this->vertexFactory)
	{
		int maxVertices = 256;

		this->vertexBuffers.PositionVertexBuffer.Init(maxVertices);
		this->vertexBuffers.StaticMeshVertexBuffer.Init(maxVertices, 1);
		this->vertexBuffers.ColorVertexBuffer.Init(maxVertices);

		this->vertexBuffers.PositionVertexBuffer.InitResource();
		this->vertexBuffers.StaticMeshVertexBuffer.InitResource();
		this->vertexBuffers.ColorVertexBuffer.InitResource();

		FLocalVertexFactory::FDataType dataType;

		this->vertexFactory = new FLocalVertexFactory(this->mazeComponent->GetScene()->GetFeatureLevel(), "SpacialParadoxMazeVertexFactory");

		this->vertexBuffers.PositionVertexBuffer.BindPositionVertexBuffer(this->vertexFactory, dataType);
		this->vertexBuffers.StaticMeshVertexBuffer.BindTangentVertexBuffer(this->vertexFactory, dataType);
		this->vertexBuffers.StaticMeshVertexBuffer.BindTexCoordVertexBuffer(this->vertexFactory, dataType);
		this->vertexBuffers.ColorVertexBuffer.BindColorVertexBuffer(this->vertexFactory, dataType);

		this->vertexFactory->SetData(dataType);
		this->vertexFactory->InitResource();

		this->indexBuffer.Indices.AddUninitialized(maxVertices);

		for (int j = 0; j < maxVertices; j++)
			this->indexBuffer.Indices[j] = j;
	}

	// Populate the vertex buffer.
	int i = 0;
	{
		FScopeLock scopeLock(&this->mazeComponent->renderWallListMutex);

		static FVector unitSquare[4] = {
			{0.0f, 0.0f, 0.0f},
			{1.0f, 0.0f, 0.0f},
			{1.0f, 1.0f, 0.0f},
			{0.0f, 1.0f, 0.0f}
		};

		static FVector xTangent(1.0f, 0.0f, 0.0f);
		static FVector yTangent(0.0f, 1.0f, 0.0f);
		static FVector zTangent(0.0f, 0.0f, 1.0f);

		USpacialParadoxMazeComponent::RenderWallList::TDoubleLinkedListNode* node = this->mazeComponent->renderWallList.GetHead();
		while (node)
		{
			const USpacialParadoxMazeComponent::RenderWall& renderWall = node->GetValue();

			for (int j = 0; j < 4; j++)
			{
				this->vertexBuffers.PositionVertexBuffer.VertexPosition(i) = renderWall.localToWorld.TransformPosition(unitSquare[j]);
				this->vertexBuffers.StaticMeshVertexBuffer.SetVertexTangents(i, renderWall.localToWorld.TransformVector(xTangent), renderWall.localToWorld.TransformVector(yTangent), renderWall.localToWorld.TransformVector(zTangent));
				this->vertexBuffers.StaticMeshVertexBuffer.SetVertexUV(i, 0, FVector2D(unitSquare[j].X * renderWall.uScale, unitSquare[j].Y * renderWall.vScale));
				this->vertexBuffers.ColorVertexBuffer.VertexColor(i) = FColor::Red;

				i++;
			}

			node = node->GetNextNode();
		}
	}

	for (int j = 0; j < Views.Num(); j++)
	{
		if (VisibilityMap & (1 << j))
		{
			// Send to collector here.

			//PT_QuadList
		}
	}
}

/*virtual*/ FPrimitiveViewRelevance SpacialParadoxMazeSceneProxy::GetViewRelevance(const FSceneView* View) const
{
	FPrimitiveViewRelevance relevance;
	relevance.bDynamicRelevance = true;
	relevance.bDrawRelevance = true;
	return relevance;
}