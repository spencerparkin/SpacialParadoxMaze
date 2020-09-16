#include "SpacialParadoxMazeSceneProxy.h"
#include "SpacialParadoxMazeComponent.h"

SpacialParadoxMazeSceneProxy::SpacialParadoxMazeSceneProxy(const UPrimitiveComponent* component) : FPrimitiveSceneProxy(component)
{
	this->mazeComponent = Cast<USpacialParadoxMazeComponent>(component);
}

/*virtual*/ SpacialParadoxMazeSceneProxy::~SpacialParadoxMazeSceneProxy()
{
}

/*virtual*/ SIZE_T SpacialParadoxMazeSceneProxy::GetTypeHash() const
{
	static size_t UniquePointer;
	return reinterpret_cast<size_t>(&UniquePointer);
}

/*virtual*/ void SpacialParadoxMazeSceneProxy::GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily, uint32 VisibilityMap, FMeshElementCollector& Collector) const
{
	if (this->mazeComponent)
	{
		//...
	}
}

/*virtual*/ FPrimitiveViewRelevance SpacialParadoxMazeSceneProxy::GetViewRelevance(const FSceneView* View) const
{
	FPrimitiveViewRelevance relevance;
	relevance.bDynamicRelevance = true;
	relevance.bDrawRelevance = true;
	return relevance;
}