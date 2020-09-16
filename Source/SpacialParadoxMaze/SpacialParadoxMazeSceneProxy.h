#pragma once

#include "PrimitiveSceneProxy.h"

class USpacialParadoxMazeComponent;

class SpacialParadoxMazeSceneProxy : public FPrimitiveSceneProxy
{
public:
	SpacialParadoxMazeSceneProxy(const UPrimitiveComponent* component);
	virtual ~SpacialParadoxMazeSceneProxy();

	virtual SIZE_T GetTypeHash() const override;
	virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily, uint32 VisibilityMap, FMeshElementCollector& Collector) const override;
	virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override;

	virtual uint32 GetMemoryFootprint(void) const override { return sizeof(*this) + this->GetAllocatedSize(); }
	uint32 GetAllocatedSize(void) const { return FPrimitiveSceneProxy::GetAllocatedSize(); }

private:
	const USpacialParadoxMazeComponent* mazeComponent;
};