#pragma once

#include "CoreMinimal.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "SpacialParadoxMaze.h"
#include "SpacialParadoxMazeComponent.generated.h"

UCLASS(BlueprintType, Blueprintable)
class USpacialParadoxMazeComponent : public UPrimitiveComponent
{
	GENERATED_BODY()

public:

	USpacialParadoxMazeComponent();
	virtual ~USpacialParadoxMazeComponent();

	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	void Regenerate(int cellCount);
	void RenderDirty(void) { this->redrawMaze = true; }

	SpacialParadoxMaze* GetMaze() { return this->maze; }

	void SetRenderTraveler(SpacialParadoxMaze::Traveler* givenRenderTraveler) { this->renderTraveler = givenRenderTraveler; }
	SpacialParadoxMaze::Traveler* GetRenderTraveler() { return this->renderTraveler; }

private:

	void GenerateRenderWall(SpacialParadoxMaze::Wall* wall);

	class RenderWall
	{
	public:
		RenderWall();
		virtual ~RenderWall();

		void DebugRender(void) const;

		static void DrawDebugLineTransformed(const FTransform& localToWorld, const FVector& pointA, const FVector& pointB, const FColor& color);

		float uScale;
		float vScale;
		FTransform localToWorld;
	};

	typedef TDoubleLinkedList<RenderWall> RenderWallList;

	SpacialParadoxMaze* maze;
	mutable RenderWallList renderWallList;
	FCriticalSection renderWallListMutex;
	bool redrawMaze;
	bool debugDraw;
	float wallHeight;
	SpacialParadoxMaze::Traveler* renderTraveler;	// Rendering is performed from the perspective of this traveler.
};