#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "SpacialParadoxMaze.h"
#include "SpacialParadoxMazeComponent.generated.h"

UCLASS(BlueprintType, Blueprintable)
class USpacialParadoxMazeComponent : public USceneComponent
{
	GENERATED_BODY()

public:

	USpacialParadoxMazeComponent();
	virtual ~USpacialParadoxMazeComponent();

	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	void Regenerate(int cellCount);

	SpacialParadoxMaze* GetMaze() { return this->maze; }

	void SetRenderTraveler(SpacialParadoxMaze::Traveler* givenRenderTraveler) { this->renderTraveler = givenRenderTraveler; }
	SpacialParadoxMaze::Traveler* GetRenderTraveler() { return this->renderTraveler; }

	static void DrawDebugLineTransformed(const FTransform& localToWorld, const FVector& pointA, const FVector& pointB, const FColor& color);

private:

	void ShowWall(SpacialParadoxMaze::Wall* wall, int& i);

	bool debugDraw;
	SpacialParadoxMaze* maze;
	int32 maxWallMeshCount;
	float wallHeight;
	SpacialParadoxMaze::Traveler* renderTraveler;

	// Note that I was using the UInstancedStaticMeshComponent here instead of
	// an array of static mesh components, but the only thing that can change
	// across instances is the transform.  I need to be able to change the
	// material parameters for each mesh.  Since there won't be too many copies
	// of the static mesh, I think this will be efficient enough.
	UPROPERTY()
	TArray<UStaticMeshComponent*> wallMeshComponentArray;
};