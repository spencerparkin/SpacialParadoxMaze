#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SpacialParadoxMaze.h"
#include "SpacialParadoxMazeActor.generated.h"

class USpacialParadoxMazeComponent;
class UCameraComponent;

UCLASS()
class ASpacialParadoxMazeActor : public AActor
{
	GENERATED_BODY()

public:
	ASpacialParadoxMazeActor();
	virtual ~ASpacialParadoxMazeActor();

	UFUNCTION(BlueprintCallable, Category = "Maze")
	void Regenerate(int cellCount);

	virtual void Tick(float DeltaTime) override;

	void Move(float angleDelta, const FVector& locationDelta);

	UPROPERTY()
	USpacialParadoxMazeComponent* mazeComponent;

	UPROPERTY()
	UCameraComponent* cameraComponent;

	SpacialParadoxMaze::Traveler* traveler;
};