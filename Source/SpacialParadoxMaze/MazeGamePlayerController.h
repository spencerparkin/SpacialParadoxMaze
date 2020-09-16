#pragma once

#include "GameFramework/PlayerController.h"
#include "MazeGamePlayerController.generated.h"

UCLASS()
class AMazeGamePlayerController : public APlayerController
{
	GENERATED_BODY()

public:
	AMazeGamePlayerController();
	virtual ~AMazeGamePlayerController();

	void MoveLeftRight(float value);
	void MoveBackwardForward(float value);
	void LookLeftRight(float value);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Maze)
	float lookSpeed;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Maze)
	float moveSpeed;
};