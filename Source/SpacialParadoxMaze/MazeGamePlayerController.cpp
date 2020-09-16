#include "MazeGamePlayerController.h"
#include "SpacialParadoxMazeActor.h"

AMazeGamePlayerController::AMazeGamePlayerController()
{
	this->lookSpeed = 0.02f;
	this->moveSpeed = 0.2f;

	this->AutoReceiveInput = EAutoReceiveInput::Player0;

	this->InputComponent = CreateDefaultSubobject<UInputComponent>(TEXT("MazeInputComponent"));
	this->InputComponent->BindAxis("MazeMoveLeftRight", this, &AMazeGamePlayerController::MoveLeftRight);
	this->InputComponent->BindAxis("MazeMoveBackwardForward", this, &AMazeGamePlayerController::MoveBackwardForward);
	this->InputComponent->BindAxis("MazeLookLeftRight", this, &AMazeGamePlayerController::LookLeftRight);
}

/*virtual*/ AMazeGamePlayerController::~AMazeGamePlayerController()
{
}

void AMazeGamePlayerController::MoveLeftRight(float value)
{
	ASpacialParadoxMazeActor* mazeActor = Cast<ASpacialParadoxMazeActor>(this->GetViewTarget());
	if (mazeActor)
		mazeActor->Move(0.0f, FVector(0.0f, value * this->moveSpeed, 0.0f));
}

void AMazeGamePlayerController::MoveBackwardForward(float value)
{
	ASpacialParadoxMazeActor* mazeActor = Cast<ASpacialParadoxMazeActor>(this->GetViewTarget());
	if (mazeActor)
		mazeActor->Move(0.0f, FVector(value * this->moveSpeed, 0.0f, 0.0f));
}

void AMazeGamePlayerController::LookLeftRight(float value)
{
	ASpacialParadoxMazeActor* mazeActor = Cast<ASpacialParadoxMazeActor>(this->GetViewTarget());
	if (mazeActor)
		mazeActor->Move(value * this->lookSpeed, FVector(0.0f, 0.0f, 0.0f));
}