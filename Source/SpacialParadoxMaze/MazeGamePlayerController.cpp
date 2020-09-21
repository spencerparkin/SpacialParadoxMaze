#include "MazeGamePlayerController.h"
#include "SpacialParadoxMazeActor.h"

AMazeGamePlayerController::AMazeGamePlayerController()
{
	this->lookSpeed = 2.0f;
	this->moveSpeed = 40.0f;
	this->lastDeltaTime = 0.0f;

	this->AutoReceiveInput = EAutoReceiveInput::Player0;
	this->PrimaryActorTick.bCanEverTick = true;

	this->InputComponent = CreateDefaultSubobject<UInputComponent>(TEXT("MazeInputComponent"));
	this->InputComponent->BindAction("MazeExit", IE_Pressed, this, &AMazeGamePlayerController::ExitGame);
	this->InputComponent->BindAxis("MazeMoveLeftRight", this, &AMazeGamePlayerController::MoveLeftRight);
	this->InputComponent->BindAxis("MazeMoveBackwardForward", this, &AMazeGamePlayerController::MoveBackwardForward);
	this->InputComponent->BindAxis("MazeLookLeftRight", this, &AMazeGamePlayerController::LookLeftRight);
}

/*virtual*/ AMazeGamePlayerController::~AMazeGamePlayerController()
{
}

/*virtual*/ void AMazeGamePlayerController::Tick(float DeltaTime)
{
	APlayerController::Tick(DeltaTime);

	this->lastDeltaTime = DeltaTime;
}

void AMazeGamePlayerController::MoveLeftRight(float value)
{
	ASpacialParadoxMazeActor* mazeActor = Cast<ASpacialParadoxMazeActor>(this->GetViewTarget());
	if (mazeActor)
		mazeActor->Move(0.0f, FVector(0.0f, value * this->moveSpeed * this->lastDeltaTime, 0.0f));
}

void AMazeGamePlayerController::MoveBackwardForward(float value)
{
	ASpacialParadoxMazeActor* mazeActor = Cast<ASpacialParadoxMazeActor>(this->GetViewTarget());
	if (mazeActor)
		mazeActor->Move(0.0f, FVector(value * this->moveSpeed * this->lastDeltaTime, 0.0f, 0.0f));
}

void AMazeGamePlayerController::LookLeftRight(float value)
{
	ASpacialParadoxMazeActor* mazeActor = Cast<ASpacialParadoxMazeActor>(this->GetViewTarget());
	if (mazeActor)
		mazeActor->Move(value * this->lookSpeed * this->lastDeltaTime, FVector(0.0f, 0.0f, 0.0f));
}

void AMazeGamePlayerController::ExitGame(void)
{
	FPlatformMisc::RequestExit(true);
}