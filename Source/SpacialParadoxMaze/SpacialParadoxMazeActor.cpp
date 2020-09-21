#include "SpacialParadoxMazeActor.h"
#include "SpacialParadoxMazeComponent.h"
#include "Components/InputComponent.h"
#include "Camera/CameraComponent.h"
#include "Engine/Engine.h"
#include "Math/RandomStream.h"

ASpacialParadoxMazeActor::ASpacialParadoxMazeActor()
{
	this->traveler = nullptr;

	this->mazeComponent = CreateDefaultSubobject<USpacialParadoxMazeComponent>(TEXT("SpacialParadoxMazeComponent"));
	this->RootComponent = mazeComponent;

	this->PrimaryActorTick.bCanEverTick = true;

	this->cameraComponent = CreateDefaultSubobject<UCameraComponent>("MazeCameraComponent");
	this->cameraComponent->AttachToComponent(this->RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
}

/*virtual*/ ASpacialParadoxMazeActor::~ASpacialParadoxMazeActor()
{
}

void ASpacialParadoxMazeActor::Regenerate(int cellCount)
{
	if (this->mazeComponent)
	{
		this->mazeComponent->Regenerate(cellCount);
		SpacialParadoxMaze* maze = this->mazeComponent->GetMaze();
		delete this->traveler;
		this->traveler = new SpacialParadoxMaze::Traveler(maze->GetFirstCell());
		this->mazeComponent->SetRenderTraveler(this->traveler);
	}
}

/*virtual*/ void ASpacialParadoxMazeActor::Tick(float DeltaTime)
{
	AActor::Tick(DeltaTime);

	if (this->mazeComponent && this->traveler)
	{
		if (!this->mazeComponent->IsRegistered())
			this->mazeComponent->RegisterComponent();

		this->mazeComponent->TickComponent(DeltaTime, ELevelTick::LEVELTICK_All, nullptr);

		SpacialParadoxMaze* maze = this->mazeComponent->GetMaze();
		this->cameraComponent->SetFieldOfView(FMath::RadiansToDegrees(this->traveler->GetViewAngle()));
		this->SetActorLocationAndRotation(this->traveler->GetViewLocation(), this->traveler->GetViewRotation());
	}
}

void ASpacialParadoxMazeActor::Move(float angleDelta, const FVector& locationDelta)
{
	if (this->mazeComponent && this->traveler)
	{
		SpacialParadoxMaze* maze = this->mazeComponent->GetMaze();
		MazeMath::Matrix3x3 matrix = this->traveler->GetViewMatrix();
		FVector xAxis = matrix.GetCol(0);
		FVector yAxis = matrix.GetCol(1);
		this->traveler->Move(angleDelta, xAxis * locationDelta.X + yAxis * locationDelta.Y);
		this->mazeComponent->RenderDirty();
	}
}