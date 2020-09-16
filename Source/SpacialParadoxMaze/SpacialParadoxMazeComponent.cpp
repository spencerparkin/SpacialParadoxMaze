#include "SpacialParadoxMazeComponent.h"
#include "Math.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "DrawDebugHelpers.h"

USpacialParadoxMazeComponent::USpacialParadoxMazeComponent()
{
	this->debugDraw = false;
	this->renderTraveler = nullptr;
	this->maxWallMeshCount = 30;
	this->wallHeight = 8.0f;
	this->maze = new SpacialParadoxMaze();
}

/*virtual*/ USpacialParadoxMazeComponent::~USpacialParadoxMazeComponent()
{
	delete this->maze;
}

void USpacialParadoxMazeComponent::Regenerate(int cellCount)
{
	FRandomStream randomStream;
	this->maze->MakeRandom(cellCount, randomStream);
}

/*virtual*/ void USpacialParadoxMazeComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	if (this->wallMeshComponentArray.Num() == 0)
	{
		FString wallMeshURL = "StaticMesh'/Game/BrickWall.BrickWall'";
		for (int i = 0; i < this->maxWallMeshCount; i++)
		{
			FString subobjectName = FString::Format(TEXT("WallMeshCompoment{0}"), { i });
			UStaticMeshComponent* wallMeshComponent = ::NewObject<UStaticMeshComponent>(this, *subobjectName);
			this->wallMeshComponentArray.Add(wallMeshComponent);
			UStaticMesh* wallMesh = Cast<UStaticMesh>(::StaticLoadObject(UStaticMesh::StaticClass(), this, *wallMeshURL));
			wallMeshComponent->SetStaticMesh(wallMesh);
			wallMeshComponent->RegisterComponent();
		}
	}

	int i = 0;

	if (this->renderTraveler)
	{
		this->maze->ForAllSides(*this->renderTraveler, [&](SpacialParadoxMaze::Side* side, bool visible) {
			if (visible && side->IsWall())
				this->ShowWall((SpacialParadoxMaze::Wall*)side, i);
		});
	}

	while (i < this->wallMeshComponentArray.Num())
		this->wallMeshComponentArray[i++]->SetVisibility(false);
}

void USpacialParadoxMazeComponent::ShowWall(SpacialParadoxMaze::Wall* wall, int& i)
{
	if(i < this->wallMeshComponentArray.Num())
	{
		UStaticMeshComponent* wallMeshComponent = this->wallMeshComponentArray[i++];
		wallMeshComponent->SetVisibility(!this->debugDraw);

		FVector xAxis = -MazeMath::Normalized(wall->Edge());
		FVector zAxis = -wall->Normal();
		FVector yAxis = MazeMath::Normalized(FVector::CrossProduct(zAxis, xAxis));

		MazeMath::Matrix3x3 matrix;
		matrix.SetCol(0, xAxis);
		matrix.SetCol(1, yAxis);
		matrix.SetCol(2, zAxis);

		FQuat quat;
		matrix.GetToQuat(quat);

		FTransform localToWorld;
		localToWorld.SetRotation(quat);
		localToWorld.SetTranslation(wall->point[1] + FVector(0.0f, 0.0f, -this->wallHeight / 2.0f));
		localToWorld.SetScale3D(FVector(MazeMath::LengthOf(wall->Edge()), this->wallHeight, 1.0f));

		wallMeshComponent->SetWorldTransform(localToWorld);

		int j = wallMeshComponent->GetMaterialIndex("WallSurface");
		if (j >= 0)
		{
			UMaterialInterface* materialInterface = wallMeshComponent->GetMaterial(j);
			UMaterialInstanceDynamic* materialDynamic = Cast<UMaterialInstanceDynamic>(materialInterface);
			if (!materialDynamic)
			{
				materialDynamic = UMaterialInstanceDynamic::Create(materialInterface, wallMeshComponent);
				wallMeshComponent->SetMaterial(j, materialDynamic);
			}

			if (materialDynamic)
			{
				float uScale = MazeMath::LengthOf(wall->Edge()) / 10.0f;
				float vScale = this->wallHeight / 10.0f;
				materialDynamic->SetVectorParameterValue(TEXT("UVScale"), FLinearColor(FVector(uScale, vScale, 0.0f)));
			}
		}

		if (!this->debugDraw)
			wallMeshComponent->MarkRenderStateDirty();
		else
		{
			FColor color = FColor::Red;
			this->DrawDebugLineTransformed(localToWorld, FVector(0.0f, 0.0f, 0.0f), FVector(1.0f, 0.0f, 0.0f), color);
			this->DrawDebugLineTransformed(localToWorld, FVector(1.0f, 0.0f, 0.0f), FVector(1.0f, 1.0f, 0.0f), color);
			this->DrawDebugLineTransformed(localToWorld, FVector(1.0f, 1.0f, 0.0f), FVector(0.0f, 1.0f, 0.0f), color);
			this->DrawDebugLineTransformed(localToWorld, FVector(0.0f, 1.0f, 0.0f), FVector(0.0f, 0.0f, 0.0f), color);
			this->DrawDebugLineTransformed(localToWorld, FVector(0.0f, 0.0f, 0.0f), FVector(1.0f, 1.0f, 0.0f), color);
			this->DrawDebugLineTransformed(localToWorld, FVector(0.0f, 1.0f, 0.0f), FVector(1.0f, 0.0f, 0.0f), color);
		}
	}
}

/*static*/ void USpacialParadoxMazeComponent::DrawDebugLineTransformed(const FTransform& localToWorld, const FVector& pointA, const FVector& pointB, const FColor& color)
{
	FVector worldPointA = localToWorld.TransformPosition(pointA);
	FVector worldPointB = localToWorld.TransformPosition(pointB);

	DrawDebugLine(GWorld, worldPointA, worldPointB, color);
}