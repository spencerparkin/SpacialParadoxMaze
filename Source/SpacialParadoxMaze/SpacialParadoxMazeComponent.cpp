#include "SpacialParadoxMazeComponent.h"
#include "Math.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "DrawDebugHelpers.h"
#include "SpacialParadoxMazeSceneProxy.h"

//------------------------ USpacialParadoxMazeComponent ------------------------

USpacialParadoxMazeComponent::USpacialParadoxMazeComponent()
{
	this->debugDraw = true;
	this->redrawMaze = true;
	this->renderTraveler = nullptr;
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

/*virtual*/ FPrimitiveSceneProxy* USpacialParadoxMazeComponent::CreateSceneProxy()
{
	SpacialParadoxMazeSceneProxy* proxy = new SpacialParadoxMazeSceneProxy(this);
	return proxy;
}

/*virtual*/ FBoxSphereBounds USpacialParadoxMazeComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	FBoxSphereBounds boxSphereBounds(FSphere(FVector(0.0f, 0.0f, 0.0f), 10000.0f));
	return boxSphereBounds;
}

/*virtual*/ void USpacialParadoxMazeComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	if (this->redrawMaze && this->renderTraveler)
	{
		this->redrawMaze = false;
		FScopeLock scopeLock(&this->renderWallListMutex);
		this->renderWallList.Empty();
		this->maze->ForAllSides(*this->renderTraveler, [&](SpacialParadoxMaze::Side* side, bool visible) {
			if (visible && side->IsWall())
				this->GenerateRenderWall((SpacialParadoxMaze::Wall*)side);
		});
	}

	if (this->debugDraw)
	{
		RenderWallList::TDoubleLinkedListNode* node = this->renderWallList.GetHead();
		while (node)
		{
			const RenderWall& renderWall = node->GetValue();
			renderWall.DebugRender();
			node = node->GetNextNode();
		}
	}
}

void USpacialParadoxMazeComponent::GenerateRenderWall(SpacialParadoxMaze::Wall* wall)
{
	RenderWall renderWall;

	FVector xAxis = -MazeMath::Normalized(wall->Edge());
	FVector zAxis = -wall->Normal();
	FVector yAxis = MazeMath::Normalized(FVector::CrossProduct(zAxis, xAxis));

	MazeMath::Matrix3x3 matrix;
	matrix.SetCol(0, xAxis);
	matrix.SetCol(1, yAxis);
	matrix.SetCol(2, zAxis);

	FQuat quat;
	matrix.GetToQuat(quat);

	renderWall.localToWorld.SetRotation(quat);
	renderWall.localToWorld.SetTranslation(wall->point[1] + FVector(0.0f, 0.0f, -this->wallHeight / 2.0f));
	renderWall.localToWorld.SetScale3D(FVector(MazeMath::LengthOf(wall->Edge()), this->wallHeight, 1.0f));

	renderWall.uScale = MazeMath::LengthOf(wall->Edge()) / 10.0f;
	renderWall.vScale = this->wallHeight / 10.0f;

	this->renderWallList.AddTail(renderWall);
}

/*


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
			
			materialDynamic->SetVectorParameterValue(TEXT("UVScale"), FLinearColor(FVector(uScale, vScale, 0.0f)));
		}
	}
			*/

//------------------------ USpacialParadoxMazeComponent::RenderWall ------------------------

USpacialParadoxMazeComponent::RenderWall::RenderWall()
{
}

/*virtual*/ USpacialParadoxMazeComponent::RenderWall::~RenderWall()
{
}

void USpacialParadoxMazeComponent::RenderWall::DebugRender(void) const
{
	FColor color = FColor::Red;

	this->DrawDebugLineTransformed(localToWorld, FVector(0.0f, 0.0f, 0.0f), FVector(1.0f, 0.0f, 0.0f), color);
	this->DrawDebugLineTransformed(localToWorld, FVector(1.0f, 0.0f, 0.0f), FVector(1.0f, 1.0f, 0.0f), color);
	this->DrawDebugLineTransformed(localToWorld, FVector(1.0f, 1.0f, 0.0f), FVector(0.0f, 1.0f, 0.0f), color);
	this->DrawDebugLineTransformed(localToWorld, FVector(0.0f, 1.0f, 0.0f), FVector(0.0f, 0.0f, 0.0f), color);
	this->DrawDebugLineTransformed(localToWorld, FVector(0.0f, 0.0f, 0.0f), FVector(1.0f, 1.0f, 0.0f), color);
	this->DrawDebugLineTransformed(localToWorld, FVector(0.0f, 1.0f, 0.0f), FVector(1.0f, 0.0f, 0.0f), color);
}

/*static*/ void USpacialParadoxMazeComponent::RenderWall::DrawDebugLineTransformed(const FTransform& localToWorld, const FVector& pointA, const FVector& pointB, const FColor& color)
{
	FVector worldPointA = localToWorld.TransformPosition(pointA);
	FVector worldPointB = localToWorld.TransformPosition(pointB);

	DrawDebugLine(GWorld, worldPointA, worldPointB, color);
}