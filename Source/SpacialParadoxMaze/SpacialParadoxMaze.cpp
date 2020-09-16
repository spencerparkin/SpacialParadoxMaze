#include "SpacialParadoxMaze.h"
#include "Math.h"
#include "Math/UnrealMathUtility.h"
#include "Engine/World.h"

//---------------------------- SpacialParadoxMaze ----------------------------

SpacialParadoxMaze::SpacialParadoxMaze(void)
{
}

/*virtual*/ SpacialParadoxMaze::~SpacialParadoxMaze(void)
{
	this->Clear();
}

void SpacialParadoxMaze::Clear(void)
{
	while (this->convexCellList.Num() > 0)
	{
		ConvexCellList::TDoubleLinkedListNode* node = this->convexCellList.GetHead();
		delete node->GetValue();
		this->convexCellList.RemoveNode(node);
	}
}

SpacialParadoxMaze::ConvexCell* SpacialParadoxMaze::GetRandomCell(FRandomStream& randomStream, ConvexCellList::TDoubleLinkedListNode** chosenNodePtr /*= nullptr*/)
{
	int i = FMath::RoundToInt(randomStream.FRandRange(-0.5f, this->convexCellList.Num() - 0.5f));
	i = FMath::Clamp(i, 0, this->convexCellList.Num() - 1);
	ConvexCellList::TDoubleLinkedListNode* node = this->convexCellList.GetHead();
	while (i-- > 0)
		node = node->GetNextNode();
	if (chosenNodePtr)
		*chosenNodePtr = node;
	return node->GetValue();
}

SpacialParadoxMaze::ConvexCell* SpacialParadoxMaze::GetFirstCell()
{
	if (this->convexCellList.Num() == 0)
		return nullptr;

	return this->convexCellList.GetHead()->GetValue();
}

void SpacialParadoxMaze::MakeRandom(int numCells, FRandomStream& randomStream, int numLoops /*= 0*/)
{
	this->Clear();

	for (int i = 0; i < numCells; i++)
	{
		ConvexCell* cell = new ConvexCell();
		cell->MakeRegular(randomStream.FRandRange(20.0f, 30.0f), FMath::RoundToInt(randomStream.FRandRange(4.0f, 8.0f)));
		this->convexCellList.AddTail(cell);
	}

	int numGroups = numCells;
	while (numGroups > 1 || numLoops > 0)
	{
		ConvexCell* cellA = nullptr;
		ConvexCell* cellB = nullptr;

		Wall* wallA = nullptr;
		Wall* wallB = nullptr;

		ConvexCellList::TDoubleLinkedListNode* chosenNodeA = nullptr;
		ConvexCellList::TDoubleLinkedListNode* chosenNodeB = nullptr;

		while (true)
		{
			cellA = this->GetRandomCell(randomStream, &chosenNodeA);
			cellB = this->GetRandomCell(randomStream, &chosenNodeB);

			if (numGroups > 1)
			{	
				if (cellA->FindGroupRep() == cellB->FindGroupRep())
					continue;
			}
			else
			{
				// I suppose you could have a cell with a portal into itself, but let's not allow that case for now.
				if (cellA == cellB)
					continue;
			}

			wallA = cellA->FindIsolatedWall();
			wallB = cellB->FindIsolatedWall();
			if (wallA && wallB && wallA != wallB)
				break;
		}

		float lengthA = wallA->Length();
		float lengthB = wallB->Length();

		if (lengthA > lengthB)
			wallA = cellA->TrisectWall(wallA, lengthB);
		else if (lengthB > lengthA)
			wallB = cellB->TrisectWall(wallB, lengthA);

		Portal* portalA = new Portal(cellA);
		Portal* portalB = new Portal(cellB);

		portalA->adjacentPortal = portalB;
		portalB->adjacentPortal = portalA;

		cellA->ReplaceSide(wallA, portalA);
		cellB->ReplaceSide(wallB, portalB);

		if (numGroups == 1)
			numLoops--;
		else
		{
			// For correctness, it doesn't matter which becomes the merged group representative.
			// But merging by rank here is a kind of optimization we can perform.
			if (cellA->rank < cellB->rank)
				cellA->parentCell = cellB;
			else if (cellA->rank > cellB->rank)
				cellB->parentCell = cellA;
			else
			{
				if (randomStream.FRandRange(0.0f, 1.0f) > 0.5f)
				{
					cellA->parentCell = cellB;
					cellB->rank++;
				}
				else
				{
					cellB->parentCell = cellA;
					cellA->rank++;
				}
			}

			numGroups--;
		}
	}
}

void SpacialParadoxMaze::ForAllSides(const Traveler& traveler, TFunctionRef<void(Side*, bool)> callback)
{
	FTransform identityTransform;
	identityTransform.SetIdentity();
	traveler.GetContainingCell()->ForAllSides(traveler.GetFovi(), identityTransform, callback);
}

//---------------------------- SpacialParadoxMaze::Side ----------------------------

SpacialParadoxMaze::Side::Side(ConvexCell* givenOwningCell)
{
	this->owningCell = givenOwningCell;

	point[0].Set(0.0f, 0.0f, 0.0f);
	point[1].Set(0.0f, 0.0f, 0.0f);
}

/*virtual*/ SpacialParadoxMaze::Side::~Side()
{
}

/*virtual*/ SpacialParadoxMaze::Side* SpacialParadoxMaze::Side::Clone() const
{
	Side* side = this->New();
	side->point[0] = this->point[0];
	side->point[1] = this->point[1];
	side->owningCell = this->owningCell;
	return side;
}

float SpacialParadoxMaze::Side::Length() const
{
	return MazeMath::LengthOf(this->point[1] - this->point[0]);
}

float SpacialParadoxMaze::Side::Angle() const
{
	FVector vector = MazeMath::Normalized(this->MidPoint());
	return FMath::Atan2(vector.Y, vector.X);
}

FVector SpacialParadoxMaze::Side::Edge() const
{
	return this->point[1] - this->point[0];
}

FVector SpacialParadoxMaze::Side::Normal() const
{
	FVector edge = this->Edge();
	FVector normal(edge.Y, -edge.X, edge.Z);	// Rotate CCW by 90 degrees.
	normal.Normalize();
	return normal;
}

FVector SpacialParadoxMaze::Side::MidPoint() const
{
	return (this->point[0] + this->point[1]) * 0.5f;
}

FPlane SpacialParadoxMaze::Side::Plane() const
{
	return FPlane(this->point[0], this->Normal());
}

void SpacialParadoxMaze::Side::Transform(const FTransform& transform)
{
	this->point[0] = transform.TransformPosition(this->point[0]);
	this->point[1] = transform.TransformPosition(this->point[1]);
}

float SpacialParadoxMaze::Side::RayCast(const FVector& origin, const FVector& unitDirection) const
{
	float param = 0.0f;

	FVector normal = this->Normal();
	float angle = FMath::Acos(FVector::DotProduct(normal, unitDirection));
	if (angle >= PI / 2.0f - SMALL_NUMBER)
	{
		FVector crossProduct = FVector::CrossProduct(normal, unitDirection);
		if (crossProduct.Z < 0.0f)
			param = -MAX_FLT;
		else
			param = MAX_FLT;
	}
	else
	{
		FPlane plane = this->Plane();
		FVector intersectionPoint = FMath::RayPlaneIntersection(origin, unitDirection, plane);
		FVector edgeVector = this->Edge();
		float squareMag = FVector::DotProduct(edgeVector, edgeVector);
		param = FVector::DotProduct(intersectionPoint - this->point[0], edgeVector) / squareMag;
	}

	return param;
}

SpacialParadoxMaze::FieldOfVision SpacialParadoxMaze::Side::ClipFovi(const FieldOfVision& fovi) const
{
	FieldOfVision clippedFovi;
	clippedFovi.angle = 0.0f;
	clippedFovi.location = fovi.location;
	clippedFovi.direction.Set(0.f, 0.0f, 0.0f);
	
	float angle = FMath::Acos(FVector::DotProduct(fovi.direction, this->Normal()));
	if (angle - PI / 2.0f < fovi.angle)
	{
		FVector vectorA = MazeMath::RotatePoint(FVector(0.0f, 0.0f, -1.0f), -fovi.angle, fovi.direction);
		FVector vectorB = MazeMath::RotatePoint(FVector(0.0f, 0.0f, -1.0f), fovi.angle, fovi.direction);

		float paramA = this->RayCast(fovi.location, vectorA);
		float paramB = this->RayCast(fovi.location, vectorB);

		if (paramA > paramB)
		{
			float temp = paramA;
			paramA = paramB;
			paramB = temp;
		}

		paramA = FMath::Max(paramA, 0.0f);
		paramB = FMath::Min(paramB, 1.0f);

		if (paramA <= paramB)
		{
			FVector edgeVector = this->Edge();

			vectorA = MazeMath::Normalized((this->point[0] + paramA * edgeVector) - fovi.location);
			vectorB = MazeMath::Normalized((this->point[0] + paramB * edgeVector) - fovi.location);

			clippedFovi.angle = FMath::Acos(FVector::DotProduct(vectorA, vectorB)) / 2.0f;
			clippedFovi.direction = MazeMath::Normalized(vectorA + vectorB);
		}
	}

	return clippedFovi;
}

bool SpacialParadoxMaze::Side::CanBeSeenByFovi(const FieldOfVision& fovi) const
{
	FieldOfVision clippedFovi = this->ClipFovi(fovi);
	float eps = 1e-6f;
	return clippedFovi.angle > eps;
}

float SpacialParadoxMaze::Side::DistanceToPoint(const FVector& givenPoint) const
{
	FVector edge = this->Edge();
	float dot = FVector::DotProduct(MazeMath::Normalized(edge), givenPoint - this->point[0]);
	if (dot >= MazeMath::LengthOf(edge))
		return FVector::Dist(givenPoint, this->point[1]);
	else if (dot <= 0.0f)
		return FVector::Dist(givenPoint, this->point[0]);
	FPlane plane = this->Plane();
	return FMath::Abs(plane.PlaneDot(givenPoint));
}

//---------------------------- SpacialParadoxMaze::Wall ----------------------------

SpacialParadoxMaze::Wall::Wall(ConvexCell* givenOwningCell) : Side(givenOwningCell)
{
}

/*virtual*/ SpacialParadoxMaze::Wall::~Wall()
{
}

/*virtual*/ SpacialParadoxMaze::Side* SpacialParadoxMaze::Wall::Clone() const
{
	Wall* wall = (Wall*)Side::Clone();
	return wall;
}

//---------------------------- SpacialParadoxMaze::Portal ----------------------------

SpacialParadoxMaze::Portal::Portal(ConvexCell* givenOwningCell) : Side(givenOwningCell)
{
	this->adjacentPortal = nullptr;
}

/*virtual*/ SpacialParadoxMaze::Portal::~Portal()
{
}

/*virtual*/ SpacialParadoxMaze::Side* SpacialParadoxMaze::Portal::Clone() const
{
	Portal* portal = (Portal*)Side::Clone();
	portal->adjacentPortal = this->adjacentPortal;
	return portal;
}

FTransform SpacialParadoxMaze::Portal::CalcEdgeTransform() const
{
	MazeMath::Matrix3x3 matrix;
	matrix.SetCol(0, this->Normal());
	matrix.SetCol(1, MazeMath::Normalized(this->Edge()));
	matrix.SetCol(2, FVector::CrossProduct(matrix.GetCol(0), matrix.GetCol(1)));
	FQuat quat;
	matrix.GetToQuat(quat);
	FTransform transform(quat, this->MidPoint());
	return transform;
}

FTransform SpacialParadoxMaze::Portal::CalcRelativeTransform() const
{
	FTransform transformA = this->CalcEdgeTransform();
	FTransform transformB = this->adjacentPortal->CalcEdgeTransform();
	FQuat quat(FVector(0.0f, 0.0f, 1.0f), PI);
	FTransform rotationTransform(quat);
	FTransform relativeTransform = transformB.Inverse() * rotationTransform * transformA;
	return relativeTransform;
}

//---------------------------- SpacialParadoxMaze::ConvexCell ----------------------------

SpacialParadoxMaze::ConvexCell::ConvexCell()
{
	this->parentCell = nullptr;
	this->rank = 0;
}

/*virtual*/ SpacialParadoxMaze::ConvexCell::~ConvexCell()
{
	this->Clear();
}

void SpacialParadoxMaze::ConvexCell::Clear(void)
{
	while (this->sideList.Num() > 0)
	{
		SideList::TDoubleLinkedListNode* node = this->sideList.GetHead();
		delete node->GetValue();
		this->sideList.RemoveNode(node);
	}
}

void SpacialParadoxMaze::ConvexCell::MakeRegular(float radius, int numSides)
{
	this->Clear();

	TArray<FVector> vertexArray;
	for(int i = 0; i < numSides; i++)
	{
		float angle = 2.0f * PI * (float(i) / float(numSides));
		FVector vertex(radius * FMath::Cos(angle), radius * FMath::Sin(angle), 0.0f);
		vertexArray.Add(vertex);
	}

	for (int i = 0; i < vertexArray.Num(); i++)
	{
		Wall* wall = new Wall(this);
		wall->point[0] = vertexArray[i];
		wall->point[1] = vertexArray[(i + 1) % vertexArray.Num()];
		this->sideList.AddTail(wall);
	}
}

SpacialParadoxMaze::ConvexCell* SpacialParadoxMaze::ConvexCell::FindGroupRep()
{
	ConvexCell* groupRep = this;
	while (groupRep->parentCell)
		groupRep = groupRep->parentCell;

	// This is purely an optimization and not needed for correctness.
	ConvexCell* cell = this;
	while (cell->parentCell)
	{
		ConvexCell* nextCell = cell->parentCell;
		cell->parentCell = groupRep;
		cell = nextCell;
	}

	return groupRep;
}

SpacialParadoxMaze::Wall* SpacialParadoxMaze::ConvexCell::FindIsolatedWall()
{
	SideList::TDoubleLinkedListNode* node = this->sideList.GetHead();
	while(node)
	{
		if (node->GetValue()->IsWall())
		{
			SideList::TDoubleLinkedListNode* nextNode = node->GetNextNode() ? node->GetNextNode() : this->sideList.GetHead();
			SideList::TDoubleLinkedListNode* prevNode = node->GetPrevNode() ? node->GetPrevNode() : this->sideList.GetTail();

			if (nextNode->GetValue()->IsWall() && prevNode->GetValue()->IsWall())
				return (Wall*)node->GetValue();
		}

		node = node->GetNextNode();
	}

	return nullptr;
}

SpacialParadoxMaze::Wall* SpacialParadoxMaze::ConvexCell::TrisectWall(Wall* wall, float centerLength)
{
	if (wall->owningCell != this)
		return nullptr;

	float sideWallLength = (wall->Length() - centerLength) / 2.0f;
	if (sideWallLength <= 0.0f)
		return nullptr;

	SideList::TDoubleLinkedListNode* node = this->FindNodeForSide(wall);
	if (!node)
		return nullptr;

	FVector sideWallVector = MazeMath::Normalized(wall->Edge()) * sideWallLength;

	Wall* centerWall = new Wall(this);
	centerWall->point[0] = wall->point[0] + sideWallVector;
	centerWall->point[1] = wall->point[1] - sideWallVector;
	node->GetValue() = centerWall;

	Wall* sideWallA = new Wall(this);
	sideWallA->point[0] = wall->point[0];
	sideWallA->point[1] = centerWall->point[0];
	this->sideList.InsertNode(sideWallA, node);

	Wall* sideWallB = new Wall(this);
	sideWallB->point[0] = centerWall->point[1];
	sideWallB->point[1] = wall->point[1];
	if (node->GetNextNode())
		this->sideList.InsertNode(sideWallB, node->GetNextNode());
	else
		this->sideList.AddTail(sideWallB);

	delete wall;
	return centerWall;
}

SpacialParadoxMaze::ConvexCell::SideList::TDoubleLinkedListNode* SpacialParadoxMaze::ConvexCell::FindNodeForSide(Side* side)
{
	SideList::TDoubleLinkedListNode* node = this->sideList.GetHead();
	while (node && node->GetValue() != side)
		node = node->GetNextNode();

	return node;
}

bool SpacialParadoxMaze::ConvexCell::ReplaceSide(Side* oldSide, Side* newSide)
{
	SideList::TDoubleLinkedListNode* node = this->FindNodeForSide(oldSide);
	if (!node)
		return false;

	node->GetValue() = newSide;
	newSide->point[0] = oldSide->point[0];
	newSide->point[1] = oldSide->point[1];
	newSide->owningCell = oldSide->owningCell;
	delete oldSide;
	return true;
}

// The initially passed localToWorld transform should always be the identity matrix.
// TODO: We should really be returning clipped walls along with UV coordinates to match.  This will solve a visual bug I've encountered.
void SpacialParadoxMaze::ConvexCell::ForAllSides(const FieldOfVision& fovi, const FTransform& localToWorld, TFunctionRef<void(Side*, bool)> callback, int depth /*= 0*/, int maxDepth /*= 7*/) const
{
	if (depth < maxDepth)
	{
		for (const SideList::TDoubleLinkedListNode* node = this->sideList.GetHead(); node; node = node->GetNextNode())
		{
			const Side* localSide = node->GetValue();
			Side* worldSide = localSide->Clone();
			worldSide->Transform(localToWorld);

			bool visible = worldSide->CanBeSeenByFovi(fovi);
			callback(worldSide, visible);
			
			if (visible && !worldSide->IsWall())
			{
				const Portal* worldPortal = (const Portal*)worldSide;
				const ConvexCell* adjacentCell = worldPortal->adjacentPortal->owningCell;

				FieldOfVision clippedFovi = worldPortal->ClipFovi(fovi);

				const Portal* localPortal = (const Portal*)localSide;
				FTransform relativeTransform = localPortal->CalcRelativeTransform();
				FTransform adjacentCellLocalToWorld = relativeTransform * localToWorld;

				adjacentCell->ForAllSides(clippedFovi, adjacentCellLocalToWorld, callback, depth + 1, maxDepth);
			}

			delete worldSide;
		}
	}
}

//---------------------------- SpacialParadoxMaze::Traveler ----------------------------

SpacialParadoxMaze::Traveler::Traveler(ConvexCell* givenContainingCell)
{
	this->containingCell = givenContainingCell;
	this->fovi.direction = FVector(1.0f, 0.0f, 0.0f);
	this->fovi.angle = 0.8f;
	this->fovi.location = FVector(0.0f, 0.0f, 0.0f);
	this->collisionRadius = 2.0f;
}

/*virtual*/ SpacialParadoxMaze::Traveler::~Traveler()
{
}

void SpacialParadoxMaze::Traveler::Move(float angleDelta, const FVector& locationDelta)
{
	this->fovi.direction = MazeMath::Normalized(MazeMath::RotatePoint(FVector(0.0f, 0.0f, -1.0f), angleDelta, this->fovi.direction));

	FVector targetLocation = this->fovi.location + locationDelta;

	for (const ConvexCell::SideList::TDoubleLinkedListNode* node = this->containingCell->sideList.GetHead(); node; node = (node ? node->GetNextNode() : nullptr))
	{
		const Side* localSide = node->GetValue();
		FPlane plane = localSide->Plane();
		//check(plane.PlaneDot(this->fovi.location) < 0.0f);
		if (localSide->IsWall())
		{
			float distance = localSide->DistanceToPoint(targetLocation);
			if (distance < collisionRadius)
				targetLocation -= plane.GetSafeNormal() * (collisionRadius - distance);
		}
		else
		{
			float signedDistance = plane.PlaneDot(targetLocation);
			if (signedDistance > 0.0f)
			{
				const Portal* localPortal = (const Portal*)localSide;
				this->containingCell = localPortal->adjacentPortal->owningCell;
				FTransform relativeTransform = localPortal->CalcRelativeTransform();
				FTransform inverseRelativeTransform = relativeTransform.Inverse();
				targetLocation = inverseRelativeTransform.TransformPosition(targetLocation);
				fovi.direction = inverseRelativeTransform.TransformVector(fovi.direction);
				break;
			}
		}
	}

	this->fovi.location = targetLocation;
}

float SpacialParadoxMaze::Traveler::GetViewAngle() const
{
	return this->fovi.angle * 2.0f;
}

FVector SpacialParadoxMaze::Traveler::GetViewLocation() const
{
	return this->fovi.location;
}

MazeMath::Matrix3x3 SpacialParadoxMaze::Traveler::GetViewMatrix() const
{
	MazeMath::Matrix3x3 matrix;
	matrix.SetCol(0, this->fovi.direction);
	matrix.SetCol(2, FVector(0.0f, 0.0f, 1.0f));
	matrix.SetCol(1, FVector::CrossProduct(matrix.GetCol(2), matrix.GetCol(0)));
	return matrix;
}

FQuat SpacialParadoxMaze::Traveler::GetViewRotation() const
{
	MazeMath::Matrix3x3 matrix = this->GetViewMatrix();
	FQuat quat;
	matrix.GetToQuat(quat);
	return quat;
}

bool SpacialParadoxMaze::Traveler::CanSeeOtherTraveler(const Traveler* traveler) const
{
	// TODO: This might be an interesting feature for AI vision.
	//       Call something like the ForAllSides() method, but for cells, not sides.
	//       Maybe refactor things so that ForAllSides() and ForAllCells() share code.
	return false;
}