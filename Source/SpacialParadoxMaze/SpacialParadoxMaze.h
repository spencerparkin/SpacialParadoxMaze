#pragma once

#include "CoreMinimal.h"
#include "Math.h"

class SpacialParadoxMaze
{
public:
	SpacialParadoxMaze();
	virtual ~SpacialParadoxMaze(void);

	class ConvexCell;
	class Side;
	class Traveler;
	typedef TDoubleLinkedList<ConvexCell*> ConvexCellList;

	void Clear(void);
	void MakeRandom(int numCells, FRandomStream& randomStream, int numLoops = 0);
	ConvexCell* GetRandomCell(FRandomStream& randomStream, ConvexCellList::TDoubleLinkedListNode** chosenNodePtr = nullptr);
	ConvexCell* GetFirstCell();
	void ForAllSides(const Traveler& traveler, TFunctionRef<void(Side*, bool)> callback);

	struct FieldOfVision
	{
		FVector location;
		FVector direction;
		float angle;
	};

	class Side
	{
	public:
		Side(ConvexCell* givenOwningCell);
		virtual ~Side();

		virtual bool IsWall() const = 0;
		virtual Side* New() const = 0;
		virtual Side* Clone() const;

		float Length() const;
		float Angle() const;
		FVector Edge() const;
		FVector Normal() const;
		FVector MidPoint() const;
		FPlane Plane() const;
		void Transform(const FTransform& transform);
		FieldOfVision ClipFovi(const FieldOfVision& fovi) const;
		bool CanBeSeenByFovi(const FieldOfVision& fovi) const;
		float RayCast(const FVector& origin, const FVector& unitDirection) const;
		float DistanceToPoint(const FVector& givenPoint) const;

		FVector point[2];
		ConvexCell* owningCell;
	};

	class Wall : public Side
	{
	public:
		Wall(ConvexCell* givenOwningCell);
		virtual ~Wall();

		virtual bool IsWall() const override { return true; }
		virtual Side* New() const override { return new Wall(nullptr); }
		virtual Side* Clone() const override;
	};

	class Portal : public Side
	{
	public:
		Portal(ConvexCell* givenOwningCell);
		virtual ~Portal();

		virtual bool IsWall() const override { return false; }
		virtual Side* New() const override { return new Portal(nullptr); }
		virtual Side* Clone() const override;

		FTransform CalcRelativeTransform() const;
		FTransform CalcEdgeTransform() const;

		Portal* adjacentPortal;
	};

	class ConvexCell
	{
	public:
		ConvexCell();
		virtual ~ConvexCell();

		typedef TDoubleLinkedList<Side*> SideList;

		void Clear(void);
		void MakeRegular(float radius, int numSides);
		ConvexCell* FindGroupRep();
		Wall* FindIsolatedWall();
		Wall* TrisectWall(Wall* wall, float centerLength);
		SideList::TDoubleLinkedListNode* FindNodeForSide(Side* side);
		bool ReplaceSide(Side* oldSide, Side* newSide);
		void ForAllSides(const FieldOfVision& fovi, const FTransform& localToWorld, TFunctionRef<void(Side*, bool)> callback, int depth = 0, int maxDepth = 7) const;

		SideList sideList;
		ConvexCell* parentCell;
		int rank;
	};

	class Traveler
	{
	public:
		Traveler(ConvexCell* givenContainingCell);
		virtual ~Traveler();

		void Move(float angleDelta, const FVector& locationDelta);

		float GetViewAngle() const;
		FVector GetViewLocation() const;
		FQuat GetViewRotation() const;
		MazeMath::Matrix3x3 GetViewMatrix() const;
		const FieldOfVision& GetFovi() const { return this->fovi; }
		const ConvexCell* GetContainingCell() const { return this->containingCell; }
		bool CanSeeOtherTraveler(const Traveler* traveler) const;

	private:
		FieldOfVision fovi;
		ConvexCell* containingCell;
		float collisionRadius;
	};

private:

	ConvexCellList convexCellList;
};