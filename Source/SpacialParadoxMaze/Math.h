#pragma once

#include "CoreMinimal.h"

namespace MazeMath
{
	extern double epsilon;

	void PerformTests(void);
	double Sign(double number);
	double Squared(double number);
	void RadsToDegs(FRotator& rotator);
	void RadsFromDegs(FRotator& rotator);
	float WrapAngle(float angle);
	float WrapAngleDegrees(float angle);
	double AdjustAngle(double givenAngle, double targetAngle);
	FVector ReflectedAbout(const FVector& vector, const FVector& unitNormal);
	FVector RejectedFrom(const FVector& vector, const FVector& unitNormal);
	FVector ProjectedOnto(const FVector& vector, const FVector& unitNormal);
	FVector Normalized(const FVector& vector);
	FVector2D Normalized(const FVector2D& vector);
	float LengthOf(const FVector& vector);
	float LengthOf(const FVector2D& vector);
	FVector2D ComponentMultiply(const FVector2D& vectorA, const FVector2D& vectorB);
	FVector RotatePoint(const FVector& axis, float angle, const FVector& point);
	FVector RotatePoint(const FVector& origin, const FVector& axis, float angle, const FVector& point);
	FVector Slerp(const FVector& vectorA, const FVector& vectorB, float alpha);
	float AngleBetween(const FVector& vectorA, const FVector& vectorB);
	bool IsBogus(const FVector& vector);
	FVector2D Convert(const FVector& vector);
	FVector Convert(const FVector2D& vector);

	class Matrix3x3
	{
	public:

		Matrix3x3();
		virtual ~Matrix3x3();

		void SetIdentity(void);

		void SetRow(int i, const FVector& vector);
		void SetCol(int j, const FVector& vector);
		FVector GetRow(int i) const;
		FVector GetCol(int j) const;

		void Orthonormalize(void);

		void SetCopy(const Matrix3x3& matrix);
		void GetCopy(Matrix3x3& matrix) const;

		void SetFromAxisAngle(const FVector& axis, double angle);
		bool GetToAxisAngle(FVector& axis, double& angle) const;

		void SetFromRotator(const FRotator& rotator);
		bool GetToRotator(FRotator& rotator) const;	// Must be orthonormal with determinant +1 for this to work.

		void SetFromQuat(const FQuat& quat);
		bool GetToQuat(FQuat& quat) const; // Must be orthonormal with determinant +1 for this to work.

		bool SetInverse(const Matrix3x3& matrix);
		bool GetInverse(Matrix3x3& matrix) const;

		void SetTranspose(const Matrix3x3& matrix);
		void GetTranspose(Matrix3x3& matrix) const;

		void SetProduct(const Matrix3x3& matrixA, const Matrix3x3& matrixB);

		void MultiplyLeft(const FVector& inVector, FVector& outVector) const;
		void MultiplyRight(const FVector& inVector, FVector& outVector) const;

		double Determinant(void) const;
		double Trace(void) const;

		void Scale(double scale);

		double ele[3][3];
	};

	class QuadraticPolynomial
	{
	public:
		QuadraticPolynomial();
		QuadraticPolynomial(double givenA, double givenB, double givenC);
		virtual ~QuadraticPolynomial();

		void Evaluate(double x, double& y) const;
		double Evaluate(double x) const;
		bool Interpolate(const FVector2D& pointA, const FVector2D& pointB, const FVector2D& pointC);
		bool InterpolateWithConstantKnown(const FVector2D& pointA, const FVector2D& pointB, double givenC);

		double A, B, C;
	};

	class QuadraticSpaceCurve
	{
	public:
		QuadraticSpaceCurve();
		QuadraticSpaceCurve(const FVector& givenA, const FVector& givenB, const FVector& givenC);
		virtual ~QuadraticSpaceCurve();

		FVector Evaluate(double t) const;
		bool Interpolate(double tA, const FVector& pointA, double tB, const FVector& pointB, double tC, const FVector& pointC);
		bool InterpolateWithConstantKnown(double tA, const FVector& pointA, double tB, const FVector& pointB, const FVector& givenC);

		FVector A, B, C;
	};
};