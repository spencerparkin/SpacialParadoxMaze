#include "Math.h"
#include "Math/UnrealMathUtility.h"

double MazeMath::epsilon = 1e-5;

void MazeMath::PerformTests(void)
{
	Matrix3x3 matrix;
	
	FQuat quatA = FRotator(30.0f, 20.0f, -40.0f).Quaternion();
	matrix.SetFromQuat(quatA);

	FQuat quatB;
	matrix.GetToQuat(quatB);

	if (Sign(quatA.W) != Sign(quatB.W))
		quatB = quatB * -1.0;

	check((quatA - quatB).SizeSquared() < epsilon);
}

double MazeMath::Sign(double number)
{
	return (number < 0.0) ? -1.0 : 1.0;
}

double MazeMath::Squared(double number)
{
	return number * number;
}

void MazeMath::RadsToDegs(FRotator& rotator)
{
	rotator.Yaw *= 180.0f / PI;
	rotator.Pitch *= 180.0f / PI;
	rotator.Roll *= 180.0f / PI;
}

void MazeMath::RadsFromDegs(FRotator& rotator)
{
	rotator.Yaw *= PI / 180.0f;
	rotator.Pitch *= PI / 180.0f;
	rotator.Roll *= PI / 180.0f;
}

float MazeMath::WrapAngle(float angle)
{
#if 0
	while (angle >= 2.0f * pi)
		angle -= 2.0f * pi;
	while (angle < 0.0f)
		angle += 2.0f * pi;
	return angle;
#else
	return FMath::Fmod(angle, 2.0f * PI);
#endif
}

float MazeMath::WrapAngleDegrees(float angle)
{
	return FMath::Fmod(angle, 360.0f);
}

double MazeMath::AdjustAngle(double givenAngle, double targetAngle)
{
	if (givenAngle < targetAngle)
	{
		while (targetAngle - givenAngle > PI)
			givenAngle += 2.0 * PI;
	}
	else
	{
		while (givenAngle - targetAngle > PI)
			givenAngle -= 2.0 * PI;
	}

	return givenAngle;
}

FVector MazeMath::ReflectedAbout(const FVector& vector, const FVector& unitNormal)
{
	return 2.0f * FVector::DotProduct(unitNormal, vector) * unitNormal - vector;
}

FVector MazeMath::RejectedFrom(const FVector& vector, const FVector& unitNormal)
{
	return vector - ProjectedOnto(vector, unitNormal);
}

FVector MazeMath::ProjectedOnto(const FVector& vector, const FVector& unitNormal)
{
	return FVector::DotProduct(unitNormal, vector) * unitNormal;
}

FVector MazeMath::Normalized(const FVector& vector)
{
	FVector unitVector = vector;
	unitVector.Normalize();
	return unitVector;
}

FVector2D MazeMath::Normalized(const FVector2D& vector)
{
	FVector2D unitVector = vector;
	unitVector.Normalize();
	return unitVector;
}

float MazeMath::LengthOf(const FVector& vector)
{
	return FMath::Sqrt(FVector::DotProduct(vector, vector));
}

float MazeMath::LengthOf(const FVector2D& vector)
{
	return FMath::Sqrt(FVector2D::DotProduct(vector, vector));
}

FVector2D MazeMath::ComponentMultiply(const FVector2D& vectorA, const FVector2D& vectorB)
{
	return FVector2D(vectorA.X * vectorB.X, vectorA.Y * vectorB.Y);
}

FVector MazeMath::RotatePoint(const FVector& axis, float angle, const FVector& point)
{
	FVector paraComponent = ProjectedOnto(point, axis);
	FVector perpComponent = RejectedFrom(point, axis);

	FVector xAxis = Normalized(perpComponent);
	FVector yAxis = FVector::CrossProduct(xAxis, axis);

	double length = LengthOf(perpComponent);

	perpComponent = length * (xAxis * FMath::Cos(angle) + yAxis * FMath::Sin(angle));

	return paraComponent + perpComponent;
}

FVector MazeMath::RotatePoint(const FVector& origin, const FVector& axis, float angle, const FVector& point)
{
	return origin + RotatePoint(axis, angle, point - origin);
}

FVector MazeMath::Slerp(const FVector& vectorA, const FVector& vectorB, float alpha)
{
	float theta = AngleBetween(vectorA, vectorB);
	FVector result;

	if (theta < epsilon)
		result = vectorA;
	else if (theta > PI - epsilon)
		result = vectorB;
	else
		result = (FMath::Sin((1.0f - alpha) * theta) * vectorA + FMath::Sin(alpha * theta) * vectorB) / FMath::Sin(theta);

	if (IsBogus(result))
		result = vectorB;

	return result;
}

float MazeMath::AngleBetween(const FVector& vectorA, const FVector& vectorB)
{
	return FMath::Acos(FVector::DotProduct(Normalized(vectorA), Normalized(vectorB)));
}

bool MazeMath::IsBogus(const FVector& vector)
{
	if (FMath::IsNaN(vector.X) || FMath::IsNaN(vector.Y) || FMath::IsNaN(vector.Z))
		return true;

	if (!FMath::IsFinite(vector.X) || !FMath::IsFinite(vector.Y) || !FMath::IsFinite(vector.Z))
		return true;

	return false;
}

FVector2D MazeMath::Convert(const FVector& vector)
{
	return FVector2D(vector.X, vector.Y);
}

FVector MazeMath::Convert(const FVector2D& vector)
{
	return FVector(vector.X, vector.Y, 0.0f);
}

MazeMath::Matrix3x3::Matrix3x3()
{
	this->SetIdentity();
}

MazeMath::Matrix3x3::~Matrix3x3()
{
}

void MazeMath::Matrix3x3::SetCopy(const Matrix3x3& matrix)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			this->ele[i][j] = matrix.ele[i][j];
}

void MazeMath::Matrix3x3::GetCopy(Matrix3x3& matrix) const
{
	matrix.SetCopy(*this);
}

void MazeMath::Matrix3x3::SetIdentity(void)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			this->ele[i][j] = (i == j) ? 1.0 : 0.0;
}

void MazeMath::Matrix3x3::SetRow(int i, const FVector& vector)
{
	check(0 <= i && i < 3);
	this->ele[i][0] = vector.X;
	this->ele[i][1] = vector.Y;
	this->ele[i][2] = vector.Z;
}

void MazeMath::Matrix3x3::SetCol(int j, const FVector& vector)
{
	check(0 <= j && j < 3);
	this->ele[0][j] = vector.X;
	this->ele[1][j] = vector.Y;
	this->ele[2][j] = vector.Z;
}

FVector MazeMath::Matrix3x3::GetRow(int i) const
{
	check(0 <= i && i < 3);
	return FVector(this->ele[i][0], this->ele[i][1], this->ele[i][2]);
}

FVector MazeMath::Matrix3x3::GetCol(int j) const
{
	check(0 <= j && j < 3);
	return FVector(this->ele[0][j], this->ele[1][j], this->ele[2][j]);
}

void MazeMath::Matrix3x3::Orthonormalize(void)
{
	// The SVD actually finds the rotation matrix that best fits, but
	// here we are just going to use Grahm-Schmidt orthornomalization.
	// TODO: Write it.
}

void MazeMath::Matrix3x3::SetFromRotator(const FRotator& rotator)
{
	FVector xAxis(1.0, 0.0, 0.0);
	FVector yAxis(0.0, 1.0, 0.0);
	FVector zAxis(0.0, 0.0, 1.0);

	xAxis = rotator.RotateVector(xAxis);
	yAxis = rotator.RotateVector(yAxis);
	zAxis = rotator.RotateVector(zAxis);

	this->SetCol(0, xAxis);
	this->SetCol(1, yAxis);
	this->SetCol(2, zAxis);
}

bool MazeMath::Matrix3x3::GetToRotator(FRotator& rotator) const
{
	FQuat quat;
	if (!this->GetToQuat(quat))
		return false;

	rotator = quat.Rotator();
	return true;
}

void MazeMath::Matrix3x3::SetFromAxisAngle(const FVector& axis, double angle)
{
	FVector xAxis(1.0, 0.0, 0.0);
	FVector yAxis(0.0, 1.0, 0.0);
	FVector zAxis(0.0, 0.0, 1.0);

	xAxis = RotatePoint(axis, angle, xAxis);
	yAxis = RotatePoint(axis, angle, yAxis);
	zAxis = RotatePoint(axis, angle, zAxis);

	this->SetCol(0, xAxis);
	this->SetCol(1, yAxis);
	this->SetCol(2, zAxis);
}

bool MazeMath::Matrix3x3::GetToAxisAngle(FVector& axis, double& angle) const
{
	FQuat quat;
	if (!this->GetToQuat(quat))
		return false;

	axis = quat.GetRotationAxis();
	angle = quat.GetAngle();

	return true;
}

void MazeMath::Matrix3x3::SetFromQuat(const FQuat& quat)
{
	FVector xAxis(1.0, 0.0, 0.0);
	FVector yAxis(0.0, 1.0, 0.0);
	FVector zAxis(0.0, 0.0, 1.0);

	xAxis = quat.RotateVector(xAxis);
	yAxis = quat.RotateVector(yAxis);
	zAxis = quat.RotateVector(zAxis);

	this->SetCol(0, xAxis);
	this->SetCol(1, yAxis);
	this->SetCol(2, zAxis);
}

bool MazeMath::Matrix3x3::GetToQuat(FQuat& quat) const
{
	// This is Cayley's method taken from "A Survey on the Computation of Quaternions from Rotation Matrices" by Sarabandi & Thomas.

	double r11 = this->ele[0][0];
	double r21 = this->ele[1][0];
	double r31 = this->ele[2][0];

	double r12 = this->ele[0][1];
	double r22 = this->ele[1][1];
	double r32 = this->ele[2][1];

	double r13 = this->ele[0][2];
	double r23 = this->ele[1][2];
	double r33 = this->ele[2][2];

	quat.W = 0.25 * FMath::Sqrt(Squared(r11 + r22 + r33 + 1.0) + Squared(r32 - r23) + Squared(r13 - r31) + Squared(r21 - r12));
	quat.X = 0.25 * FMath::Sqrt(Squared(r32 - r23) + Squared(r11 - r22 - r33 + 1.0) + Squared(r21 + r12) + Squared(r31 + r13)) * Sign(r32 - r23);
	quat.Y = 0.25 * FMath::Sqrt(Squared(r13 - r31) + Squared(r21 + r12) + Squared(r22 - r11 - r33 + 1.0) + Squared(r32 + r23)) * Sign(r13 - r31);
	quat.Z = 0.25 * FMath::Sqrt(Squared(r21 - r12) + Squared(r31 + r13) + Squared(r32 + r23) + Squared(r33 - r11 - r22 + 1.0)) * Sign(r21 - r12);

	return true;
}

bool MazeMath::Matrix3x3::SetInverse(const Matrix3x3& matrix)
{
	return matrix.GetInverse(*this);
}

bool MazeMath::Matrix3x3::GetInverse(Matrix3x3& matrix) const
{
	double det = this->Determinant();
	double scale = 1.0f / det;
	if (scale != scale || !FMath::IsFinite(scale) || FMath::IsNaN(scale))
		return false;

	matrix.ele[0][0] = this->ele[1][1] * this->ele[2][2] - this->ele[2][1] * this->ele[1][2];
	matrix.ele[0][1] = this->ele[0][2] * this->ele[2][1] - this->ele[2][2] * this->ele[0][1];
	matrix.ele[0][2] = this->ele[0][1] * this->ele[1][2] - this->ele[1][1] * this->ele[0][2];

	matrix.ele[1][0] = this->ele[1][2] * this->ele[2][0] - this->ele[2][2] * this->ele[1][0];
	matrix.ele[1][1] = this->ele[0][0] * this->ele[2][2] - this->ele[2][0] * this->ele[0][2];
	matrix.ele[1][2] = this->ele[0][2] * this->ele[1][0] - this->ele[1][2] * this->ele[0][0];

	matrix.ele[2][0] = this->ele[1][0] * this->ele[2][1] - this->ele[2][0] * this->ele[1][1];
	matrix.ele[2][1] = this->ele[0][1] * this->ele[2][0] - this->ele[2][1] * this->ele[0][0];
	matrix.ele[2][2] = this->ele[0][0] * this->ele[1][1] - this->ele[1][0] * this->ele[0][1];

	matrix.Scale(scale);

	return true;
}

void MazeMath::Matrix3x3::SetTranspose(const Matrix3x3& matrix)
{
	matrix.GetTranspose(*this);
}

void MazeMath::Matrix3x3::GetTranspose(Matrix3x3& matrix) const
{
	check(this != &matrix);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			matrix.ele[i][j] = this->ele[j][i];
}

void MazeMath::Matrix3x3::SetProduct(const Matrix3x3& matrixA, const Matrix3x3& matrixB)
{
	FVector row[3], col[3];
	for (int i = 0; i < 3; i++)
	{
		row[i] = matrixA.GetRow(i);
		col[i] = matrixB.GetCol(i);
	}

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			this->ele[i][j] = FVector::DotProduct(row[i], col[j]);
}

void MazeMath::Matrix3x3::MultiplyLeft(const FVector& inVector, FVector& outVector) const
{
	FVector col[3];
	for (int j = 0; j < 3; j++)
		col[j] = this->GetCol(j);

	outVector.X = FVector::DotProduct(inVector, col[0]);
	outVector.Y = FVector::DotProduct(inVector, col[1]);
	outVector.Z = FVector::DotProduct(inVector, col[2]);
}

void MazeMath::Matrix3x3::MultiplyRight(const FVector& inVector, FVector& outVector) const
{
	FVector row[3];
	for (int i = 0; i < 3; i++)
		row[i] = this->GetRow(i);

	outVector.X = FVector::DotProduct(inVector, row[0]);
	outVector.Y = FVector::DotProduct(inVector, row[1]);
	outVector.Z = FVector::DotProduct(inVector, row[2]);
}

double MazeMath::Matrix3x3::Determinant(void) const
{
	return
		+ this->ele[0][0] * (this->ele[1][1] * this->ele[2][2] - this->ele[2][1] * this->ele[1][2])
		- this->ele[0][1] * (this->ele[1][0] * this->ele[2][2] - this->ele[2][0] * this->ele[1][2])
		+ this->ele[0][2] * (this->ele[1][0] * this->ele[2][1] - this->ele[2][0] * this->ele[1][1]);
}

double MazeMath::Matrix3x3::Trace(void) const
{
	return this->ele[0][0] + this->ele[1][1] + this->ele[2][2];
}

void MazeMath::Matrix3x3::Scale(double scale)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			this->ele[i][j] *= scale;
}

MazeMath::QuadraticPolynomial::QuadraticPolynomial()
{
	this->A = 1.0;
	this->B = 1.0;
	this->C = 1.0;
}

MazeMath::QuadraticPolynomial::QuadraticPolynomial(double givenA, double givenB, double givenC)
{
	this->A = givenA;
	this->B = givenB;
	this->C = givenC;
}

/*virtual*/ MazeMath::QuadraticPolynomial::~QuadraticPolynomial()
{
}

void MazeMath::QuadraticPolynomial::Evaluate(double x, double& y) const
{
	y = this->Evaluate(x);
}

double MazeMath::QuadraticPolynomial::Evaluate(double x) const
{
	return this->A * x * x + this->B * x + this->C;
}

bool MazeMath::QuadraticPolynomial::Interpolate(const FVector2D& pointA, const FVector2D& pointB, const FVector2D& pointC)
{
	Matrix3x3 vandermondeMatrix;

	vandermondeMatrix.ele[0][0] = pointA.X * pointA.X;
	vandermondeMatrix.ele[0][1] = pointA.X;
	vandermondeMatrix.ele[0][2] = 1.0f;

	vandermondeMatrix.ele[1][0] = pointB.X * pointB.X;
	vandermondeMatrix.ele[1][1] = pointB.X;
	vandermondeMatrix.ele[1][2] = 1.0f;

	vandermondeMatrix.ele[2][0] = pointC.X * pointC.X;
	vandermondeMatrix.ele[2][1] = pointC.X;
	vandermondeMatrix.ele[2][2] = 1.0f;

	Matrix3x3 vandermondeMatrixInverse;
	if (!vandermondeMatrix.GetInverse(vandermondeMatrixInverse))
		return false;

	FVector vector(pointA.Y, pointB.Y, pointC.Y);
	FVector coeficientVector;
	vandermondeMatrixInverse.MultiplyRight(vector, coeficientVector);

	this->A = coeficientVector.X;
	this->B = coeficientVector.Y;
	this->C = coeficientVector.Z;

	//static double eps = 1e-3;
	//check(FMath::Abs(this->Evaluate(pointA.X) - pointA.Y) < eps);
	//check(FMath::Abs(this->Evaluate(pointB.X) - pointB.Y) < eps);
	//check(FMath::Abs(this->Evaluate(pointC.X) - pointC.Y) < eps);

	return true;
}

// In this case we assume that the constant of the polynomial is already known and here we solve for just A and B.
bool MazeMath::QuadraticPolynomial::InterpolateWithConstantKnown(const FVector2D& pointA, const FVector2D& pointB, double givenC)
{
	this->C = givenC;

	double u = pointA.Y - this->C;
	double v = pointB.Y - this->C;

	double r = pointA.X * (pointA.X - pointB.X);
	double s = pointB.X * (pointB.X - pointA.X);

	this->A = u / r + v / s;
	this->B = (-pointB.X * u) / r + (-pointA.X * v) / s;

	return !FMath::IsNaN(this->A) && !FMath::IsNaN(this->B) && FMath::IsFinite(this->A) && FMath::IsFinite(this->B);
}

MazeMath::QuadraticSpaceCurve::QuadraticSpaceCurve()
{
}

MazeMath::QuadraticSpaceCurve::QuadraticSpaceCurve(const FVector& givenA, const FVector& givenB, const FVector& givenC)
{
	this->A = givenA;
	this->B = givenB;
	this->C = givenC;
}

/*virtual*/ MazeMath::QuadraticSpaceCurve::~QuadraticSpaceCurve()
{
}

FVector MazeMath::QuadraticSpaceCurve::Evaluate(double t) const
{
	return this->A * t * t + this->B * t + this->C;
}

bool MazeMath::QuadraticSpaceCurve::Interpolate(double tA, const FVector& pointA, double tB, const FVector& pointB, double tC, const FVector& pointC)
{
	QuadraticPolynomial polyX, polyY, polyZ;

	if (!polyX.Interpolate(FVector2D(tA, pointA.X), FVector2D(tB, pointB.X), FVector2D(tC, pointC.X)))
		return false;

	if (!polyY.Interpolate(FVector2D(tA, pointA.Y), FVector2D(tB, pointB.Y), FVector2D(tC, pointC.Y)))
		return false;

	if (!polyZ.Interpolate(FVector2D(tA, pointA.Z), FVector2D(tB, pointB.Z), FVector2D(tC, pointC.Z)))
		return false;

	this->A.X = polyX.A;
	this->B.X = polyX.B;
	this->C.X = polyX.C;

	this->A.Y = polyY.A;
	this->B.Y = polyY.B;
	this->C.Y = polyY.C;

	this->A.Z = polyZ.A;
	this->B.Z = polyZ.B;
	this->C.Z = polyZ.C;

	/*
	static double eps = 1e-3;
	check(MazeMath::LengthOf(this->Evaluate(tA) - pointA) < eps);
	check(MazeMath::LengthOf(this->Evaluate(tB) - pointB) < eps);
	check(MazeMath::LengthOf(this->Evaluate(tC) - pointC) < eps);
	*/

	return true;
}

bool MazeMath::QuadraticSpaceCurve::InterpolateWithConstantKnown(double tA, const FVector& pointA, double tB, const FVector& pointB, const FVector& givenC)
{
	QuadraticPolynomial polyX, polyY, polyZ;

	if (!polyX.InterpolateWithConstantKnown(FVector2D(tA, pointA.X), FVector2D(tB, pointB.X), givenC.X))
		return false;

	if (!polyY.InterpolateWithConstantKnown(FVector2D(tA, pointA.Y), FVector2D(tB, pointB.Y), givenC.Y))
		return false;

	if (!polyZ.InterpolateWithConstantKnown(FVector2D(tA, pointA.Z), FVector2D(tB, pointB.Z), givenC.Z))
		return false;

	this->A.X = polyX.A;
	this->B.X = polyX.B;
	this->C.X = polyX.C;

	this->A.Y = polyY.A;
	this->B.Y = polyY.B;
	this->C.Y = polyY.C;

	this->A.Z = polyZ.A;
	this->B.Z = polyZ.B;
	this->C.Z = polyZ.C;

	return true;
}