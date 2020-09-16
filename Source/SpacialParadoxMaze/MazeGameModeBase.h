// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "MazeGameModeBase.generated.h"

/**
 * 
 */
UCLASS()
class SPACIALPARADOXMAZE_API AMazeGameModeBase : public AGameModeBase
{
	GENERATED_BODY()

public:

	AMazeGameModeBase();
	virtual ~AMazeGameModeBase();

	virtual void InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage) override;
	virtual void InitGameState() override;
};
