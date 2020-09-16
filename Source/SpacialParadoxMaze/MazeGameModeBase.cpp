// Fill out your copyright notice in the Description page of Project Settings.

#include "MazeGameModeBase.h"
#include "MazeGamePlayerController.h"

AMazeGameModeBase::AMazeGameModeBase()
{
	this->PrimaryActorTick.bCanEverTick = false;
	this->PrimaryActorTick.bStartWithTickEnabled = false;
	this->PrimaryActorTick.bAllowTickOnDedicatedServer = false;
	
	this->PlayerControllerClass = AMazeGamePlayerController::StaticClass();
}

/*virtual*/ AMazeGameModeBase::~AMazeGameModeBase()
{
}

/*virtual*/ void AMazeGameModeBase::InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage)
{
	AGameModeBase::InitGame(MapName, Options, ErrorMessage);
}

/*virtual*/ void AMazeGameModeBase::InitGameState()
{
	AGameModeBase::InitGameState();
}