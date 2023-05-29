// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralGear.generated.h"

class UProceduralMeshComponent;
class UPhysicsConstraintComponent;

UCLASS()
class GEARS_API AProceduralGear : public AActor
{
	GENERATED_BODY()

		virtual void PostLoad() override;
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif // WITH_EDITOR

	const unsigned int CENTER_RINGS = 2;
	const unsigned int SECTIONS_PER_TOOTH = 3;


	//UPROPERTY(EditAnywhere);
	USceneComponent* scene;

	UPROPERTY(EditAnywhere, Category = "Gear Properties", Meta = (Units = "Millimeters", ClampMin = 0.1, ClampMax = 50.0));
	float _module = 10;

	UPROPERTY(EditAnywhere, Category = "Gear Properties", Meta = (ClampMin = 8, ClampMax = 100));
	unsigned int _number_of_teeth = 24;

	UPROPERTY(EditAnywhere, Category = "Gear Properties", Meta = (Units = "Millimeters", ClampMin = .01, ClampMax = 700.0));
	float _width = 15;

	//UPROPERTY(EditAnywhere, Category = "Gear Properties", Meta = (Units = "Millimeters", ClampMin = 0.0, ClampMax = 10.0));
	float _profile_shift = 0.0;

	UPROPERTY(EditAnywhere, Category = "Gear Properties", Meta = (Units = "Degrees", ClampMin = 14.5, ClampMax = 25.0));
	float _pressure_angle = 20.0;

	UPROPERTY(VisibleAnywhere, Category = "Gear Properties", Meta = (Units = "Millimeters"));
	float _reference_diameter = _module * _number_of_teeth;

	UPROPERTY(VisibleAnywhere, Category = "Gear Properties", Meta = (Units = "Millimeters"));
	float _base_diameter = _reference_diameter * cos(_pressure_angle * PI / 180.0);

	UPROPERTY(VisibleAnywhere, Category = "Gear Properties", Meta = (Units = "Millimeters"));
	float _base_radius = _base_diameter / 2.0;

	UPROPERTY(EditAnywhere);
	bool _apply_rotation = false;

	UPROPERTY(EditAnyWhere, Meta = (EditCondition = "_apply_rotation", ClampMin = -10000.0, ClampMax = 10000.0));
	float _rpm = 0.0;

	UPROPERTY(EditAnywhere, Meta = (EditCondition = "_apply_rotation"));
	float _velocity_strength = 0.0;

	//UPROPERTY(EditAnywhere);
	UPhysicsConstraintComponent* constraint;

	UPROPERTY(EditAnywhere, Category = "Contraint Properties");
	TSoftObjectPtr<AActor> join_to;

	UPROPERTY(EditAnywhere, Category = "Contraint Properties");
	FConstrainComponentPropName join_to_component_name;

	UPROPERTY(EditAnywhere, Category = "Contraint Properties");
	bool lock_rotation = false;

	UPROPERTY(EditAnywhere, Category = "Contraint Properties");
	bool disable_joined_collision = false;

	UPROPERTY(EditAnywhere);
	UMaterialInstance* _material = nullptr;

	UPROPERTY(EditAnywhere, AdvancedDisplay, Meta = (ClampMin = 4, ClampMax = 50));
	unsigned int _involute_steps = 4;

	UPROPERTY(EditAnywhere, AdvancedDisplay);
	bool _enable_collision = true;

	void generateGear();
public:
	// Sets default values for this actor's properties
	AProceduralGear();

	void Initialize();

	//Accessors
	float getModule() const;
	unsigned int getNumberOfTeeth() const;
	float getWidth() const;
	float getProfileShift() const;
	float getPressureAngle() const;
	float getRefDiameter() const;
	float getBaseDiameter() const;
	float getBaseRadius() const;
	bool hasRotationApplied() const;
	float getRPM() const;
	float getVelocityStrength() const;
	const TSoftObjectPtr<AActor>& getJoinedActor() const;
	const FConstrainComponentPropName& getJoinedComponent() const;
	bool rotationLocked() const;
	bool isJoinedCollisionDisabled() const;
	const UMaterialInstance* getMaterial() const;
	unsigned int getInvoluteSteps() const;
	bool isCollisionEnabled() const;

	//Mutators
	void setModule(float module_value);
	void setNumberOfTeeth(unsigned int num);
	void setWidth(float width);
	void setPressureAngle(float angle);
	void ApplyRotation(bool value);
	void setRPM(float rpm);
	void setVelocityStrength(float strength);
	void setJoinedActor(AActor* actor);
	void setJoinedComponent(const FConstrainComponentPropName& name);
	void lockRotation(bool value);
	void disableJoinedCollision(bool value);
	void setMaterial(UMaterialInstance* material);
	void setInvoluteSteps(unsigned int steps);
	void enableCollision(bool value);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

private:
	//UPROPERTY(VisibleAnywhere);
	UProceduralMeshComponent* mesh;

	void updateReferenceDiameter();
	void updateBaseDiameter();
	void updateBaseRadius();
};
