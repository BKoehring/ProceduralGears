// Fill out your copyright notice in the Description page of Project Settings.


#include "ProceduralGear.h"
#include "ProceduralMeshComponent.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "Math/UnitConversion.h"

// Sets default values
AProceduralGear::AProceduralGear()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	scene = CreateDefaultSubobject<USceneComponent>("DefaultSceneRoot");
	SetRootComponent(scene);

	mesh = CreateDefaultSubobject<UProceduralMeshComponent>("Gear Mesh");
	mesh->SetupAttachment(scene);
	mesh->bUseComplexAsSimpleCollision = false;
	mesh->SetSimulatePhysics(true);

	constraint = CreateDefaultSubobject<UPhysicsConstraintComponent>("PhysicsConstraint");
	constraint->SetupAttachment(scene);
	constraint->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
	constraint->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
	constraint->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);
	constraint->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0);
	constraint->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 0);
	constraint->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0);
	constraint->ConstraintActor1 = this;
	constraint->ComponentName1 = FConstrainComponentPropName{ mesh->GetFName() };

	Initialize();
}

void AProceduralGear::Initialize()
{
	constraint->ConstraintActor2 = join_to.Get();
	constraint->ComponentName2 = join_to_component_name;

	generateGear();
	mesh->SetMaterial(0, _material);
}

// Called when the game starts or when spawned
void AProceduralGear::BeginPlay()
{
	Super::BeginPlay();

	if (_apply_rotation) {
		constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
		constraint->SetAngularVelocityTarget(FVector(0, _rpm/60.0, 0));
		constraint->SetAngularVelocityDrive(true, false);
		constraint->SetAngularDriveParams(0, _velocity_strength, 0);
	}

	if (lock_rotation) {
		constraint->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0);
	}
}

void AProceduralGear::generateGear()
{
	TArray<FVector>verts;
	TArray<int>indices;
	TArray<FVector>normals;

	TArray<FVector>collision_verts;
	TArray<TArray<FVector>>collision_shapes;

	auto base_radius = getBaseRadius();
	auto tip_diameter = getRefDiameter() + 2 * getModule() * (1 + getProfileShift());
	auto tip_radius = tip_diameter / 2.0;
	auto u = sqrt((pow(tip_radius, 2) / pow(base_radius, 2)) - 1);
	auto tip_pressure_angle = acos(getBaseDiameter() / tip_diameter) * 180.0 / PI;
	auto inv_alpha = tan(getPressureAngle() * PI / 180.0) - getPressureAngle() * PI / 180.0;
	auto inv_alpha_a = tan(tip_pressure_angle * PI / 180.0) - tip_pressure_angle * PI / 180.0;
	auto top_thickness = PI / (2.0 * getNumberOfTeeth()) + inv_alpha - inv_alpha_a;
	auto end_x = base_radius * (cos(u) + u * sin(u));
	auto end_y = base_radius * (sin(u) - u * cos(u));
	auto distance = sqrt(pow(base_radius - end_x, 2) + pow(end_y, 2));
	auto cosx = (pow(base_radius, 2) + pow(tip_radius, 2) - pow(distance, 2)) / 2.0 / base_radius / tip_radius;
	auto tooth_thickness_rad = 2.0 * top_thickness + 2.0 * acos(cosx);
	auto spacing_arc_length = FMath::DegreesToRadians(360.0 / getNumberOfTeeth()) - tooth_thickness_rad;

	auto max_width = getWidth() / 2.0;
	auto min_width = getWidth() / -2.0;

	int radial_segments = getNumberOfTeeth() * _involute_steps * SECTIONS_PER_TOOTH;

	mesh->ClearMeshSection(0);
	mesh->ClearCollisionConvexMeshes();

	//Create Center of Gear
	//Add Top center vertice
	auto vert = FVector(0, max_width, 0);
	verts.Add(vert);
	normals.Add(vert.GetSafeNormal());

	//Add bottom center vertice
	vert = FVector(0, min_width, 0);
	verts.Add(vert);
	normals.Add(vert.GetSafeNormal());

	auto center_radial_segments = getNumberOfTeeth();
	for (unsigned int ring = 0; ring < CENTER_RINGS; ring++) {
		auto ring_radius = float(ring + 1.0) / float(CENTER_RINGS + 1.0) * base_radius;
		for (unsigned int segment = 0; segment < center_radial_segments; segment++) {
			auto radian = 2.0 * PI * (segment / float(center_radial_segments));
			auto x = ring_radius * cos(radian);
			auto z = ring_radius * sin(radian);

			//Top ring
			vert = FVector(x, max_width, z);
			verts.Add(vert);
			normals.Add(vert.GetSafeNormal());

			//Bottom ring
			vert = FVector(x, min_width, z);
			verts.Add(vert);
			normals.Add(vert.GetSafeNormal());

			if (segment > 0) {
				auto current_point = segment * 2 + 2;
				if (ring == 0) {
					//Add Top triangle around center point
					indices.Add(0);
					indices.Add(current_point - 2);
					indices.Add(current_point);

					//Add bottom trianlge around center point
					indices.Add(1);
					indices.Add(current_point + 1);
					indices.Add(current_point - 1);
				}
				else {
					//Connect Top rings
					indices.Add(center_radial_segments * (ring - 1) * 2 + (current_point - 2));
					indices.Add(center_radial_segments * ring * 2 + (current_point - 2));
					indices.Add(center_radial_segments * (ring - 1) * 2 + current_point);

					indices.Add(center_radial_segments * (ring - 1) * 2 + current_point);
					indices.Add(center_radial_segments * ring * 2 + (current_point - 2));
					indices.Add(center_radial_segments * ring * 2 + current_point);

					//Connect Bottom rings
					indices.Add(center_radial_segments * (ring - 1) * 2 + (current_point - 1));
					indices.Add(center_radial_segments * (ring - 1) * 2 + (current_point + 1));
					indices.Add(center_radial_segments * ring * 2 + (current_point - 1));

					indices.Add(center_radial_segments * (ring - 1) * 2 + (current_point + 1));
					indices.Add(center_radial_segments * ring * 2 + (current_point + 1));
					indices.Add(center_radial_segments * ring * 2 + (current_point - 1));
				}
			}
		}

		//Complete the circle
		auto last_vert = (ring + 1) * center_radial_segments * 2;
		if (ring == 0) {
			//Add top triangle around center point
			indices.Add(0);
			indices.Add(last_vert);
			indices.Add(2);

			//Add bottom triangle around center point
			indices.Add(1);
			indices.Add(3);
			indices.Add(last_vert + 1);
		}
		else {
			//Add top triangles connecting rings
			indices.Add(center_radial_segments * ring * 2);
			indices.Add(last_vert);
			indices.Add(center_radial_segments * (ring - 1) * 2 + 2);

			indices.Add(center_radial_segments * (ring - 1) * 2 + 2);
			indices.Add(last_vert);
			indices.Add(center_radial_segments * ring * 2 + 2);

			//Add bottom triangles connecting rings
			indices.Add(center_radial_segments * ring * 2 + 1);
			indices.Add(center_radial_segments * (ring - 1) * 2 + 3);
			indices.Add(last_vert + 1);

			indices.Add(center_radial_segments * (ring - 1) * 2 + 3);
			indices.Add(center_radial_segments * ring * 2 + 3);
			indices.Add(last_vert + 1);
		}
	}

	for (unsigned int current_tooth = 0; current_tooth < getNumberOfTeeth(); current_tooth++) {
		for (unsigned int segment = 0; segment < (_involute_steps * SECTIONS_PER_TOOTH); segment++) {
			auto t = u * (1.0 + (0.5 / (_involute_steps - 0.5))) / PI * acos(cos((segment + 0.5) * PI / _involute_steps));
			auto x = base_radius;
			auto z = base_radius;
			auto offset = FMath::DegreesToRadians(current_tooth * (360.0 / getNumberOfTeeth()));

			if (segment < _involute_steps) {
				x *= (cos(t + offset) + t * sin(t + offset));
				z *= (sin(t + offset) - t * cos(t + offset));
			}
			else if (segment < (_involute_steps * 2)) {
				x *= (cos(-t + offset + tooth_thickness_rad) - t * sin(-t + offset + tooth_thickness_rad));
				z *= (sin(-t + offset + tooth_thickness_rad) + t * cos(-t + offset + tooth_thickness_rad));
			}
			else {
				auto spacing_arc_start_rad = tooth_thickness_rad + offset;
				auto spacing_arc_start_coord = FVector2f(x * cos(spacing_arc_start_rad), z * sin(spacing_arc_start_rad));
				auto spacing_arc_end_rad = spacing_arc_start_rad + spacing_arc_length;
				auto arc_end_coord = FVector2f(x * cos(spacing_arc_end_rad), z * sin(spacing_arc_end_rad));
				auto spacing_center_rad = spacing_arc_start_rad + spacing_arc_length / 2.0;
				auto spacing_center_coord = FVector2f(x * cos(spacing_center_rad), z * sin(spacing_center_rad));
				auto spacing_circle_start_angle = atan2(abs(spacing_arc_start_coord.Y - spacing_center_coord.Y), abs(spacing_arc_start_coord.X - spacing_center_coord.X));
				auto spacing_circle_end_angle = atan2(abs(arc_end_coord.Y - spacing_center_coord.Y), abs(arc_end_coord.X - spacing_center_coord.X));

				double spacing_circle_start = spacing_circle_start_angle;
				if (spacing_arc_start_coord.X > spacing_center_coord.X) {
					if (spacing_arc_start_coord.Y < spacing_center_coord.Y) {
						spacing_circle_start = 2 * PI - spacing_circle_start;
					}
				}
				else {
					if (spacing_arc_start_coord.Y > spacing_center_coord.Y) {
						spacing_circle_start = PI - spacing_circle_start;
					}
					else {
						spacing_circle_start += PI;
					}
				}

				double spacing_circle_end = spacing_circle_end_angle;
				if (arc_end_coord.X > spacing_center_coord.X) {
					if (arc_end_coord.Y < spacing_center_coord.Y) {
						spacing_circle_end = 2 * PI - spacing_circle_end;
					}
				}
				else {
					if (arc_end_coord.Y > spacing_center_coord.Y) {
						spacing_circle_end = PI - spacing_circle_end;
					}
					else {
						spacing_circle_end += PI;
					}
				}

				if (spacing_circle_start < spacing_circle_end) {
					spacing_circle_start += 2 * PI;
				}

				auto spacing_circle_radius = sqrt(pow(spacing_center_coord.X - spacing_arc_start_coord.X, 2) + pow(spacing_center_coord.Y - spacing_arc_start_coord.Y, 2));
				auto spacing_circle_step = (spacing_circle_start - spacing_circle_end) / (_involute_steps + 1.0);
				auto spacing_circle_radial = spacing_circle_start - spacing_circle_step * (segment % _involute_steps + 1);
				x = spacing_circle_radius * cos(spacing_circle_radial) + spacing_center_coord.X;
				z = spacing_circle_radius * sin(spacing_circle_radial) + spacing_center_coord.Y;

				if (!collision_verts.IsEmpty()) {
					collision_shapes.Add(collision_verts);
					collision_verts.Empty();
				}
			}

			if (segment == (_involute_steps - 1) || segment == _involute_steps) {
				//Add top vert
				vert = FVector(x, max_width * .8, z);
				verts.Add(vert);
				normals.Add(vert.GetSafeNormal());

				//Add bottom vert
				vert = FVector(x, min_width * .8, z);
				verts.Add(vert);
				normals.Add(vert.GetSafeNormal());
			}
			else {
				//Add top vert
				vert = FVector(x, max_width, z);
				verts.Add(vert);
				normals.Add(vert.GetSafeNormal());

				//Add bottom vert
				vert = FVector(x, min_width, z);
				verts.Add(vert);
				normals.Add(vert.GetSafeNormal());
			}

			if (_enable_collision) {
				collision_verts.Add(FVector{ x,max_width,z });
				collision_verts.Add(FVector{ x,min_width,z });
			}
		}

		if (current_tooth > 0) {
			auto first_point = (CENTER_RINGS - 1) * center_radial_segments * 2 + (2 * current_tooth);
			auto tooth_starting_point = (current_tooth - 1) * _involute_steps * SECTIONS_PER_TOOTH * 2 + (CENTER_RINGS * center_radial_segments * 2) + 2;
			auto tooth_end_point = tooth_starting_point + _involute_steps * 4 - 2;

			//Top triangles
			indices.Add(first_point);
			indices.Add(tooth_starting_point);
			indices.Add(tooth_end_point);

			//Bottom triangles
			indices.Add(first_point + 1);
			indices.Add(tooth_end_point + 1);
			indices.Add(tooth_starting_point + 1);

			for (unsigned int involute_step = 0; involute_step < (_involute_steps - 1); involute_step++) {
				auto increment = involute_step * 2;

				//Top Tooth Triangles
				indices.Add(tooth_starting_point + increment);
				indices.Add(tooth_starting_point + 2 + increment);
				indices.Add(tooth_end_point - increment);

				indices.Add(tooth_end_point - increment);
				indices.Add(tooth_starting_point + 2 + increment);
				indices.Add(tooth_end_point - 2 - increment);

				//Bottom Tooth Triangles
				indices.Add(tooth_starting_point + increment + 1);
				indices.Add(tooth_end_point - increment + 1);
				indices.Add(tooth_starting_point + 3 + increment);

				indices.Add(tooth_end_point - increment + 1);
				indices.Add(tooth_end_point - 1 - increment);
				indices.Add(tooth_starting_point + 3 + increment);

				//Connect Top and Bottom Teeth
				indices.Add(tooth_starting_point + increment);
				indices.Add(tooth_starting_point + increment + 1);
				indices.Add(tooth_starting_point + increment + 2);

				indices.Add(tooth_starting_point + increment + 2);
				indices.Add(tooth_starting_point + increment + 1);
				indices.Add(tooth_starting_point + increment + 3);

				indices.Add(tooth_end_point - increment);
				indices.Add(tooth_end_point - increment - 2);
				indices.Add(tooth_end_point - increment + 1);

				indices.Add(tooth_end_point - increment + 1);
				indices.Add(tooth_end_point - increment - 2);
				indices.Add(tooth_end_point - increment - 1);

				//Spacing Triangles
				if (involute_step < (_involute_steps / 2.0 - 0.5)) {
					//Top
					indices.Add(first_point);
					indices.Add(tooth_end_point + (involute_step * 2));
					indices.Add(tooth_end_point + (involute_step * 2) + 2);

					//Bottom
					indices.Add(first_point + 1);
					indices.Add(tooth_end_point + (involute_step * 2) + 3);
					indices.Add(tooth_end_point + (involute_step * 2) + 1);
				}
				else if (involute_step <= (_involute_steps / 2.0)) {
					//Top
					indices.Add(first_point);
					indices.Add(tooth_end_point + (involute_step * 2));
					indices.Add(first_point + 2);

					indices.Add(first_point + 2);
					indices.Add(tooth_end_point + (involute_step * 2));
					indices.Add(tooth_end_point + (involute_step * 2) + 2);

					//Bottom
					indices.Add(first_point + 1);
					indices.Add(first_point + 3);
					indices.Add(tooth_end_point + (involute_step * 2) + 1);

					indices.Add(first_point + 3);
					indices.Add(tooth_end_point + (involute_step * 2) + 3);
					indices.Add(tooth_end_point + (involute_step * 2) + 1);
				}
				else {
					//Top
					indices.Add(first_point + 2);
					indices.Add(tooth_end_point + (involute_step * 2));
					indices.Add(tooth_end_point + (involute_step * 2) + 2);

					//Bottom
					indices.Add(first_point + 3);
					indices.Add(tooth_end_point + (involute_step * 2) + 3);
					indices.Add(tooth_end_point + (involute_step * 2) + 1);
				}


				//Connect top and bottom Spacing
				indices.Add(tooth_end_point + (involute_step * 2));
				indices.Add(tooth_end_point + (involute_step * 2) + 1);
				indices.Add(tooth_end_point + (involute_step * 2) + 2);

				indices.Add(tooth_end_point + (involute_step * 2) + 2);
				indices.Add(tooth_end_point + (involute_step * 2) + 1);
				indices.Add(tooth_end_point + (involute_step * 2) + 3);

				////Connect gaps
				if (involute_step == _involute_steps - 2) {
					//Connect tooth ends
					indices.Add(tooth_starting_point + (involute_step * 2) + 2);
					indices.Add(tooth_starting_point + (involute_step * 2) + 3);
					indices.Add(tooth_starting_point + (involute_step * 2) + 4);

					indices.Add(tooth_starting_point + (involute_step * 2) + 4);
					indices.Add(tooth_starting_point + (involute_step * 2) + 3);
					indices.Add(tooth_starting_point + (involute_step * 2) + 5);

					//Connect top spacing to next tooth
					indices.Add(first_point + 2);
					indices.Add(tooth_end_point + (involute_step * 2) + 2);
					indices.Add(tooth_end_point + (involute_step * 2) + 4);

					indices.Add(first_point + 2);
					indices.Add(tooth_end_point + (involute_step * 2) + 4);
					indices.Add(tooth_end_point + (involute_step * 2) + 6);

					//Connect bottom spacing to next tooth
					indices.Add(first_point + 3);
					indices.Add(tooth_end_point + (involute_step * 2) + 5);
					indices.Add(tooth_end_point + (involute_step * 2) + 3);

					indices.Add(first_point + 3);
					indices.Add(tooth_end_point + (involute_step * 2) + 7);
					indices.Add(tooth_end_point + (involute_step * 2) + 5);

					//Connect top and bottom Spacing
					indices.Add(tooth_end_point + (involute_step * 2) + 2);
					indices.Add(tooth_end_point + (involute_step * 2) + 3);
					indices.Add(tooth_end_point + (involute_step * 2) + 4);

					indices.Add(tooth_end_point + (involute_step * 2) + 4);
					indices.Add(tooth_end_point + (involute_step * 2) + 3);
					indices.Add(tooth_end_point + (involute_step * 2) + 5);

					indices.Add(tooth_end_point + (involute_step * 2) + 4);
					indices.Add(tooth_end_point + (involute_step * 2) + 5);
					indices.Add(tooth_end_point + (involute_step * 2) + 6);

					indices.Add(tooth_end_point + (involute_step * 2) + 6);
					indices.Add(tooth_end_point + (involute_step * 2) + 5);
					indices.Add(tooth_end_point + (involute_step * 2) + 7);
				}
			}
		}

		//Add Last Tooth
		if (current_tooth == getNumberOfTeeth() - 1) {
			auto first_point = (CENTER_RINGS - 1) * center_radial_segments * 2 + (2 * (current_tooth + 1));
			auto next_point = (CENTER_RINGS - 1) * center_radial_segments * 2 + 2;
			auto tooth_starting_point = current_tooth * _involute_steps * SECTIONS_PER_TOOTH * 2 + (CENTER_RINGS * center_radial_segments * 2) + 2;
			auto tooth_end_point = tooth_starting_point + _involute_steps * 4 - 2;
			auto next_tooth = CENTER_RINGS * center_radial_segments * 2 + 2;

			//Top triangles
			indices.Add(first_point);
			indices.Add(tooth_starting_point);
			indices.Add(tooth_end_point);

			//Bottom triangles
			indices.Add(first_point + 1);
			indices.Add(tooth_end_point + 1);
			indices.Add(tooth_starting_point + 1);

			for (unsigned int involute_step = 0; involute_step < (_involute_steps - 1); involute_step++) {
				auto increment = involute_step * 2;

				//Top Tooth Triangles
				indices.Add(tooth_starting_point + increment);
				indices.Add(tooth_starting_point + 2 + increment);
				indices.Add(tooth_end_point - increment);

				indices.Add(tooth_end_point - increment);
				indices.Add(tooth_starting_point + 2 + increment);
				indices.Add(tooth_end_point - 2 - increment);

				//Bottom Tooth Triangles
				indices.Add(tooth_starting_point + increment + 1);
				indices.Add(tooth_end_point - increment + 1);
				indices.Add(tooth_starting_point + 3 + increment);

				indices.Add(tooth_end_point - increment + 1);
				indices.Add(tooth_end_point - 1 - increment);
				indices.Add(tooth_starting_point + 3 + increment);

				//Connect Top and Bottom Teeth
				indices.Add(tooth_starting_point + increment);
				indices.Add(tooth_starting_point + increment + 1);
				indices.Add(tooth_starting_point + increment + 2);

				indices.Add(tooth_starting_point + increment + 2);
				indices.Add(tooth_starting_point + increment + 1);
				indices.Add(tooth_starting_point + increment + 3);

				indices.Add(tooth_end_point - increment);
				indices.Add(tooth_end_point - increment - 2);
				indices.Add(tooth_end_point - increment + 1);

				indices.Add(tooth_end_point - increment + 1);
				indices.Add(tooth_end_point - increment - 2);
				indices.Add(tooth_end_point - increment - 1);

				//Spacing Triangles
				if (involute_step < (_involute_steps / 2.0 - 0.5)) {
					//Top
					indices.Add(first_point);
					indices.Add(tooth_end_point + (involute_step * 2));
					indices.Add(tooth_end_point + (involute_step * 2) + 2);

					//Bottom
					indices.Add(first_point + 1);
					indices.Add(tooth_end_point + (involute_step * 2) + 3);
					indices.Add(tooth_end_point + (involute_step * 2) + 1);
				}
				else if (involute_step <= (_involute_steps / 2.0)) {
					//Top
					indices.Add(first_point);
					indices.Add(tooth_end_point + (involute_step * 2));
					indices.Add(next_point);

					indices.Add(next_point);
					indices.Add(tooth_end_point + (involute_step * 2));
					indices.Add(tooth_end_point + (involute_step * 2) + 2);

					//Bottom
					indices.Add(first_point + 1);
					indices.Add(next_point + 1);
					indices.Add(tooth_end_point + (involute_step * 2) + 1);

					indices.Add(next_point + 1);
					indices.Add(tooth_end_point + (involute_step * 2) + 3);
					indices.Add(tooth_end_point + (involute_step * 2) + 1);
				}
				else {
					//Top
					indices.Add(next_point);
					indices.Add(tooth_end_point + (involute_step * 2));
					indices.Add(tooth_end_point + (involute_step * 2) + 2);

					//Bottom
					indices.Add(next_point + 1);
					indices.Add(tooth_end_point + (involute_step * 2) + 3);
					indices.Add(tooth_end_point + (involute_step * 2) + 1);
				}


				//Connect top and bottom Spacing
				indices.Add(tooth_end_point + (involute_step * 2));
				indices.Add(tooth_end_point + (involute_step * 2) + 1);
				indices.Add(tooth_end_point + (involute_step * 2) + 2);

				indices.Add(tooth_end_point + (involute_step * 2) + 2);
				indices.Add(tooth_end_point + (involute_step * 2) + 1);
				indices.Add(tooth_end_point + (involute_step * 2) + 3);

				////Connect gaps
				if (involute_step == _involute_steps - 2) {
					//Connect tooth ends
					indices.Add(tooth_starting_point + (involute_step * 2) + 2);
					indices.Add(tooth_starting_point + (involute_step * 2) + 3);
					indices.Add(tooth_starting_point + (involute_step * 2) + 4);

					indices.Add(tooth_starting_point + (involute_step * 2) + 4);
					indices.Add(tooth_starting_point + (involute_step * 2) + 3);
					indices.Add(tooth_starting_point + (involute_step * 2) + 5);

					//Connect top spacing to next tooth
					indices.Add(next_point);
					indices.Add(tooth_end_point + (involute_step * 2) + 2);
					indices.Add(tooth_end_point + (involute_step * 2) + 4);

					indices.Add(next_point);
					indices.Add(tooth_end_point + (involute_step * 2) + 4);
					indices.Add(next_tooth);

					//Connect bottom spacing to next tooth
					indices.Add(next_point + 1);
					indices.Add(tooth_end_point + (involute_step * 2) + 5);
					indices.Add(tooth_end_point + (involute_step * 2) + 3);

					indices.Add(next_point + 1);
					indices.Add(next_tooth + 1);
					indices.Add(tooth_end_point + (involute_step * 2) + 5);

					//Connect top and bottom Spacing
					indices.Add(tooth_end_point + (involute_step * 2) + 2);
					indices.Add(tooth_end_point + (involute_step * 2) + 3);
					indices.Add(tooth_end_point + (involute_step * 2) + 4);

					indices.Add(tooth_end_point + (involute_step * 2) + 4);
					indices.Add(tooth_end_point + (involute_step * 2) + 3);
					indices.Add(tooth_end_point + (involute_step * 2) + 5);

					indices.Add(tooth_end_point + (involute_step * 2) + 4);
					indices.Add(tooth_end_point + (involute_step * 2) + 5);
					indices.Add(next_tooth);

					indices.Add(next_tooth);
					indices.Add(tooth_end_point + (involute_step * 2) + 5);
					indices.Add(next_tooth + 1);
				}
			}
		}
	}

	mesh->CreateMeshSection(
		0,
		verts,
		indices,
		normals,
		TArray<FVector2D>(),
		TArray<FColor>(),
		TArray<FProcMeshTangent>(),
		false);

	for (auto collision_shape : collision_shapes) {
		mesh->AddCollisionConvexMesh(collision_shape);
	}
}

// Called every frame
void AProceduralGear::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AProceduralGear::PostLoad()
{
	Super::PostLoad();

	Initialize();
}

#if WITH_EDITOR  
void AProceduralGear::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) {
	auto property_name = PropertyChangedEvent.GetPropertyName();
	auto regenerate_gear = true;
	if (property_name == "_module") {
		updateReferenceDiameter();
	}
	else if (property_name == "_number_of_teeth") {
		updateReferenceDiameter();
	}
	else if (property_name == "_pressure_angle") {
		updateBaseDiameter();
	}
	else if (property_name == "_width") {
		//Nothing special to do. Gear will be regenerated
	}
	else if (property_name == "_profile_shift") {
		//Nothing special to do. Gear will be regenerated
	}
	else if (property_name == "_involute_steps") {
		//Nothing special to do. Gear will be regenerated
	}
	else if (property_name == "material") {
		mesh->SetMaterial(0, _material);
		regenerate_gear = false;
	}
	else if (property_name == "_enable_collision") {
		//Nothing special to do. Gear will be regenerated
		//mesh->SetSimulatePhysics(_enable_collision);
	}
	else if (property_name == "join_to") {
		constraint->ConstraintActor2 = join_to.Get();
		regenerate_gear = false;
	}
	else if (property_name == "join_to_component_name") {
		constraint->ComponentName2 = join_to_component_name;
		regenerate_gear = false;
	}
	else if (property_name == "disable_joined_collision") {
		constraint->SetDisableCollision(disable_joined_collision);
		regenerate_gear = false;
	}
	else if (property_name == "ComponentName") {
		constraint->ComponentName2 = join_to_component_name;
		regenerate_gear = false;
	}

	if (regenerate_gear) {
		generateGear();
	}

	Super::PostEditChangeProperty(PropertyChangedEvent);
}
#endif

float AProceduralGear::getModule() const
{
	return FUnitConversion::Convert<float>(_module, EUnit::Millimeters, EUnit::Centimeters);
}

unsigned int AProceduralGear::getNumberOfTeeth() const
{
	return _number_of_teeth;
}

float AProceduralGear::getWidth() const
{
	return FUnitConversion::Convert<float>(_width, EUnit::Millimeters, EUnit::Centimeters);
}

float AProceduralGear::getProfileShift() const
{
	return FUnitConversion::Convert<float>(_profile_shift, EUnit::Millimeters, EUnit::Centimeters);
}

float AProceduralGear::getPressureAngle() const
{
	return _pressure_angle;
}

float AProceduralGear::getRefDiameter() const
{
	return FUnitConversion::Convert<float>(_reference_diameter, EUnit::Millimeters, EUnit::Centimeters);
}

float AProceduralGear::getBaseDiameter() const
{
	return FUnitConversion::Convert<float>(_base_diameter, EUnit::Millimeters, EUnit::Centimeters);
}

float AProceduralGear::getBaseRadius() const
{
	return FUnitConversion::Convert<float>(_base_radius, EUnit::Millimeters, EUnit::Centimeters);
}

bool AProceduralGear::hasRotationApplied() const
{
	return _apply_rotation;
}

float AProceduralGear::getRPM() const
{
	return _rpm;
}

float AProceduralGear::getVelocityStrength() const
{
	return _velocity_strength;
}

const TSoftObjectPtr<AActor>& AProceduralGear::getJoinedActor() const
{
	return join_to;
}

const FConstrainComponentPropName& AProceduralGear::getJoinedComponent() const
{
	return join_to_component_name;
}

bool AProceduralGear::rotationLocked() const
{
	return lock_rotation;
}

bool AProceduralGear::isJoinedCollisionDisabled() const
{
	return disable_joined_collision;
}

const UMaterialInstance* AProceduralGear::getMaterial() const
{
	return _material;
}

unsigned int AProceduralGear::getInvoluteSteps() const
{
	return _involute_steps;
}

bool AProceduralGear::isCollisionEnabled() const
{
	return _enable_collision;
}

void AProceduralGear::setModule(float module_value)
{
	_module = module_value;
	updateReferenceDiameter();
	generateGear();
}

void AProceduralGear::setNumberOfTeeth(unsigned int num)
{
	_number_of_teeth = num;
	updateReferenceDiameter();
	generateGear();
}

void AProceduralGear::setWidth(float width)
{
	_width = width;
	generateGear();
}

void AProceduralGear::setPressureAngle(float angle)
{
	_pressure_angle = angle;
	generateGear();
}

void AProceduralGear::ApplyRotation(bool value)
{
	_apply_rotation = value;
}

void AProceduralGear::setRPM(float rpm)
{
	_rpm = rpm;
}

void AProceduralGear::setVelocityStrength(float strength)
{
	_velocity_strength = strength;
}

void AProceduralGear::setJoinedActor(AActor* actor)
{
	join_to = actor;
	constraint->ConstraintActor2 = join_to.Get();
}

void AProceduralGear::setJoinedComponent(const FConstrainComponentPropName& name)
{
	join_to_component_name = name;
	constraint->ComponentName2 = join_to_component_name;
}

void AProceduralGear::lockRotation(bool value)
{
	lock_rotation = value;
}

void AProceduralGear::disableJoinedCollision(bool value)
{
	disable_joined_collision = value;
	constraint->SetDisableCollision(disable_joined_collision);
}

void AProceduralGear::setMaterial(UMaterialInstance* material)
{
	_material = material;
	generateGear();
}

void AProceduralGear::setInvoluteSteps(unsigned int steps)
{
	_involute_steps = steps;
	generateGear();
}

void AProceduralGear::enableCollision(bool value)
{
	_enable_collision = value;
	generateGear();
}

void AProceduralGear::updateReferenceDiameter()
{
	_reference_diameter = _module * _number_of_teeth;
	updateBaseDiameter();
}

void AProceduralGear::updateBaseDiameter()
{
	_base_diameter = _reference_diameter * cos(_pressure_angle * PI / 180.0);
	updateBaseRadius();
}

void AProceduralGear::updateBaseRadius()
{
	_base_radius = _base_diameter / 2.0;
}

