#include "shared.h"
#include "gs_physics.h"

#include "config/gs_config.h"
#include "debug/debug_graph.h"
#include "gs_timer.h"
#include "gs_physicsfx.h"
#include "flash/mng_fade.h"
#include "flash/modal_message_box.h"
#include "game/gm_base.h"
#include "game/scenaric/gs_scenaric.h"
#include "game/vehicle/car_state.h"
#include "game/game_object.h"
#include "game/vehicle/gs_car_state.h"
#include "vehicle/physics_vehicle.h"
#include "world_collisions.h"
#include "physics_layers.h"
#include "physics_material.h"
#include "game/material_types.h"
#include "havok/havok_container.h"
#include "havok/havok_heightfield_agent.h"
#include "havok/havok_packfile.h"
#include "havok/havok_to_jolt.h"
#include "database/gs_database.h"
#include "input/ed_dev.h"
#include "input/input_device.h"
#include "input/gs_device.h"
#include "player_data/gs_playerdata.h"

#include "render/camera/gs_camera.h"
#include "render/draw_list.h"
#include "render/gs_render.h"
#include "render/2dm.h"
#include "render/render_context.h"
#include "world/gs_world.h"
#include "world/gs_world_streaming.h"
#include "world/streaming_manager.h"

#include "imgui.h"

#include <cmath>
#include <vector>

#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/SimShapeFilter.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/TransformedShape.h>

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Geometry/RayAABox.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/EActivation.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>

class PhysicsCharacter {
public:
    JPH::Ref<JPH::CharacterVirtual> pCharacter;
    // Invalid = collide with everything (the default).
    JPH::BodyID RestrictToBody;
    // Only honoured while RestrictToBody is valid.
    std::vector<JPH::BodyID> ExtraBodies;
};

class AllowListBodyFilter final : public JPH::BodyFilter {
public:
    explicit AllowListBodyFilter( const PhysicsCharacter &character )
        : character( character ) {}

    bool ShouldCollide( const JPH::BodyID &inBodyID ) const override {
        if ( inBodyID == character.RestrictToBody ) {
            return true;
        }
        for ( const JPH::BodyID &extraBody : character.ExtraBodies ) {
            if ( inBodyID == extraBody ) {
                return true;
            }
        }
        return false;
    }

    bool ShouldCollideLocked( const JPH::Body &inBody ) const override {
        return ShouldCollide( inBody.GetID() );
    }

private:
    const PhysicsCharacter &character;
};

static constexpr float kMaxCharacterFallSpeed = 15.0f;

namespace {
    // Terrain replaces terrain, so another sector's heightfield never counts.
    class NonTerrainBodyFilter : public JPH::BodyFilter {
    public:
        bool ShouldCollideLocked( const JPH::Body &inBody ) const override {
            const JPH::Shape *pShape = inBody.GetShape();
            return pShape != nullptr &&
                   pShape->GetSubType() != JPH::EShapeSubType::HeightField;
        }
    };

    // Static world geometry plus the wheel-only road: the drivable surface
    // inside a tunnel is a ROAD body, and keeping terrain over it would wall
    // the tunnel off exactly like the bug this replaces.
    class StaticOrRoadLayerFilter : public JPH::ObjectLayerFilter {
    public:
        bool ShouldCollide( JPH::ObjectLayer inLayer ) const override {
            return inLayer == TDUPhysLayers::NON_MOVING ||
                   inLayer == TDUPhysLayers::ROAD;
        }
    };

    class NonSensorLayerFilter : public JPH::ObjectLayerFilter {
    public:
        bool ShouldCollide( JPH::ObjectLayer inLayer ) const override {
            return inLayer != TDUPhysLayers::SENSOR &&
                   inLayer != TDUPhysLayers::ROAD_GROUND;
        }
    };

    class AsphaltNormalCollector final : public JPH::CollideShapeCollector {
    public:
        AsphaltNormalCollector( JPH::CollideShapeCollector &inChained,
                                const JPH::Body &inBody1,
                                const JPH::Body &inBody2,
                                bool bInWorldIsBody2 )
            : JPH::CollideShapeCollector( inChained ),
              chained( inChained ),
              body1( inBody1 ),
              pWorld( bInWorldIsBody2 ? &inBody2 : &inBody1 ),
              bWorldIsBody2( bInWorldIsBody2 ) {}

        void Reset() override {
            JPH::CollideShapeCollector::Reset();
            chained.Reset();
        }

        void OnBody( const JPH::Body &inBody ) override {
            chained.OnBody( inBody );
        }

        void AddHit( const JPH::CollideShapeResult &inResult ) override {
            const JPH::SubShapeID &subShapeID =
                bWorldIsBody2 ? inResult.mSubShapeID2 : inResult.mSubShapeID1;
            const JPH::Shape *pShape = pWorld->GetShape();
            const uint32_t material =
                ( pShape != nullptr ) ? TDUPhysicsMaterials::codeOf(
                                            pShape->GetMaterial( subShapeID ) )
                                      : TDUMaterialValue::kDefault;
            if ( material !=
                 static_cast<uint32_t>( eMaterialType::MT_Asphalt ) ) {
                forward( inResult );
                return;
            }

            // Results are relative to body com
            const JPH::Vec3 &pointOnWorld = bWorldIsBody2
                                                ? inResult.mContactPointOn2
                                                : inResult.mContactPointOn1;
            const JPH::RVec3 worldPoint =
                body1.GetCenterOfMassPosition() + pointOnWorld;
            JPH::Vec3 normal =
                pWorld->GetWorldSpaceSurfaceNormal( subShapeID, worldPoint );

            constexpr float kFlattenCosine = 0.75f; // FUN_0041a0d0
            if ( std::fabs( normal.GetY() ) < kFlattenCosine ) {
                normal.SetY( 0.0f );
            }
            normal = normal.NormalizedOr( JPH::Vec3::sZero() );
            if ( normal.IsNearZero() ) {
                forward( inResult );
                return;
            }

            JPH::CollideShapeResult result = inResult;
            result.mPenetrationAxis = ( bWorldIsBody2 ? -normal : normal ) *
                                      inResult.mPenetrationAxis.Length();
            forward( result );
        }

    private:
        void forward( const JPH::CollideShapeResult &inResult ) {
            chained.SetContext( GetContext() );
            chained.AddHit( inResult );
            UpdateEarlyOutFraction( chained.GetEarlyOutFraction() );
        }

        JPH::CollideShapeCollector &chained;
        const JPH::Body &body1;
        const JPH::Body *pWorld;
        bool bWorldIsBody2;
    };

    // Only the world meshes carry the listener in the original
    // (addCollisionListener, FUN_0047b390); everything else keeps Jolt's
    // normals.
    inline bool IsWorldMesh( const JPH::Body &inBody ) {
        const uint32_t surface = static_cast<uint32_t>(
            inBody.GetUserData() & TDUSurface::kClassMask );
        return surface == TDUSurface::Paved || surface == TDUSurface::RoadMesh;
    }

    void TDUSimCollideBodyVsBody( const JPH::Body &inBody1,
                                  const JPH::Body &inBody2,
                                  JPH::Mat44Arg inCenterOfMassTransform1,
                                  JPH::Mat44Arg inCenterOfMassTransform2,
                                  JPH::CollideShapeSettings &ioSettings,
                                  JPH::CollideShapeCollector &ioCollector,
                                  const JPH::ShapeFilter &inShapeFilter ) {
        const bool bWorld2 = IsWorldMesh( inBody2 );
        if ( !bWorld2 && !IsWorldMesh( inBody1 ) ) {
            JPH::PhysicsSystem::sDefaultSimCollideBodyVsBody(
                inBody1, inBody2, inCenterOfMassTransform1,
                inCenterOfMassTransform2, ioSettings, ioCollector,
                inShapeFilter );
            return;
        }

        AsphaltNormalCollector collector( ioCollector, inBody1, inBody2,
                                          bWorld2 );
        JPH::PhysicsSystem::sDefaultSimCollideBodyVsBody(
            inBody1, inBody2, inCenterOfMassTransform1,
            inCenterOfMassTransform2, ioSettings, collector, inShapeFilter );
    }

    // hkContactPointMaterial stores friction as u16 / 256 (FLOAT_00f419c0)
    // and restitution as u8 / 128 (FLOAT_00f4176c).
    float QuantizeFriction( const float fixedFriction ) {
        const int32_t fixed =
            static_cast<int32_t>( std::nearbyint( fixedFriction ) );
        return static_cast<float>( static_cast<uint16_t>( fixed ) ) / 256.0f;
    }

    float QuantizeRestitution( const float fixedRestitution ) {
        const int32_t fixed =
            static_cast<int32_t>( std::nearbyint( fixedRestitution ) );
        return static_cast<float>( static_cast<uint8_t>( fixed ) ) / 128.0f;
    }

    float CombineFriction( const JPH::Body &inBody1,
                           const JPH::SubShapeID &,
                           const JPH::Body &inBody2,
                           const JPH::SubShapeID & ) {
        // FUN_0048d730
        const float combined =
            std::sqrt( inBody1.GetFriction() * inBody2.GetFriction() );
        return QuantizeFriction( combined * 256.0f );
    }

    float CombineRestitution( const JPH::Body &inBody1,
                              const JPH::SubShapeID &,
                              const JPH::Body &inBody2,
                              const JPH::SubShapeID & ) {
        const float combined =
            std::sqrt( inBody1.GetRestitution() * inBody2.GetRestitution() );
        return QuantizeRestitution( combined * 128.0f );
    }

    // Havok group-filter layer of a body
    uint32_t RetailCollisionLayer( const JPH::Body &body ) {
        switch ( body.GetUserData() & TDUSurface::kClassMask ) {
        case TDUSurface::RoadGround:
            return 6; // DAT_010d6ec0 (FUN_00848530)
        case TDUSurface::RoadMesh:
            return 7; // FUN_00845380
        case TDUSurface::Vehicle:
            return body.GetObjectLayer() == TDUPhysLayers::VEHICLE_GHOST ? 4
                                                                         : 2;
        case TDUSurface::BikeRider:
            return 2;
        default:
            break;
        }
        // DAT_010d6ec4: .shk objects (FUN_00844f00) and the terrain
        // heightfield (FUN_00848b50).
        return body.GetObjectLayer() == TDUPhysLayers::MOVING ? 1 : 0;
    }

    bool RetailLayersCollide( uint32_t a, uint32_t b ) {
        // FUN_00841100: layer 15 collides with nothing; otherwise the
        // FUN_00841270 default (a == b, or layer 0) + Initialize_Scene
        // (FUN_0081d960) overrides, for the layers a wheel sphere uses.
        if ( a == 15 || b == 15 ) {
            return false;
        }
        if ( a == b || a == 0 || b == 0 ) {
            return true;
        }
        const uint32_t other = ( a == 5 ) ? b : a;
        return other == 1 || other == 7;
    }

    // The chassis wheel spheres sit on their own Havok layer (FUN_0083a560)
    // inside the chassis list shape.
    class WheelSphereFilter final : public JPH::SimShapeFilter {
    public:
        bool ShouldCollide( const JPH::Body &inBody1,
                            const JPH::Shape *inShape1,
                            const JPH::SubShapeID &,
                            const JPH::Body &inBody2,
                            const JPH::Shape *inShape2,
                            const JPH::SubShapeID & ) const override {
            const bool bSphere1 = PhysicsVehicle::IsWheelSphere( inShape1 );
            const bool bSphere2 = PhysicsVehicle::IsWheelSphere( inShape2 );
            if ( !bSphere1 && !bSphere2 ) {
                return true;
            }
            return RetailLayersCollide( layerOf( inBody1, bSphere1 ),
                                        layerOf( inBody2, bSphere2 ) );
        }

    private:
        static uint32_t layerOf( const JPH::Body &body, bool bSphere ) {
            if ( bSphere ) {
                const PhysicsVehicle *pVehicle = gpPhysics->findVehicle( body );
                return ( pVehicle != nullptr ) ? pVehicle->getWheelSphereLayer()
                                               : 15;
            }
            return RetailCollisionLayer( body );
        }
    };
    WheelSphereFilter gWheelSphereFilter;

    // Equivalent of hkCollisionListener used by the original game.
    class PhysicsFXContactListener : public JPH::ContactListener {
    public:
        JPH::ValidateResult OnContactValidate(
            const JPH::Body &inBody1,
            const JPH::Body &inBody2,
            JPH::RVec3Arg,
            const JPH::CollideShapeResult &inCollisionResult ) override {
            // FUN_00833BA0
            if ( inBody1.IsSensor() || inBody2.IsSensor() ) {
                return JPH::ValidateResult::AcceptContact;
            }
            if ( IsOwnBikeRiderPair( inBody1, inBody2 ) ) {
                return JPH::ValidateResult::RejectContact;
            }
            const bool bVehicle1 = IsVehicle( inBody1 );
            if ( bVehicle1 == IsVehicle( inBody2 ) ) {
                return JPH::ValidateResult::AcceptContact;
            }
            const JPH::Body &world = bVehicle1 ? inBody2 : inBody1;
            const JPH::SubShapeID &subShapeID =
                bVehicle1 ? inCollisionResult.mSubShapeID2
                          : inCollisionResult.mSubShapeID1;
            const JPH::Shape *pShape = world.GetShape();
            if ( pShape == nullptr ) {
                return JPH::ValidateResult::AcceptContact;
            }
            const uint32_t material = TDUPhysicsMaterials::codeOf(
                pShape->GetMaterial( subShapeID ) );
            if ( material == static_cast<uint32_t>( eMaterialType::MT_SP1 ) ) {
                return JPH::ValidateResult::RejectContact;
            }

            const JPH::Body &vehicle = bVehicle1 ? inBody1 : inBody2;
            const JPH::Vec3 towardVehicle =
                bVehicle1 ? -inCollisionResult.mPenetrationAxis
                          : inCollisionResult.mPenetrationAxis;
            return IsRoadContactIgnored(
                       vehicle, world,
                       towardVehicle.NormalizedOr( JPH::Vec3::sZero() ) )
                       ? JPH::ValidateResult::RejectContact
                       : JPH::ValidateResult::AcceptContact;
        }

        void OnContactAdded( const JPH::Body &inBody1,
                             const JPH::Body &inBody2,
                             const JPH::ContactManifold &inManifold,
                             JPH::ContactSettings &ioSettings ) override {
            if ( inBody1.IsSensor() || inBody2.IsSensor() ) {
                onSensorContactAdded( inBody1, inBody2 );
                return;
            }
            processVehicleContact( inBody1, inBody2, inManifold, ioSettings );
            for ( uint32_t i = 0;
                  i < inManifold.mRelativeContactPointsOn1.size(); i++ ) {
                if ( IsVehicle( inBody1 ) ) {
                    queueVehicleCollision( inBody1, inBody2, inManifold, i,
                                           true );
                }
                if ( IsVehicle( inBody2 ) ) {
                    queueVehicleCollision( inBody2, inBody1, inManifold, i,
                                           false );
                }
            }
            emit( inBody1, inBody2, inManifold, PFXEK_Impact );
            onBikeRiderContact( inBody1, inBody2 );
        }

        void OnContactRemoved( const JPH::SubShapeIDPair &inPair ) override {
            RoadPhantomManager &phantoms = gpPhysics->getRoadPhantomManager();
            phantoms.onOverlapRemoved( inPair.GetBody1ID(),
                                       inPair.GetBody2ID() );
            phantoms.onOverlapRemoved( inPair.GetBody2ID(),
                                       inPair.GetBody1ID() );
        }

        void OnContactPersisted( const JPH::Body &inBody1,
                                 const JPH::Body &inBody2,
                                 const JPH::ContactManifold &inManifold,
                                 JPH::ContactSettings &ioSettings ) override {
            // A contact that survives across steps is a scrape/slide, which is
            // what the original's looping kind describes.
            if ( inBody1.IsSensor() || inBody2.IsSensor() ) {
                return;
            }
            processVehicleContact( inBody1, inBody2, inManifold, ioSettings );
            emit( inBody1, inBody2, inManifold, PFXEK_Slide );
            onBikeRiderContact( inBody1, inBody2 );
        }

    private:
        static void onSensorContactAdded( const JPH::Body &inBody1,
                                          const JPH::Body &inBody2 ) {
            // FUN_0081a0f0
            const bool bPhantom1 =
                ( inBody1.GetUserData() & TDUSurface::kClassMask ) ==
                TDUSurface::RoadPhantom;
            const JPH::Body &phantom = bPhantom1 ? inBody1 : inBody2;
            const JPH::Body &other = bPhantom1 ? inBody2 : inBody1;
            if ( ( phantom.GetUserData() & TDUSurface::kClassMask ) !=
                     TDUSurface::RoadPhantom ||
                 other.IsStatic() || !IsVehicle( other ) ) {
                return;
            }
            void *pOwner = reinterpret_cast<void *>( other.GetUserData() >>
                                                     TDUSurface::kOwnerShift );
            if ( pOwner != nullptr ) {
                gpPhysics->getRoadPhantomManager().onOverlapAdded(
                    phantom.GetID(), other.GetID(), pOwner );
            }
        }

        static PhysicsVehicle *GetBikeRiderOwner( const JPH::Body &body ) {
            if ( ( body.GetUserData() & TDUSurface::kClassMask ) !=
                 TDUSurface::BikeRider ) {
                return nullptr;
            }
            return reinterpret_cast<PhysicsVehicle *>(
                body.GetUserData() >> TDUSurface::kOwnerShift );
        }

        // The rider body shares its bike's collision filter in retail.
        static bool IsOwnBikeRiderPair( const JPH::Body &inBody1,
                                        const JPH::Body &inBody2 ) {
            PhysicsVehicle *pOwner1 = GetBikeRiderOwner( inBody1 );
            PhysicsVehicle *pOwner2 = GetBikeRiderOwner( inBody2 );
            if ( pOwner1 == nullptr && pOwner2 == nullptr ) {
                return false;
            }
            const JPH::Body &other = ( pOwner1 != nullptr ) ? inBody2 : inBody1;
            PhysicsVehicle *pOwner = ( pOwner1 != nullptr ) ? pOwner1 : pOwner2;
            return IsVehicle( other ) &&
                   gpPhysics->findVehicle( other ) == pOwner;
        }

        static void onBikeRiderContact( const JPH::Body &inBody1,
                                        const JPH::Body &inBody2 ) {
            // FUN_0082d570
            PhysicsVehicle *pOwner1 = GetBikeRiderOwner( inBody1 );
            PhysicsVehicle *pOwner2 = GetBikeRiderOwner( inBody2 );
            if ( pOwner1 == nullptr && pOwner2 == nullptr ) {
                return;
            }
            const JPH::Body &other = ( pOwner1 != nullptr ) ? inBody2 : inBody1;
            const uint64_t physType =
                other.GetUserData() & TDUSurface::kClassMask;
            if ( physType > 0 && physType <= 6 ) {
                ( ( pOwner1 != nullptr ) ? pOwner1 : pOwner2 )
                    ->notifyBikeRiderGroundContact();
            }
        }

        static float ContactInvInertiaScale( const JPH::Body &body ) {
            static constexpr float kInertia = 20000.0f; // DAT_00fab808
            const JPH::MotionProperties *pMotion = body.GetMotionProperties();
            if ( pMotion == nullptr ) {
                return 1.0f;
            }
            const float minInvInertia =
                pMotion->GetInverseInertiaDiagonal().ReduceMin();
            if ( minInvInertia <= 0.0f ) {
                return 1.0f;
            }
            return Min( 1.0f, 1.0f / ( kInertia * minInvInertia ) );
        }

        static void
        requestCollisionInertia( const JPH::Body &body,
                                 PhysicsVehicle &vehicle,
                                 const bool bBody1,
                                 JPH::ContactSettings &ioSettings ) {
            if ( !vehicle.requestCollisionInertia() ) {
                return;
            }
            const float scale = ContactInvInertiaScale( body );
            ( bBody1 ? ioSettings.mInvInertiaScale1
                     : ioSettings.mInvInertiaScale2 ) = scale;
        }

        static void
        applyVehicleContactProperties( const JPH::Body &self,
                                       const JPH::SubShapeID &selfSubShape,
                                       const JPH::Body &other,
                                       const bool bSelfIsBody1,
                                       JPH::ContactSettings &ioSettings ) {
            // FUN_0082e740
            static constexpr float kUprightMinY = 0.5f;        // FLOAT_00f41674
            static constexpr float kBodyFriction = 115.2f;     // DAT_00f41df0
            static constexpr float kBikeBodyFriction = 51.2f;  // DAT_00f41df4
            static constexpr float kGenericRestitution = 6.4f; // DAT_00f41dec
            static constexpr float kVehicleRestitution = 32.0f; // DAT_00f41ae8

            PhysicsVehicle *pSelf = gpPhysics->findVehicle( self );
            if ( pSelf == nullptr ) {
                return;
            }

            const uint64_t otherType =
                other.GetUserData() & TDUSurface::kClassMask;
            if ( otherType >= TDUSurface::Generic &&
                 otherType <= TDUSurface::Vehicle ) {
                if ( otherType == TDUSurface::Vehicle ) {
                    PhysicsVehicle *pOther = gpPhysics->findVehicle( other );
                    if ( pOther != nullptr ) {
                        requestCollisionInertia( other, *pOther, !bSelfIsBody1,
                                                 ioSettings );
                    }
                }
                requestCollisionInertia( self, *pSelf, bSelfIsBody1,
                                         ioSettings );
            }

            // m_userData bit 31 (FUN_0082f030): a contact on a wheel sphere
            // grips with the brake pedal below 10 m/s.
            JPH::SubShapeID remainder;
            if ( PhysicsVehicle::IsWheelSphere( self.GetShape()->GetLeafShape(
                     selfSubShape, remainder ) ) ) {
                static constexpr float kGripSpeed = 10.0f; // FLOAT_00f4166c
                const float grip =
                    Clamp( kGripSpeed - self.GetLinearVelocity().Length(), 0.0f,
                           kGripSpeed );
                ioSettings.mCombinedFriction =
                    QuantizeFriction( pSelf->getInputBrake() * grip * 256.0f );
                ioSettings.mCombinedRestitution = 0.0f;
                return;
            }

            // Havok also flattens near-vertical Generic contact normals here;
            // Jolt exposes no contact normal to edit.
            float friction = 0.0f;
            float restitution = 0.0f;
            switch ( otherType ) {
            case TDUSurface::RoadGround:
            case TDUSurface::Paved:
            case TDUSurface::RoadMesh: {
                const float upY =
                    ( self.GetRotation() * JPH::Vec3::sAxisY() ).GetY();
                if ( !pSelf->areWheelsActive() ||
                     std::fabs( upY ) <= kUprightMinY ) {
                    friction =
                        pSelf->isBike() ? kBikeBodyFriction : kBodyFriction;
                }
                break;
            }
            case 6:
                break;
            case TDUSurface::Generic:
                friction = kBodyFriction;
                restitution = kGenericRestitution;
                break;
            case 8:
                restitution = kGenericRestitution;
                break;
            case TDUSurface::Vehicle:
                restitution = kVehicleRestitution;
                break;
            default:
                // Case 4 (friction 89.6 unless GrassAsTarmac) never runs:
                // the retail heightfield carries no phys-type property.
                return;
            }
            ioSettings.mCombinedFriction = QuantizeFriction( friction );
            ioSettings.mCombinedRestitution =
                QuantizeRestitution( restitution );
        }

        static void
        processVehicleContact( const JPH::Body &inBody1,
                               const JPH::Body &inBody2,
                               const JPH::ContactManifold &inManifold,
                               JPH::ContactSettings &ioSettings ) {
            if ( IsVehicle( inBody1 ) ) {
                applyVehicleContactProperties( inBody1, inManifold.mSubShapeID1,
                                               inBody2, true, ioSettings );
            }
            if ( IsVehicle( inBody2 ) ) {
                applyVehicleContactProperties( inBody2, inManifold.mSubShapeID2,
                                               inBody1, false, ioSettings );
            }
        }

        static bool IsVehicle( const JPH::Body &body ) {
            return ( body.GetUserData() & TDUSurface::kClassMask ) ==
                   TDUSurface::Vehicle;
        }

        static const CarState *FindCarState( const JPH::Body &chassis ) {
            if ( gpCarState == nullptr ) {
                return nullptr;
            }
            const PhysicsVehicle *pVehicle = gpPhysics->findVehicle( chassis );
            return ( pVehicle != nullptr )
                       ? gpCarState->findCarState( pVehicle )
                       : nullptr;
        }

        static float GetCollisionMass( const JPH::Body &body ) {
            // FUN_0047aa70
            const JPH::MotionProperties *pMotion = body.GetMotionProperties();
            if ( !body.IsDynamic() || pMotion == nullptr ||
                 pMotion->GetInverseMass() == 0.0f ) {
                return 0.0f;
            }
            return 1.0f / pMotion->GetInverseMass();
        }

        static Eigen::Vector4f ToVehicleSpace( const PhysicsVehicle &vehicle,
                                               const JPH::Vec3 &v,
                                               const float w ) {
            const Eigen::Vector4f row( v.GetX(), v.GetY(), v.GetZ(), w );
            return ( row.transpose() * vehicle.getInvChassisMatrix() )
                .transpose();
        }

        static void queueVehicleCollision( const JPH::Body &self,
                                           const JPH::Body &other,
                                           const JPH::ContactManifold &manifold,
                                           const uint32_t pointIndex,
                                           const bool bSelfIsBody1 ) {
            // FUN_0082f030
            static std::atomic<int32_t> sEventID{ 0 }; // DAT_010e7670

            PhysicsVehicle *pSelf = gpPhysics->findVehicle( self );
            if ( pSelf == nullptr ) {
                return;
            }

            const JPH::Vec3 towardSelf = bSelfIsBody1
                                             ? -manifold.mWorldSpaceNormal
                                             : manifold.mWorldSpaceNormal;
            const JPH::Vec3 position = JPH::Vec3(
                bSelfIsBody1
                    ? manifold.GetWorldSpaceContactPointOn1( pointIndex )
                    : manifold.GetWorldSpaceContactPointOn2( pointIndex ) );
            const JPH::Vec3 selfVelocity = self.GetLinearVelocity();
            const JPH::Vec3 otherVelocity = other.GetLinearVelocity();

            const float relativeVelocity =
                towardSelf.Dot( otherVelocity - selfVelocity );
            const float energy =
                relativeVelocity * relativeVelocity *
                ( GetCollisionMass( other ) + GetCollisionMass( self ) ) * 0.5f;

            const JPH::SubShapeID &selfSubShape =
                bSelfIsBody1 ? manifold.mSubShapeID1 : manifold.mSubShapeID2;
            const JPH::SubShapeID &otherSubShape =
                bSelfIsBody1 ? manifold.mSubShapeID2 : manifold.mSubShapeID1;

            CollisionEvent event;
            event.EventID = sEventID.fetch_add( 1 );
            event.RelativeVelocity = relativeVelocity;
            event.Energy = energy;
            event.CarMaterial = static_cast<eMaterialType>(
                ResolveMaterial( self, selfSubShape ) );
            event.OtherMaterial = static_cast<eMaterialType>(
                ResolveMaterial( other, otherSubShape ) );
            event.Velocity = ToVehicleSpace( *pSelf, selfVelocity, 0.0f );
            event.ContactPosition = ToVehicleSpace( *pSelf, position, 1.0f );

            PhysicsVehicle *pOther = nullptr;
            switch ( other.GetUserData() & TDUSurface::kClassMask ) {
            case TDUSurface::Generic:
                event.Type = eCollisionType::CT_RigidBody;
                event.pOtherBody = &other;
                break;
            case TDUSurface::Vehicle:
                pOther = gpPhysics->findVehicle( other );
                event.Type = eCollisionType::CT_Vehicle;
                event.pOtherVehicle = pOther;
                break;
            default:
                break;
            }
            pSelf->pushCollisionEvent( event );

            if ( pOther == nullptr ) {
                return;
            }
            // FUN_00827c80
            CollisionEvent mirror = event;
            mirror.pOtherVehicle = pSelf;
            mirror.RelativeVelocity = 0.0f - relativeVelocity;
            mirror.CarMaterial = event.OtherMaterial;
            mirror.OtherMaterial = event.CarMaterial;
            mirror.Velocity = ToVehicleSpace( *pSelf, otherVelocity, 0.0f );
            mirror.ContactPosition = ToVehicleSpace( *pOther, position, 1.0f );
            pOther->pushCollisionEvent( mirror );
        }

        static bool IsRoadContactIgnored( const JPH::Body &vehicle,
                                          const JPH::Body &world,
                                          const JPH::Vec3 &normal ) {
            // FUN_0082f030
            if ( ( world.GetUserData() & TDUSurface::kClassMask ) !=
                 TDUSurface::RoadMesh ) {
                return false;
            }
            if ( normal.GetY() < 0.0f ||
                 normal.GetY() < 0.75f ) { // DAT_00f416a4
                return false;
            }
            const CarState *pCarState = FindCarState( vehicle );
            if ( pCarState != nullptr &&
                 pCarState->getBody()->NumWheelsOnRoad ==
                     static_cast<int32_t>(
                         pCarState->getPhysicsVehicle()->getNumWheels() ) ) {
                return false;
            }
            return true;
        }

        static int32_t ResolveMaterial( const JPH::Body &body,
                                        const JPH::SubShapeID &subShapeID ) {
            uint32_t code = TDUMaterialValue::kDefault;

            const JPH::Shape *pShape = body.GetShape();
            if ( pShape != nullptr ) {
                code = TDUPhysicsMaterials::codeOf(
                    pShape->GetMaterial( subShapeID ) );
            }

            if ( code == TDUMaterialValue::kDefault &&
                 ( body.GetUserData() & TDUSurface::kClassMask ) ==
                     TDUSurface::Vehicle ) {
                code = static_cast<uint32_t>( eMaterialType::MT_Body );
            }

            return static_cast<int32_t>( code );
        }

        static float GetMass( const JPH::Body &body ) {
            // NOTE: Static and kinematic bodies should report 0
            if ( !body.IsDynamic() ) {
                return 0.0f;
            }
            const JPH::MotionProperties *pMotion = body.GetMotionProperties();
            if ( pMotion == nullptr ) {
                return 0.0f;
            }
            const float inverseMass = pMotion->GetInverseMass();
            return ( inverseMass > 0.0f ) ? ( 1.0f / inverseMass ) : 0.0f;
        }

        static Eigen::Vector4f ToVector4( const JPH::Vec3 &v, float w = 0.0f ) {
            return Eigen::Vector4f( v.GetX(), v.GetY(), v.GetZ(), w );
        }

        static void emit( const JPH::Body &inBody1,
                          const JPH::Body &inBody2,
                          const JPH::ContactManifold &inManifold,
                          ePhysicsFXEventKind kind ) {
            if ( gpPhysicsFX == nullptr || !gpPhysicsFX->isReady() ) {
                return;
            }
            if ( inManifold.mRelativeContactPointsOn1.empty() ) {
                return;
            }

            const int32_t materialA =
                ResolveMaterial( inBody1, inManifold.mSubShapeID1 );
            const int32_t materialB =
                ResolveMaterial( inBody2, inManifold.mSubShapeID2 );
            const int32_t fxMaterialA = GSPhysicsFX::RemapMaterial( materialA );
            const int32_t fxMaterialB = GSPhysicsFX::RemapMaterial( materialB );

            if ( !gpPhysicsFX->hasEffectFor( fxMaterialA, fxMaterialB,
                                             kind ) ) {
                OTDU_LOG_ERROR( "No entry: mat %d/%d -> fx %d/%d kind %d "
                                "(userData %llu/%llu)!!\n",
                                materialA, materialB, fxMaterialA, fxMaterialB,
                                int32_t( kind ),
                                (unsigned long long)inBody1.GetUserData(),
                                (unsigned long long)inBody2.GetUserData() );
                return;
            }

            constexpr int32_t kFXAsphalt = 0;
            constexpr int32_t kFXChassis = 14;
            constexpr float kChassisMaxSeparation = 0.01f;
            const bool bAsphaltChassis =
                ( fxMaterialA == kFXAsphalt && fxMaterialB == kFXChassis ) ||
                ( fxMaterialA == kFXChassis && fxMaterialB == kFXAsphalt );
            if ( bAsphaltChassis &&
                 !( -inManifold.mPenetrationDepth <= kChassisMaxSeparation ) ) {
                return;
            }

            const JPH::RVec3 contactPoint =
                inManifold.GetWorldSpaceContactPointOn1( 0 );

            const JPH::Vec3 velocity1 =
                inBody1.GetPointVelocity( contactPoint );
            const JPH::Vec3 velocity2 =
                inBody2.GetPointVelocity( contactPoint );

            const float energy = std::fabs(
                ( velocity1 - velocity2 ).Dot( inManifold.mWorldSpaceNormal ) );

            PhysicsFXEvent event;
            event.FXMaterialA = fxMaterialA;
            event.FXMaterialB = fxMaterialB;
            event.pBodyA = &inBody1;
            event.pBodyB = &inBody2;
            event.Kind = kind;
            event.Energy = energy;

            event.MassA = GetMass( inBody1 );
            event.MassB = GetMass( inBody2 );
            const float heaviest =
                ( event.MassA > event.MassB ) ? event.MassA : event.MassB;
            if ( event.MassA == 0.0f ) {
                event.MassA = heaviest;
            }
            if ( event.MassB == 0.0f ) {
                event.MassB = heaviest;
            }

            event.VelocityA = ToVector4( velocity1 );
            event.VelocityB = ToVector4( velocity2 );

            const Eigen::Vector4f shiftOrigin = GSPhysics::GetShiftOrigin();
            event.ContactPosition =
                ToVector4( JPH::Vec3( contactPoint ), 1.0f ) + shiftOrigin;
            event.ContactPosition.w() = 1.0f;
            event.Normal = ToVector4( inManifold.mWorldSpaceNormal );

            gpPhysicsFX->getEventQueue().push( event );
        }
    };
} // namespace

GSPhysics gPhysics = {}; // DAT_0129c0ac
GSPhysics *gpPhysics = nullptr;

bool gDrawCollisionVis = false;
bool gDrawCollisionVisTerrain = false;
bool gDrawVehicleCollisionVis = false;
bool gDrawVehicleCoMVis = false;

static DebugGraph gPhysicsGraph; // DAT_00fa951c

ePhysicsThreadMode gPhysicsThreadPriority = PTM_Off; // DAT_010e765c
int32_t gPhysicsThreadFrequency = 0x64;              // DAT_00f87ffc

int32_t DAT_010e7640 = 0;        // DAT_010e7640
int32_t INT_00f87fec = 0x200;    // DAT_00f87fec
int32_t INT_00f87ff4 = 0x500000; // DAT_00f87ff4
int32_t INT_00f87ff8 = 0x1;      // INT_00f87ff8
bool BOOL_010e7644 = false;      // DAT_010e7644

float gPhysicsRefreshRate = 0.0f;                          // DAT_0168d6b4
static float gPhysicsStepPeriod = 0.0f;                    // DAT_0168d6b8
static bool gbPhysicsUseMeasuredDelta = true;              // DAT_00f8a2e8
static std::atomic<bool> gbPhysicsTickInProgress{ false }; // DAT_010e7a58+0x1b0

static int32_t DAT_010e7668 = 0; // DAT_010e7668

static int32_t gGMeterAvailableBits = 0; // DAT_016a9878

bool gbShowGMeter = false;                            // DAT_010e7620
static Device *gpGMeterKeyboard = nullptr;            // DAT_016a97d0
static InputChannel *gpChannelRightControl = nullptr; // DAT_016a97cc
static InputChannel *gpChannelLeftControl = nullptr;  // DAT_016a97c8
static InputChannel *gpChannelG = nullptr;            // DAT_016a97c4

// Global time multiplier (1.0 = real-time; < 1.0 = slow-mo).
static float gPhysicsTimeScale = 1.0f; // DAT_00f8a2ec

// TEMP bring-up diagnostic counters for sector collision orientation.
static int32_t gTallBodyCount = 0;
static int32_t gFlatBodyCount = 0;
static float gMaxBodyY = 0.0f;

// If incoming dt exceeds this, clamp to kPhysicsMaxDt.
static constexpr float kPhysicsDtClampThreshold =
    1.0f / 45.0f; // DAT_00f41c08 (0x3cb60b61)

// Clamped dt value used when the step would be too large.
static constexpr float kPhysicsMaxDt =
    1.0f / 90.0f; // DAT_00f41c04 (0x3c360b61)

// Fixed-step cap in GSPhysics::Tick's no-physics-thread path.
static constexpr int32_t kMaxPhysicsSubSteps = 5;

constexpr uint32_t kSR20Magic = 0x30325253; // "SR20"
constexpr uint32_t kER20Magic = 0x30325245; // "ER20"
constexpr int32_t kLongSteps = 10; // Bezier longitudinal divisions/record

static constexpr float kHeightmapSeaLevel = -1.0f;     // FLOAT_00f41664
static constexpr float kHeightmapFlatThreshold = 0.1f; // FLOAT_00f41668
static constexpr float kTileProbeHalfHeight = 25.0f;
static constexpr float kSupportProbeRise = 3.0f;
static constexpr float kSupportProbeDepth = 40.0f;

static void FUN_00607370() {
    // FUN_00607370
    // Some kind of global linked list to keep track of mutexes?
    // Does not seem to be useful at runtime (so it could be a debug leftover)
    OTDU_UNIMPLEMENTED;
}

static void FUN_006072c0() {
    // FUN_006072c0
    // Same as FUN_00607370 (registers to the same global list) but registers a
    // thread
    OTDU_UNIMPLEMENTED;
}

using PhysicsClock_t = std::chrono::steady_clock;

// Replaces raw QueryPerformanceCounter calls from the original pcode
static PhysicsClock_t::time_point gPhysicsLastStepTime = {};
static bool gbPhysicsLastStepTimeValid = false;

void FUN_0081ee50( GSPhysics *pPhysics ) {
    // FUN_0081ee50
    if ( gbPhysicsTickInProgress.exchange( true ) ) {
        return;
    }

    const PhysicsClock_t::time_point now = PhysicsClock_t::now();
    if ( !gbPhysicsLastStepTimeValid ) {
        gPhysicsLastStepTime = now;
        gbPhysicsLastStepTimeValid = true;
    }

    const float elapsed =
        std::chrono::duration<float>( now - gPhysicsLastStepTime ).count();

    if ( gPhysicsStepPeriod * 0.5f <= elapsed ) {
        gPhysicsLastStepTime = now;

        if ( !pPhysics->bFreezeSimulation ) {
            float stepDelta = 0.0f;
            if ( !pPhysics->bPausePhysicsThread && !pPhysics->bPauseJoltTick ) {
                stepDelta =
                    gbPhysicsUseMeasuredDelta ? elapsed : gPhysicsStepPeriod;
            }

            pPhysics->physicsStep( stepDelta, 0.0f );
        }
    }

    gbPhysicsTickInProgress = false;
}

void *FUN_0081f0b0( void *pArg ) {
    // FUN_0081f0b0
    GSPhysics *pPhysics = reinterpret_cast<GSPhysics *>( pArg );

    gPhysicsRefreshRate = 1.0f / static_cast<float>( gPhysicsThreadFrequency );
    gPhysicsStepPeriod = 1.0f / static_cast<float>( gPhysicsThreadFrequency );

    gbPhysicsTickInProgress = false;
    gbPhysicsLastStepTimeValid = false;

    const auto period = std::chrono::duration_cast<PhysicsClock_t::duration>(
        std::chrono::duration<float>( gPhysicsStepPeriod ) );

    PhysicsClock_t::time_point nextWake = PhysicsClock_t::now() + period;
    while ( true ) {
        std::this_thread::sleep_until( nextWake );
        nextWake += period;

        const PhysicsClock_t::time_point now = PhysicsClock_t::now();
        if ( nextWake < now ) {
            nextWake = now + period;
        }

        if ( pPhysics->field27335_0x8670 == (void *)1 ||
             pPhysics->simulationThread.isStopRequested() ) {
            pPhysics->field27335_0x8670 = (void *)2;
            break;
        }

        FUN_0081ee50( pPhysics );
    }

    return nullptr;
}

static void *
JoltReallocate( void *inBlock, size_t inOldSize, size_t inNewSize ) {
    return TestDrive::Realloc( inBlock, inNewSize );
}

struct PropConstraintRecord {
    JPH::Ref<JPH::FixedConstraint> pConstraint;
    JPH::BodyID BodyA;
    JPH::BodyID BodyB;
    float Threshold = 0.0f;
    bool bBreakable = false;
    bool bBroken = false;
};
std::vector<PropConstraintRecord> gPropConstraints;

// Shared collision filter: members of the same placed instance (same group
// id) never collide with each other while joined; different instances (and
// the car/world, which carry no filter) collide normally.
JPH::Ref<JPH::GroupFilterTable> gPropGroupFilter;
uint32_t gNextPropGroupID = 1;

// Max members of one prop instance that get mutually-disabled self-collision
// (273 member bodies / 93 mechas ~= 3 each; 32 is a safe ceiling).
constexpr uint32_t kMaxPropMembers = 32;
constexpr float kBreakThresholdScale = 1.0f;

JPH::GroupFilterTable *GetPropGroupFilter() {
    if ( gPropGroupFilter == nullptr ) {
        gPropGroupFilter = new JPH::GroupFilterTable( kMaxPropMembers );
        for ( uint32_t i = 0; i < kMaxPropMembers; i++ ) {
            for ( uint32_t j = i + 1; j < kMaxPropMembers; j++ ) {
                gPropGroupFilter->DisableCollision( i, j );
            }
        }
    }
    return gPropGroupFilter.GetPtr();
}

void BuildMechaMembers( const HavokContainer &container,
                        int32_t entryCount,
                        HavokFactoryData *pData ) {
    const uint8_t *pBuf = container.getVehicleBuffer();
    const uint32_t bufSize = container.getVehicleBufferSize();
    if ( pBuf == nullptr || bufSize < 0x40 ) {
        return; // no descriptors (e.g. a plain vehicle factory)
    }

    auto readU32 = [&]( uint32_t off ) -> uint32_t {
        return *reinterpret_cast<const uint32_t *>( pBuf + off );
    };

    uint32_t o = 0;
    bool bLayoutOk = true;

    while ( o + 0x40 <= bufSize ) {
        const uint32_t regIdx = readU32( o + 0x00 );
        const uint32_t flags = readU32( o + 0x04 );
        const uint32_t marker = readU32( o + 0x10 );

        if ( regIdx >= static_cast<uint32_t>( entryCount ) ||
             container.getEntry( static_cast<int32_t>( regIdx ) ).Type !=
                 ePhysEntityType::PET_Vehicle ||
             marker != 0xFFFFFFFFu ) {
            bLayoutOk = false;
            break;
        }

        const uint32_t rbCount = readU32( o + 0x30 );
        const uint32_t rbCount2 = readU32( o + 0x34 );
        const uint32_t refsOff = o + 0x40;
        if ( rbCount > 4096 || rbCount2 > 4096 ||
             refsOff + ( static_cast<uint64_t>( rbCount ) + rbCount2 ) * 0x10 >
                 bufSize ) {
            bLayoutOk = false;
            break;
        }

        std::vector<HavokToJolt::MechaMember> members;
        members.reserve( rbCount );
        for ( uint32_t r = 0; r < rbCount; r++ ) {
            HavokToJolt::MechaMember mm;
            mm.BodyHashcode = *reinterpret_cast<const uint64_t *>(
                pBuf + refsOff + r * 0x10 );
            members.push_back( std::move( mm ) );
        }

        const uint32_t mapOff = refsOff + rbCount * 0x10;
        for ( uint32_t m = 0; m < rbCount2; m++ ) {
            const uint32_t entryOff = mapOff + m * 0x10;
            const uint64_t logicID =
                *reinterpret_cast<const uint64_t *>( pBuf + entryOff + 0x00 );
            const uint32_t k = readU32( entryOff + 0x08 );
            if ( k < rbCount ) {
                members[k].LogicIDs.push_back( logicID );
            }
        }

        uint32_t next = refsOff + ( rbCount + rbCount2 ) * 0x10;
        if ( ( flags & 1u ) != 0 ) {
            if ( next + 0x10 > bufSize ) {
                bLayoutOk = false;
                break;
            }
            const uint32_t cCount = readU32( next + 0x00 );
            const uint32_t cCount2 = readU32( next + 0x04 );
            next += 0x10 + ( cCount + cCount2 ) * 0x10;
        }
        if ( next > bufSize ) {
            bLayoutOk = false;
            break;
        }

        const PhysFactory::Entry e =
            container.getEntry( static_cast<int32_t>( regIdx ) );
        if ( !members.empty() ) {
            pData->MechaMembers.emplace_back( e.Hashcode,
                                              std::move( members ) );
        }
        o = next;
    }

    if ( !bLayoutOk || o != bufSize ) {
        OTDU_LOG_WARN( "Prop factory: Mecha buffer parse drifted (consumed "
                       "%u/%u bytes)\n",
                       o, bufSize );
    }
}

GSPhysics::GSPhysics()
    : GameSystem(),
      Bank(),
      isLocked( 0 ),
      numTires( 0 ),
      numPhysicsVehicle( 0 ),
      pPhysicsVehicles( nullptr ),
      pPhysicsVehiclesBegin( nullptr ),
      pPhysicsVehiclesEnd( nullptr ),
      field_0x2cc( nullptr ),
      numFreeSectors( 0 ),
      maxNumSectors( 0 ),
      pSectors( nullptr ),
      pSectorsBegin( nullptr ),
      pSectorsEnd( nullptr ),
      numFreeRoadSectors( 0 ),
      maxNumRoadSectors( 0 ),
      pRoads( nullptr ),
      pRoadsBegin( nullptr ),
      pRoadsEnd( nullptr ),
      field27335_0x8670( nullptr ),
      field_0x31a0{ 0, 0 },
      field_0x4dc0{ 0, 0 },
      field_0x5bd0{ 0, 0 },
      field_0x69e0{ 0, 0 },
      field_0x8600{ 0, 0 },
      bJoltInit( false ),
      bFreezeSimulation( false ),
      bPauseJoltTick( false ),
      bHeightfieldInit( false ),
      bSectorsInit( false ),
      bHavokDebugEnabled( false ),
      bDrawDebugInfos( false ),
      debugDrawFlags( 0x200 ) {
    // FUN_008471e0
    // Original: initializes factories, Havok memory masters,
    // thread, mutex, and various flags. Havok memory subsystem init is not
    // needed for Jolt.
    pTireProfiles.fill( nullptr );

    for ( auto &factory : pFactories ) {
        factory.reset();
    }

    gpPhysics = this;
}

GSPhysics::~GSPhysics() {
    // FUN_00419310
    if ( gpWorldStreaming != nullptr ) {
        gpWorldStreaming->setPhysicsCallback( nullptr );
    }
    gpPhysics = nullptr;
}

bool GSPhysics::initialize( TestDriveGameInstance *param_2 ) {
    // FUN_0084d570
    pGameInstance = param_2;
    bJoltInit = false;
    bPauseJoltTick = true;
    bFreezeSimulation = true;
    field27335_0x8670 = nullptr;
    bPaused = false;

    if ( gpWorldStreaming != nullptr ) {
        gpWorldStreaming->setPhysicsCallback( this );
    }

    OTDU_LOG_INFO( "Intall BEGIN\n" ); // DAT_00f2794c

    // NOTE: Original P-Code initializes Havok-specific allocator
    // (hkMemoryRouter) to use the game allocator. This is the equivalent with
    // Jolt.
    JPH::Allocate = TestDrive::Alloc;
    JPH::Reallocate = JoltReallocate;
    JPH::Free = TestDrive::Free;
    JPH::AlignedAllocate = TestDrive::AllocAligned;
    JPH::AlignedFree = TestDrive::FreeAligned;

    OTDU_LOG_INFO( "Install Pacejaka Tire models\n" ); // DAT_00f2792c

    std::string acStack_200 = gpConfig->getResRootPath();
    acStack_200 += "Physics/tires.bnk";

    loadBootBank( acStack_200.c_str(), nullptr );
    numTires = 0;

    if ( bLoaded ) {
        void *peVar4 = getFirstEntry( AG_Physics, AT_TireProfile, nullptr );
        while ( peVar4 != nullptr ) {
            pTireProfiles[numTires] = (TirePacejka *)peVar4;
            numTires++;

            peVar4 = getNextEntry( AG_Physics, AT_TireProfile, nullptr );
        }
    }

    OTDU_LOG_INFO( "Initialise Physics Cars\n" ); // DAT_00f27900

    numPhysicsVehicle = kMaxNumPhysicsVehicles;
    pPhysicsVehicles =
        new ( TestDrive::Alloc( sizeof( PhysicsVehicle ) * numPhysicsVehicle ) )
            PhysicsVehicle[numPhysicsVehicle];
    pPhysicsVehiclesBegin = pPhysicsVehicles;
    pPhysicsVehiclesEnd = pPhysicsVehicles + numPhysicsVehicle;
    field_0x2cc = nullptr;
    for ( auto it = pPhysicsVehiclesBegin; it != pPhysicsVehiclesEnd; it++ ) {
        it->initialize();
    }

    OTDU_LOG_INFO( "HeightFields Creation\n" ); // DAT_00f278e8
    bHeightfieldInit = true;
    createHeightfields();

    OTDU_LOG_INFO( "Sectors Creation\n" ); // DAT_00f278d4

    for ( auto &factory : pFactories ) {
        factory.reset();
    }

    numFreeSectors = 0x40;
    maxNumSectors = 0x40;
    DAT_010e7640 = 0;

    pSectors = new ( TestDrive::Alloc( sizeof( PhysicsSHK ) * maxNumSectors ) )
        PhysicsSHK[maxNumSectors];
    pSectorsBegin = pSectors;
    pSectorsEnd = pSectors + maxNumSectors;

    for ( auto it = pSectorsBegin; it != pSectorsEnd; it++ ) {
        it->MaxNumObjects = kMaxNumObjectsPerSector;
        it->NumObjects = 0;
        it->pObjects = new ( TestDrive::Alloc( sizeof( PhysicsSectorObj ) *
                                               kMaxNumObjectsPerSector ) )
            PhysicsSectorObj[kMaxNumObjectsPerSector];
        OTDU_ASSERT( it->pObjects );

        it->pObjectsBegin = it->pObjects;
        it->pObjectsEnd = it->pObjects + it->MaxNumObjects;
        it->pFilestream = nullptr;
    }

    OTDU_LOG_INFO( "Road Sectors Creation\n" ); // DAT_00f278bc
    maxNumRoadSectors = 7;
    numFreeRoadSectors = 7;

    pRoads =
        new ( TestDrive::Alloc( sizeof( PhysicsSHKRoad ) * maxNumRoadSectors ) )
            PhysicsSHKRoad[maxNumRoadSectors];
    pRoadsBegin = pRoads;
    pRoadsEnd = pRoads + maxNumRoadSectors;

    bSectorsInit = true;

    for ( auto it = pRoadsBegin; it != pRoadsEnd; it++ ) {
        it->MaxNumObjects = kMaxNumObjectsPerSector;
        it->NumObjects = 0;
        it->pObjects = new ( TestDrive::Alloc( sizeof( PhysicsRoadObj ) *
                                               kMaxNumObjectsPerSector ) )
            PhysicsRoadObj[kMaxNumObjectsPerSector];
        OTDU_ASSERT( it->pObjects );

        it->pObjectsBegin = it->pObjects;
        it->pObjectsEnd = it->pObjects + it->MaxNumObjects;
        it->pFilestream = nullptr;
    }

    initializeJolt();

    isLocked = 0;
    FUN_00607370();
    if ( !simulationThread.isValid() && gPhysicsThreadPriority != PTM_Off ) {
        simulationThread.setName( "PhysThread" );
        simulationThread.initialize(
            (TestDriveThread::StartRoutine_t)&FUN_0081f0b0, this, 0x40000, 0 );

        simulationThread.setPriority( TP_Critical );
        FUN_006072c0();
    }

    if ( bHavokDebugEnabled ) {
        // TODO: I think this is the logic to init/enable Havok Remote debugger
        // We could swap this with whatever debug features Jolt offers
        OTDU_UNIMPLEMENTED;
    }

    field_0x31a0[0] = 0;
    field_0x31a0[1] = 0;

    field_0x4dc0[0] = 0;
    field_0x4dc0[1] = 0;

    field_0x5bd0[0] = 0;
    field_0x5bd0[1] = 0;

    field_0x69e0[0] = 0;
    field_0x69e0[1] = 0;

    field_0x8600[0] = 0;
    field_0x8600[1] = 0;

    OTDU_LOG_INFO( "Road Phantom Manager Creation\n" ); // DAT_00f27884

    roadPhantomManager.initialize( 0x41a, 4 );

    createGraph();

    if ( bDrawDebugInfos ) {
        OTDU_UNIMPLEMENTED;
    }

    bFreezeSimulation = false;
    BOOL_010e7644 = true;

    return true;
}

void GSPhysics::terminate() {
    // FUN_00846cf0
    bFreezeSimulation = true;
    bPauseJoltTick = true;

    if ( gpWorldStreaming != nullptr ) {
        gpWorldStreaming->setPhysicsCallback( nullptr );
    }

    if ( gPhysicsThreadPriority != PTM_Off && simulationThread.isValid() ) {
        field27335_0x8670 = (void *)1;
        while ( field27335_0x8670 != (void *)2 ) {
            std::this_thread::yield();
        }
        simulationThread.terminate();
    }

    gPhysicsGraph.release();
    if ( pGraphDrawList != nullptr ) {
        pGraphDrawList->destroy();
        delete pGraphDrawList;
        pGraphDrawList = nullptr;
    }

#if OTDU_DEVBUILD
    if ( pCollisionDebugList != nullptr ) {
        pCollisionDebugList->destroy();
        delete pCollisionDebugList;
        pCollisionDebugList = nullptr;
    }

    if ( pVehicleDebugList != nullptr ) {
        pVehicleDebugList->destroy();
        delete pVehicleDebugList;
        pVehicleDebugList = nullptr;
    }
#endif

    // Reset all factories (destroy any Havok/physics objects they hold)
    for ( auto &factory : pFactories ) {
        factory.reset();
    }

    // Free sector collision object arrays
    if ( pSectors != nullptr ) {
        for ( auto it = pSectorsBegin; it != pSectorsEnd; it++ ) {
            if ( it->pObjects != nullptr ) {
                it->NumObjects = 0;
                it->MaxNumObjects = 0;
                TestDrive::Free( it->pObjects );
                it->pObjects = nullptr;
                it->pObjectsBegin = nullptr;
                it->pObjectsEnd = nullptr;
            }
        }
        TestDrive::Free( pSectors );
        pSectors = nullptr;
        pSectorsBegin = nullptr;
        pSectorsEnd = nullptr;
    }
    maxNumSectors = 0;
    numFreeSectors = 0;
    DAT_010e7640 = 0;

    // Free road sector collision object arrays
    if ( pRoads != nullptr ) {
        for ( auto it = pRoadsBegin; it != pRoadsEnd; it++ ) {
            if ( it->pObjects != nullptr ) {
                it->NumObjects = 0;
                it->MaxNumObjects = 0;
                TestDrive::Free( it->pObjects );
                it->pObjects = nullptr;
                it->pObjectsBegin = nullptr;
                it->pObjectsEnd = nullptr;
            }
        }
        TestDrive::Free( pRoads );
        pRoads = nullptr;
        pRoadsBegin = nullptr;
        pRoadsEnd = nullptr;
    }
    maxNumRoadSectors = 0;
    numFreeRoadSectors = 0;

    // Free physics car array
    if ( pPhysicsVehicles != nullptr ) {
        TestDrive::Free( pPhysicsVehicles );
        pPhysicsVehicles = nullptr;
        pPhysicsVehiclesBegin = nullptr;
        pPhysicsVehiclesEnd = nullptr;
    }
    numPhysicsVehicle = 0;

    // Remove and destroy Jolt bodies before tearing down the physics system
    if ( pPhysicsSystem != nullptr ) {
        JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();
        for ( auto &staticMesh : staticMeshBodies ) {
            JPH::Body *pBody = static_cast<JPH::Body *>( staticMesh.second );
            if ( pBody != nullptr ) {
                bodyInterface.RemoveBody( pBody->GetID() );
                bodyInterface.DestroyBody( pBody->GetID() );
            }
        }
        staticMeshBodies.clear();

        if ( worldRefBody != nullptr ) {
            JPH::Body *pBody = static_cast<JPH::Body *>( worldRefBody );
            bodyInterface.RemoveBody( pBody->GetID() );
            bodyInterface.DestroyBody( pBody->GetID() );
            worldRefBody = nullptr;
        }

        if ( infinitePlaneBodyID != JPH::BodyID::cInvalidBodyID ) {
            JPH::BodyID planeID( infinitePlaneBodyID );
            bodyInterface.RemoveBody( planeID );
            bodyInterface.DestroyBody( planeID );
            infinitePlaneBodyID = JPH::BodyID::cInvalidBodyID;
        }

        for ( auto &hf : pHeightfields ) {
            if ( hf.pBody != nullptr ) {
                JPH::Body *pBody = static_cast<JPH::Body *>( hf.pBody );
                bodyInterface.RemoveBody( pBody->GetID() );
                bodyInterface.DestroyBody( pBody->GetID() );
                hf.pBody = nullptr;
                hf.pKey = nullptr;
            }
        }

        pPhysicsSystem->SetContactListener( nullptr );

        delete pPhysicsSystem;
        pPhysicsSystem = nullptr;
    }

    delete pContactListener;
    pContactListener = nullptr;

    delete pJobSystem;
    pJobSystem = nullptr;

    delete pTempAllocator;
    pTempAllocator = nullptr;

    delete pBpLayerInterface;
    pBpLayerInterface = nullptr;

    delete pObjectVsBPLayerFilter;
    pObjectVsBPLayerFilter = nullptr;

    delete pObjectLayerPairFilter;
    pObjectLayerPairFilter = nullptr;

    JPH::UnregisterTypes();
    if ( JPH::Factory::sInstance != nullptr ) {
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }

    bJoltInit = false;
}

void GSPhysics::tick( float totalTime, float deltaTime ) {
    // FUN_0084de60

    // Pause toggle: a deltaTime of 0 is the engine's sentinel for "simulation
    // frozen" (paused frame / hitch with no elapsed time). The original tests
    // the delta argument, not the accumulated total.
    bPauseJoltTick = ( deltaTime == 0.0f );

    // Sub-stepped physics — only when no dedicated physics thread is running.
    // With PTM_Normal / PTM_Realtime the dedicated thread drives physicsStep.
    if ( gPhysicsThreadPriority == PTM_Off && gPhysicsThreadFrequency > 0 ) {
        const float stepDt =
            1.0f / static_cast<float>( gPhysicsThreadFrequency );

        int32_t numSteps = static_cast<int32_t>( ceilf( deltaTime / stepDt ) );
        float subStepDt = stepDt;
        if ( numSteps > kMaxPhysicsSubSteps ) {
            numSteps = kMaxPhysicsSubSteps;
        } else if ( numSteps > 0 ) {
            subStepDt = deltaTime / static_cast<float>( numSteps );
        }

        if ( numSteps <= 0 ) {
            physicsStep( deltaTime, totalTime );
        } else {
            float accTime = totalTime;
            for ( int32_t i = 0; i < numSteps; i++ ) {
                physicsStep( subStepDt, accTime );
                accTime += subStepDt;
            }
        }
    }

    // Input device wizard (FUN_00846590, "DevicePCNew.ini")
    if ( DAT_010e7668 != 0 ) {
        OTDU_UNIMPLEMENTED;
    }

    // G-meter overlay init (see FUN_009078f0)
    if ( ( gGMeterAvailableBits & 1 ) == 0 ) {
        gGMeterAvailableBits |= 1;
    }

    OTDU_UNIMPLEMENTED; // TODO: car tester state machine (DAT_010e7654 /
                        // DAT_010e7658, writes CarData.txt)

    updateGMeterToggle();

    if ( bHavokDebugEnabled ) {
        OTDU_UNIMPLEMENTED;
    }

    updateSecretIslandTeleporter();
}

static bool gbSecretIslandTeleporterEnabled = true;       // DAT_00fab50c
static constexpr float kSecretIslandTriggerRadius = 5.0f; // DAT_00f41750
static constexpr float kSecretIslandMinSpeedKmh = 50.0f;  // DAT_00fab508
static constexpr float kMsToKmh = 3.6f;                   // DAT_00f41768
static const Eigen::Vector4f kSecretIslandMainland( 63515.0f,
                                                    224.0f,
                                                    46303.0f,
                                                    1.0f ); // DAT_0168bf30
static const Eigen::Vector4f
    kSecretIslandIsland( 64426.0f, 4.0f, 44637.0f, 1.0f ); // DAT_0168bf40
static int32_t gSecretIslandTeleporterState = 0;           // DAT_016a2d48

static void PlaceAtSecretIslandEndpoint( PhysicsVehicle *pVehicle,
                                         Matrix4x4f_t &matrix,
                                         const Eigen::Vector4f &endpoint ) {
    matrix.row( 3 ) = endpoint.transpose();

    // FUN_0083d900
    PhysicsLock lock;
    pVehicle->resetToTransform( matrix, true );
}

static void SetSecretIslandPostponedPause( PhysicsVehicle *pVehicle,
                                           const uint8_t request ) {
    PhysicsLock lock;
    pVehicle->setPostponedPause( request );
}

void GSPhysics::updateSecretIslandTeleporter() {
    // FUN_0084de60 (tail)
    CarState *pCarState = gpScenaric != nullptr
                              ? gpScenaric->getLocalPlayer().getCarState()
                              : nullptr;
    if ( !gbSecretIslandTeleporterEnabled || pCarState == nullptr ) {
        return;
    }
    PhysicsVehicle *pVehicle = pCarState->getPhysicsVehicle();
    if ( pVehicle == nullptr || !pVehicle->isActive() ) {
        return;
    }

    Matrix4x4f_t matrix = pCarState->getBody()->WorldMatrix;
    const Eigen::Vector4f position = matrix.row( 3 ).transpose();
    const float distanceToMainland =
        ( position - kSecretIslandMainland ).norm();
    const float distanceToIsland = ( position - kSecretIslandIsland ).norm();

    switch ( gSecretIslandTeleporterState ) {
    case 0: {
        const float speedKmh =
            pVehicle->getChassisVelocity().head<3>().norm() * kMsToKmh;
        if ( distanceToMainland < kSecretIslandTriggerRadius &&
             kSecretIslandMinSpeedKmh < speedKmh ) {
            gbBackOnTrackFadeGuard = false;
            SetSecretIslandPostponedPause( pVehicle, 1 );
            pushFadeRequest( eFadeRequest::FR_FadeOut );
            gSecretIslandTeleporterState = 1;
        }
        if ( distanceToIsland < kSecretIslandTriggerRadius &&
             kSecretIslandMinSpeedKmh < speedKmh ) {
            gbBackOnTrackFadeGuard = false;
            SetSecretIslandPostponedPause( pVehicle, 1 );
            pushFadeRequest( eFadeRequest::FR_FadeOut );
            gSecretIslandTeleporterState = 4;
        }
    } break;
    case 1:
        if ( peekFadeRequest() == eFadeRequest::FR_Idle ) {
            PlaceAtSecretIslandEndpoint( pVehicle, matrix,
                                         kSecretIslandIsland );
            gSecretIslandTeleporterState = 2;
        }
        break;
    case 2:
        if ( gpWorld->isStreamedAround( &kSecretIslandIsland, false, false ) ) {
            gSecretIslandTeleporterState = 3;
            pushFadeRequest( eFadeRequest::FR_FadeIn );
        }
        break;
    case 3:
        PlaceAtSecretIslandEndpoint( pVehicle, matrix, kSecretIslandIsland );
        SetSecretIslandPostponedPause( pVehicle, 2 );
        gSecretIslandTeleporterState = 0;
        gbBackOnTrackFadeGuard = true;
        break;
    case 4:
        if ( peekFadeRequest() == eFadeRequest::FR_Idle ) {
            PlaceAtSecretIslandEndpoint( pVehicle, matrix,
                                         kSecretIslandMainland );
            gSecretIslandTeleporterState = 5;
        }
        break;
    case 5:
        if ( gpWorld->isStreamedAround( &kSecretIslandMainland, false,
                                        false ) ) {
            gSecretIslandTeleporterState = 6;
            pushFadeRequest( eFadeRequest::FR_FadeIn );
        }
        break;
    case 6:
        PlaceAtSecretIslandEndpoint( pVehicle, matrix, kSecretIslandMainland );
        SetSecretIslandPostponedPause( pVehicle, 2 );
        gSecretIslandTeleporterState = 0;
        gbBackOnTrackFadeGuard = true;
        break;
    default:
        break;
    }
}

void GSPhysics::updateGMeterToggle() {
    // FUN_0084de60
    if ( ( gGMeterAvailableBits & 2 ) == 0 ) {
        gGMeterAvailableBits |= 2;
        gpGMeterKeyboard = gEdDev.getDevice( eDeviceType::DT_Keyboard, 0 );
    }

    if ( gpGMeterKeyboard == nullptr ) {
        return;
    }

    if ( ( gGMeterAvailableBits & 4 ) == 0 ) {
        gGMeterAvailableBits |= 4;
        gpChannelRightControl = gpGMeterKeyboard->getChannel( "RCTRL" );
    }
    if ( ( gGMeterAvailableBits & 8 ) == 0 ) {
        gGMeterAvailableBits |= 8;
        gpChannelLeftControl = gpGMeterKeyboard->getChannel( "LCTRL" );
    }
    if ( ( gGMeterAvailableBits & 0x10 ) == 0 ) {
        gGMeterAvailableBits |= 0x10;
        gpChannelG = gpGMeterKeyboard->getChannel( "G" );
    }

    if ( gpChannelRightControl == nullptr || gpChannelLeftControl == nullptr ||
         gpChannelG == nullptr ) {
        return;
    }

    const bool bControlDown = ( gpChannelLeftControl->Flags & 0x10 ) != 0 ||
                              ( gpChannelRightControl->Flags & 0x10 ) != 0;
    if ( bControlDown && static_cast<int8_t>( gpChannelG->Flags ) < 0 ) {
        gbShowGMeter = !gbShowGMeter;
    }
}

#if OTDU_DEVBUILD
inline ColorRGBA PackDebugColor( uint8_t r, uint8_t g, uint8_t b ) {
    return ColorRGBA( b, g, r, 0xff );
}

// Per-collision-body color
static uint32_t CollisionBodyColor( JPH::EShapeSubType subType,
                                    JPH::ObjectLayer layer,
                                    uint64_t userData ) {
    if ( subType == JPH::EShapeSubType::HeightField ) {
        return PackDebugColor( 90, 90, 90 ).Color; // terrain: grey
    }
    if ( layer == TDUPhysLayers::MOVING || layer == TDUPhysLayers::VEHICLE ||
         layer == TDUPhysLayers::VEHICLE_GHOST ) {
        return PackDebugColor( 255, 140, 0 ).Color; // props / car: orange
    }
    switch ( userData & TDUSurface::kClassMask ) {
    case TDUSurface::RoadMesh:
    case TDUSurface::RoadGround:
        return PackDebugColor( 60, 120, 255 ).Color; // road mesh: blue
    case TDUSurface::Paved:
    case TDUSurface::Generic:
        return PackDebugColor( 40, 230, 40 ).Color; // world .shk: green
    default:
        return PackDebugColor( 200, 200, 40 ).Color; // other static: yellow
    }
}

constexpr uint32_t kMaxDebugLineVerts = 300000u; // 150k segments

void GSPhysics::drawCollisionShapes() {
    if ( pPhysicsSystem == nullptr || gpRender == nullptr ) {
        return;
    }

    struct BodySnapshot {
        JPH::RefConst<JPH::Shape> shape;
        JPH::Mat44 com;
        uint32_t color;
        bool bMesh;
    };
    std::vector<BodySnapshot> snapshots;

    lockMutex();
    JPH::BodyIDVector bodyIDs;
    pPhysicsSystem->GetBodies( bodyIDs );
    const JPH::BodyInterface &bi = pPhysicsSystem->GetBodyInterfaceNoLock();
    snapshots.reserve( bodyIDs.size() );
    for ( const JPH::BodyID id : bodyIDs ) {
        JPH::RefConst<JPH::Shape> shape = bi.GetShape( id );
        if ( shape.GetPtr() == nullptr ) {
            continue;
        }
        const JPH::EShapeSubType st = shape->GetSubType();
        if ( st == JPH::EShapeSubType::HeightField &&
             !gDrawCollisionVisTerrain ) {
            continue;
        }
        BodySnapshot s;
        s.shape = shape;
        s.com = bi.GetCenterOfMassTransform( id );
        s.color = CollisionBodyColor( st, bi.GetObjectLayer( id ),
                                      bi.GetUserData( id ) );
        s.bMesh = ( st == JPH::EShapeSubType::Mesh );
        snapshots.push_back( std::move( s ) );
    }
    unlockMutex();

    // Lazily create the dynamic line list.
    if ( pCollisionDebugList == nullptr ) {
        pCollisionDebugList = new DrawList();
        if ( !pCollisionDebugList->initialize( 1, kMaxDebugLineVerts, 0, true,
                                               nullptr, "CollisionDebug",
                                               false ) ) {
            pCollisionDebugList->destroy();
            delete pCollisionDebugList;
            pCollisionDebugList = nullptr;
            return;
        }
    }

    // Gather line-segment endpoints (render space) into a reusable buffer.
    struct LineVert {
        float x, y, z;
        uint32_t color;
    };
    static std::vector<LineVert> verts;
    verts.clear();

    auto pushSeg = [&]( const JPH::Vec3 &a, const JPH::Vec3 &b,
                        uint32_t color ) {
        if ( verts.size() + 2 > kMaxDebugLineVerts ) {
            return;
        }
        verts.push_back( { a.GetX(), a.GetY(), a.GetZ(), color } );
        verts.push_back( { b.GetX(), b.GetY(), b.GetZ(), color } );
    };

    for ( const BodySnapshot &s : snapshots ) {
        if ( verts.size() + 2 > kMaxDebugLineVerts ) {
            break;
        }
        if ( s.bMesh ) {
            // Actual triangle wireframe (buildings/walls/road surfaces).
            JPH::Shape::GetTrianglesContext ctx;
            s.shape->GetTrianglesStart(
                ctx, JPH::AABox::sBiggest(), s.com.GetTranslation(),
                s.com.GetQuaternion(), JPH::Vec3::sReplicate( 1.0f ) );
            constexpr int kBatch = 64;
            JPH::Float3 tri[3 * kBatch];
            for ( ;; ) {
                const int n = s.shape->GetTrianglesNext( ctx, kBatch, tri );
                if ( n == 0 ) {
                    break;
                }
                for ( int t = 0; t < n; t++ ) {
                    const JPH::Vec3 a( tri[3 * t + 0].x, tri[3 * t + 0].y,
                                       tri[3 * t + 0].z );
                    const JPH::Vec3 b( tri[3 * t + 1].x, tri[3 * t + 1].y,
                                       tri[3 * t + 1].z );
                    const JPH::Vec3 c( tri[3 * t + 2].x, tri[3 * t + 2].y,
                                       tri[3 * t + 2].z );
                    pushSeg( a, b, s.color );
                    pushSeg( b, c, s.color );
                    pushSeg( c, a, s.color );
                }
                if ( verts.size() + 2 > kMaxDebugLineVerts ) {
                    break;
                }
            }
        } else {
            // OBB from the shape's local bounds (posts, boxes, capsules,
            // props, car)
            const JPH::AABox lb = s.shape->GetLocalBounds();
            JPH::Vec3 corner[8];
            for ( int i = 0; i < 8; i++ ) {
                const JPH::Vec3 local(
                    ( i & 1 ) ? lb.mMax.GetX() : lb.mMin.GetX(),
                    ( i & 2 ) ? lb.mMax.GetY() : lb.mMin.GetY(),
                    ( i & 4 ) ? lb.mMax.GetZ() : lb.mMin.GetZ() );
                corner[i] = s.com * local;
            }
            static const int kEdges[12][2] = {
                {0, 1},
                {1, 3},
                {3, 2},
                {2, 0}, // bottom
                {4, 5},
                {5, 7},
                {7, 6},
                {6, 4}, // top
                {0, 4},
                {1, 5},
                {2, 6},
                {3, 7}
            }; // verticals
            for ( const auto &e : kEdges ) {
                pushSeg( corner[e[0]], corner[e[1]], s.color );
            }
        }
    }

    pCollisionDebugList->reset();
    if ( verts.empty() ) {
        return;
    }
    pCollisionDebugList->setIdentityTransform();
    if ( !pCollisionDebugList->beginPrimitive(
             ePrimitiveType::PT_Line, static_cast<int32_t>( verts.size() ) ) ) {
        return;
    }
    for ( const LineVert &v : verts ) {
        pCollisionDebugList->setDiffuse( 0, ColorRGBA( v.color ) );
        pCollisionDebugList->pushVertex( v.x, v.y, v.z );
    }
    pCollisionDebugList->commitPrimitive();
    gpRender->getRenderPass<eRenderPass::RP_Near>()
        .pScene->enqueueDynamicDrawList( pCollisionDebugList );
}

constexpr uint32_t kMaxVehicleDebugLineVerts = 65536u;

static Material *GetDebugNoDepthMaterial() {
    static StaticRender2DM debug2DM( false, 0x0d7b9f2a4c61e305ull );
    static bool bPrepared = false;
    if ( !bPrepared ) {
        gpRenderContext->prepareMaterial( debug2DM.pMaterial );
        bPrepared = true;
    }
    debug2DM.pMaterial->OT = DAT_00fe6d50.pMaterial->OT;
    debug2DM.pMaterial->DepthTest = eDepthTest::DT_NoneZTest;
    debug2DM.pMaterial->DepthWrite = 2; // no depth write
    return debug2DM.pMaterial;
}

void GSPhysics::drawVehicleShapes() {
    if ( pPhysicsSystem == nullptr || gpRender == nullptr ) {
        return;
    }

    struct VehicleSnapshot {
        JPH::RefConst<JPH::Shape> shape;
        // shape -> shifted world, origin at the CoM
        JPH::Mat44 comTransform;
        // center of mass, shifted world
        JPH::Vec3 comPosition;
        // body/shape origin, shifted world
        JPH::Vec3 bodyPosition;
    };
    std::vector<VehicleSnapshot> snapshots;

    lockMutex();
    for ( PhysicsVehicle *it = pPhysicsVehiclesBegin; it != pPhysicsVehiclesEnd;
          ++it ) {
        if ( !it->isActive() ) {
            continue;
        }
        const JPH::Body *pChassis = it->getChassisBody();
        if ( pChassis == nullptr || pChassis->GetShape() == nullptr ) {
            continue;
        }
        VehicleSnapshot s;
        s.shape = pChassis->GetShape();
        s.comTransform = pChassis->GetCenterOfMassTransform();
        s.comPosition = JPH::Vec3( pChassis->GetCenterOfMassPosition() );
        s.bodyPosition = JPH::Vec3( pChassis->GetPosition() );
        snapshots.push_back( std::move( s ) );
    }
    unlockMutex();

    if ( snapshots.empty() ) {
        return;
    }

    if ( pVehicleDebugList == nullptr ) {
        pVehicleDebugList = new DrawList();
        if ( !pVehicleDebugList->initialize( 2, kMaxVehicleDebugLineVerts, 0,
                                             true, nullptr, "VehicleDebug",
                                             false ) ) {
            pVehicleDebugList->destroy();
            delete pVehicleDebugList;
            pVehicleDebugList = nullptr;
            return;
        }
    }

    struct LineVert {
        float x, y, z;
        uint32_t color;
    };
    static std::vector<LineVert> colliderVerts;
    static std::vector<LineVert> gizmoVerts;
    colliderVerts.clear();
    gizmoVerts.clear();

    auto pushSeg = [&]( std::vector<LineVert> &out, const JPH::Vec3 &a,
                        const JPH::Vec3 &b, uint32_t color ) {
        if ( out.size() + 2 > kMaxVehicleDebugLineVerts ) {
            return;
        }
        out.push_back( { a.GetX(), a.GetY(), a.GetZ(), color } );
        out.push_back( { b.GetX(), b.GetY(), b.GetZ(), color } );
    };

    auto pushMarker = [&]( const JPH::Vec3 &center, const JPH::Mat44 &basis,
                           float radius, uint32_t color ) {
        const JPH::Vec3 axis[3] = { basis.GetAxisX() * radius,
                                    basis.GetAxisY() * radius,
                                    basis.GetAxisZ() * radius };
        for ( const JPH::Vec3 &a : axis ) {
            pushSeg( gizmoVerts, center - a, center + a, color );
        }
        for ( int32_t i = 0; i < 3; i++ ) {
            const JPH::Vec3 h = axis[i] * 0.5f;
            const JPH::Vec3 k = axis[( i + 1 ) % 3] * 0.5f;
            pushSeg( gizmoVerts, center + h, center + k, color );
            pushSeg( gizmoVerts, center + k, center - h, color );
            pushSeg( gizmoVerts, center - h, center - k, color );
            pushSeg( gizmoVerts, center - k, center + h, color );
        }
    };

    const uint32_t kColliderColor = PackDebugColor( 0, 230, 255 ).Color;
    const uint32_t kCoMColor = PackDebugColor( 255, 40, 200 ).Color;
    const uint32_t kOriginColor = PackDebugColor( 240, 240, 240 ).Color;

    for ( const VehicleSnapshot &s : snapshots ) {
        if ( gDrawVehicleCollisionVis ) {
            JPH::AllHitCollisionCollector<JPH::TransformedShapeCollector>
                leaves;
            s.shape->CollectTransformedShapes(
                JPH::AABox::sBiggest(), s.comTransform.GetTranslation(),
                s.comTransform.GetQuaternion(), JPH::Vec3::sReplicate( 1.0f ),
                JPH::SubShapeIDCreator(), leaves, {} );

            for ( const JPH::TransformedShape &leaf : leaves.mHits ) {
                if ( colliderVerts.size() + 2 > kMaxVehicleDebugLineVerts ) {
                    break;
                }
                JPH::Shape::GetTrianglesContext ctx;
                leaf.GetTrianglesStart( ctx, JPH::AABox::sBiggest(),
                                        JPH::RVec3::sZero() );
                constexpr int kBatch = 64;
                JPH::Float3 tri[3 * kBatch];
                for ( ;; ) {
                    const int n = leaf.GetTrianglesNext( ctx, kBatch, tri );
                    if ( n == 0 ) {
                        break;
                    }
                    for ( int t = 0; t < n; t++ ) {
                        const JPH::Vec3 a( tri[3 * t + 0].x, tri[3 * t + 0].y,
                                           tri[3 * t + 0].z );
                        const JPH::Vec3 b( tri[3 * t + 1].x, tri[3 * t + 1].y,
                                           tri[3 * t + 1].z );
                        const JPH::Vec3 c( tri[3 * t + 2].x, tri[3 * t + 2].y,
                                           tri[3 * t + 2].z );
                        pushSeg( colliderVerts, a, b, kColliderColor );
                        pushSeg( colliderVerts, b, c, kColliderColor );
                        pushSeg( colliderVerts, c, a, kColliderColor );
                    }
                    if ( colliderVerts.size() + 2 >
                         kMaxVehicleDebugLineVerts ) {
                        break;
                    }
                }
            }
        }

        if ( gDrawVehicleCoMVis ) {
            pushMarker( s.bodyPosition, s.comTransform, 0.15f, kOriginColor );
            pushMarker( s.comPosition, s.comTransform, 0.30f, kCoMColor );
            pushSeg( gizmoVerts, s.bodyPosition, s.comPosition, kCoMColor );
        }
    }

    pVehicleDebugList->reset();
    pVehicleDebugList->setIdentityTransform();

    auto emit = [&]( const std::vector<LineVert> &in, Material *pMaterial ) {
        if ( in.empty() ) {
            return;
        }
        pVehicleDebugList->setActiveMaterial( pMaterial );
        if ( !pVehicleDebugList->beginPrimitive(
                 ePrimitiveType::PT_Line,
                 static_cast<int32_t>( in.size() ) ) ) {
            return;
        }
        for ( const LineVert &v : in ) {
            pVehicleDebugList->setDiffuse( 0, ColorRGBA( v.color ) );
            pVehicleDebugList->pushVertex( v.x, v.y, v.z );
        }
        pVehicleDebugList->commitPrimitive();
    };

    emit( colliderVerts, GetDebugNoDepthMaterial() );
    emit( gizmoVerts, GetDebugNoDepthMaterial() );

    if ( pVehicleDebugList->getNumPrimitives() == 0 ) {
        return;
    }
    gpRender->getRenderPass<eRenderPass::RP_Near>()
        .pScene->enqueueDynamicDrawList( pVehicleDebugList );
}
#endif

void GSPhysics::draw( float deltaTime ) {
    // FUN_0084b650
    if ( gDrawCollisionVis ) {
#if OTDU_DEVBUILD
        drawCollisionShapes();
#endif
    }
    if ( gDrawVehicleCollisionVis || gDrawVehicleCoMVis ) {
#if OTDU_DEVBUILD
        drawVehicleShapes();
#endif
    }

    if ( ( debugDrawFlags & 0xc00 ) != 0 ) {
        drawGraph();
    }

    if ( !bHavokDebugEnabled || !bDrawDebugInfos )
        return;

    OTDU_UNIMPLEMENTED;
}

void GSPhysics::createGraph() {
    DrawStreams streams;
    streams.Diffuse = '\1';
    streams.bIndex16 = '\1';

    pGraphDrawList = new DrawList();
    if ( !pGraphDrawList->initialize( 16, 0x1000, 0, true, &streams,
                                      "PhysicsGraph", false ) ) {
        pGraphDrawList->destroy();
        delete pGraphDrawList;
        pGraphDrawList = nullptr;
        return;
    }

    // FUN_0084d570 (graph creation)
    gPhysicsGraph.create( 4, pGraphDrawList, -10.0f, 10.0f, 1000, 2.0f );
    if ( gPhysicsGraph.getNumCurves() == 4 ) {
        gPhysicsGraph.getCurve( 0 ).setColor( 0xf0, 0x00, 0x00, 0xff );
        gPhysicsGraph.getCurve( 1 ).setColor( 0x00, 0xf0, 0x00, 0xff );
        gPhysicsGraph.getCurve( 2 ).setColor( 0x00, 0x00, 0xf0, 0xff );
        gPhysicsGraph.getCurve( 3 ).setColor( 0xf0, 0x00, 0xf0, 0xff );
    }
    gPhysicsGraph.setBounds( 0.0f, 0.5f, 1.0f, 0.95f );
}

void GSPhysics::drawGraph() {
    // FUN_0084b650 (graph draw)
    if ( pGraphDrawList == nullptr ) {
        return;
    }

    pGraphDrawList->reset();
    lockMutex();
    gPhysicsGraph.draw( gpTimer->SystemTotalTime );
    unlockMutex();

    if ( pGraphDrawList->getNumPrimitives() == 0 ) {
        return;
    }
    gpRender->getRenderPass<eRenderPass::RP_2D>()
        .pScene->enqueueDynamicDrawList( pGraphDrawList );
}

void GSPhysics::FUN_00843b40() {
    // FUN_00843b40
    if ( pPhysicsSystem == nullptr )
        return;

    lockMutex();
    JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();
    for ( auto &hf : pHeightfields ) {
        if ( hf.pBody != nullptr ) {
            JPH::Body *pBody = static_cast<JPH::Body *>( hf.pBody );
            bodyInterface.RemoveBody( pBody->GetID() );
            bodyInterface.DestroyBody( pBody->GetID() );
            hf.pBody = nullptr;
            hf.pKey = nullptr;
        }
    }
    unlockMutex();
}

PhysFactory *GSPhysics::createFromBinary( void *param_3, int32_t param_2 ) {
    // FUN_008450b0 / FUN_00845240
    PhysicsLock lock;

    HavokContainer container;
    if ( !container.parse( param_3, static_cast<uint32_t>( param_2 ) ) ) {
        OTDU_LOG_WARN( "Failed to parse physics container blob\n" );
        return nullptr;
    }

    HavokPackfile packfile;
    if ( !packfile.parse( container.getHavokScene(),
                          container.getHavokSceneSize() ) ) {
        OTDU_LOG_WARN( "Failed to parse the embedded Havok scene\n" );
        return nullptr;
    }

    HavokFactoryData *pData = new HavokFactoryData();
    HavokToJolt::buildBodies( packfile, pData->Bodies );
    if ( pData->Bodies.empty() ) {
        OTDU_LOG_WARN( "Havok scene produced no collision bodies\n" );
        delete pData;
        return nullptr;
    }

    const int32_t entryCount = container.getEntryCount();
    for ( HavokToJolt::RigidBody &b : pData->Bodies ) {
        if ( b.RegistryIndex >= static_cast<uint32_t>( entryCount ) ) {
            continue;
        }
        const PhysFactory::Entry e =
            container.getEntry( static_cast<int32_t>( b.RegistryIndex ) );
        if ( e.Type == ePhysEntityType::PET_RigidBody ) {
            b.Hashcode = e.Hashcode;
        }
    }

    pData->Registry.reserve( entryCount );
    for ( int32_t i = 0; i < entryCount; i++ ) {
        const PhysFactory::Entry e = container.getEntry( i );
        pData->Registry.emplace_back( e.Hashcode,
                                      static_cast<uint32_t>( e.Type ) );
    }

    BuildMechaMembers( container, entryCount, pData );

    // Parse the scene's joints (hkConstraintInstance list)
    HavokToJolt::buildConstraints( packfile, pData->Constraints );
    for ( HavokToJolt::Constraint &c : pData->Constraints ) {
        if ( c.BodyARegistryIndex < static_cast<uint32_t>( entryCount ) ) {
            const PhysFactory::Entry ea = container.getEntry(
                static_cast<int32_t>( c.BodyARegistryIndex ) );
            if ( ea.Type == ePhysEntityType::PET_RigidBody ) {
                c.BodyAHashcode = ea.Hashcode;
            }
        }
        if ( c.BodyBRegistryIndex < static_cast<uint32_t>( entryCount ) ) {
            const PhysFactory::Entry eb = container.getEntry(
                static_cast<int32_t>( c.BodyBRegistryIndex ) );
            if ( eb.Type == ePhysEntityType::PET_RigidBody ) {
                c.BodyBHashcode = eb.Hashcode;
            }
        }
    }

    PhysFactory *pFactory = nullptr;
    for ( PhysFactory &factory : pFactories ) {
        if ( !factory.isInitialized() ) {
            pFactory = &factory;
            break;
        }
    }
    if ( pFactory == nullptr ) {
        OTDU_LOG_WARN( "No free physics factory slot for the vehicle blob\n" );
        delete pData;
        return nullptr;
    }

    pFactory->initialize( pData );
    OTDU_LOG_INFO( "Registered physics factory: %d collision bodies\n",
                   static_cast<int>( pData->Bodies.size() ) );
    return pFactory;
}

static const HavokToJolt::RigidBody *
FindFactoryMember( const HavokFactoryData &data, const uint64_t logicID ) {
    for ( const auto &mecha : data.MechaMembers ) {
        for ( const HavokToJolt::MechaMember &member : mecha.second ) {
            if ( std::find( member.LogicIDs.begin(), member.LogicIDs.end(),
                            logicID ) == member.LogicIDs.end() ) {
                continue;
            }
            for ( const HavokToJolt::RigidBody &body : data.Bodies ) {
                if ( body.Hashcode == member.BodyHashcode ) {
                    return &body;
                }
            }
        }
    }
    return nullptr;
}

void GSPhysics::createVehicleParts( PhysicsVehicle &vehicle,
                                    JPH::Body &chassis,
                                    const HavokFactoryData &data,
                                    const HavokToJolt::RigidBody &chassisMember,
                                    const JPH::Mat44 &factoryWorld,
                                    const uint32_t groupID ) {
    // FUN_0082fc80 (inlined) + FUN_00453690 (constraint remap)
    static constexpr float kPartAngularDamping = 0.1f; // FLOAT_00f41668

    JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();
    JPH::GroupFilterTable *pFilter = GetPropGroupFilter();

    std::vector<std::pair<uint64_t, JPH::Body *>> placed;
    placed.emplace_back( chassisMember.Hashcode, &chassis );

    std::vector<PhysicsVehicle::AttachedPart> parts;
    uint32_t subGroup = 1;
    for ( const HavokToJolt::RigidBody &member : data.Bodies ) {
        if ( &member == &chassisMember || member.Shape == nullptr ||
             member.bFixedMotion ) {
            continue;
        }
        const JPH::Mat44 m =
            factoryWorld * JPH::Mat44::sRotationTranslation( member.Rotation,
                                                             member.Position );

        JPH::BodyCreationSettings settings(
            member.Shape, JPH::RVec3( m.GetTranslation() ),
            m.GetQuaternion().Normalized(), JPH::EMotionType::Dynamic,
            TDUPhysLayers::VEHICLE );
        settings.mOverrideMassProperties =
            JPH::EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass =
            member.Mass > 0.0f ? member.Mass : kDynamicPropMass;
        settings.mLinearDamping = member.LinearDamping;
        settings.mAngularDamping = kPartAngularDamping;
        settings.mMaxLinearVelocity = 138.888885f; // DAT_00f41ee8
        settings.mMaxAngularVelocity = 12.566371f; // DAT_00f41eec
        settings.mMotionQuality = JPH::EMotionQuality::Discrete;
        settings.mFriction = member.Friction;
        settings.mRestitution = member.Restitution;
        settings.mCollisionGroup = JPH::CollisionGroup(
            pFilter, groupID, Min( subGroup++, kMaxPropMembers - 1 ) );
        settings.mUserData = TDUSurface::Generic;

        JPH::Body *pBody = bodyInterface.CreateBody( settings );
        if ( pBody == nullptr ) {
            continue;
        }
        bodyInterface.AddBody( pBody->GetID(), JPH::EActivation::Activate );
        placed.emplace_back( member.Hashcode, pBody );

        PhysicsVehicle::AttachedPart part;
        part.pBody = pBody;
        part.RenderKey = member.Hashcode;
        for ( const auto &mecha : data.MechaMembers ) {
            for ( const HavokToJolt::MechaMember &mm : mecha.second ) {
                if ( mm.BodyHashcode == member.Hashcode &&
                     !mm.LogicIDs.empty() ) {
                    part.RenderKey = mm.LogicIDs.front();
                }
            }
        }
        parts.push_back( part );
    }

    for ( const HavokToJolt::Constraint &jc : data.Constraints ) {
        JPH::Body *pBodyA = nullptr;
        JPH::Body *pBodyB = nullptr;
        for ( const auto &entry : placed ) {
            if ( entry.first == jc.BodyAHashcode ) {
                pBodyA = entry.second;
            }
            if ( entry.first == jc.BodyBHashcode ) {
                pBodyB = entry.second;
            }
        }
        if ( pBodyA == nullptr || pBodyB == nullptr || pBodyA == pBodyB ) {
            continue;
        }

        JPH::FixedConstraintSettings cs;
        cs.mAutoDetectPoint = true;
        JPH::TwoBodyConstraint *pConstraint = cs.Create( *pBodyA, *pBodyB );
        if ( pConstraint == nullptr ) {
            continue;
        }
        pPhysicsSystem->AddConstraint( pConstraint );

        PropConstraintRecord record;
        record.pConstraint = static_cast<JPH::FixedConstraint *>( pConstraint );
        record.BodyA = pBodyA->GetID();
        record.BodyB = pBodyB->GetID();
        record.Threshold = jc.Threshold;
        record.bBreakable = jc.bBreakable;
        gPropConstraints.push_back( record );
    }

    vehicle.setAttachedParts( std::move( parts ),
                              GetIdentifier64bit( "BODY" ) );
}

PhysicsVehicle *GSPhysics::installVehicle( VehicleInstallRequest &request ) {
    // FUN_0084a5d0
    if ( request.pCarData == nullptr ) {
        return nullptr;
    }

    PhysicsVehicle *pVehicle =
        createVehicle( request.pFactory, request.Transform, *request.pCarData,
                       request.pTorqueBinary, request.pPack, request.bIsAI,
                       request.bIsNetwork );
    if ( pVehicle == nullptr ) {
        return nullptr;
    }

    if ( request.InstallType == VIT_Player ) {
        pVehicle->setDrivingControl( this );
    }

    pVehicle->setPostponedPause( 0 );
    pVehicle->setPhysicsPaused( request.bAddPhysicsPaused );

    OTDU_UNIMPLEMENTED_ONCE; // TODO: PHYSICS

    {
        PhysicsLock lock;
        if ( request.bSetPosition ) {
            if ( request.bResetCar ) {
                pVehicle->resetToTransform( request.Transform,
                                            request.bCastToGround );
            } else {
                pVehicle->setPostponedMatrix( request.Transform );
            }
        }
        pVehicle->requestPhantomMode( uint8_t( request.PhantomMode ), 1.0f );
        if ( request.bKeyframed ) {
            pVehicle->setKeyframed( true, false );
        }
        pVehicle->setCinematicMode( request.bCinematicMode );
    }

    OTDU_UNIMPLEMENTED_ONCE; // TODO: PHYSICS

    return pVehicle;
}

PhysicsVehicle *GSPhysics::createVehicle( PhysFactory *pFactory,
                                          const Matrix4x4f_t &worldTransform,
                                          const DBEntryCarPhysicsData &db,
                                          const void *pTorqueBinary,
                                          const DBEntryAftermarketPack *pPack,
                                          const bool bIsAI,
                                          const bool bIsNetwork ) {
    // FUN_0082fc80
    if ( pPhysicsSystem == nullptr ) {
        return nullptr;
    }

    HavokFactoryData *pData =
        ( pFactory != nullptr ) ? pFactory->getHavokData() : nullptr;
    const bool bHasFactoryShape =
        ( pData != nullptr && !pData->Bodies.empty() );

    const float kMmToM = 0.001f;
    const float kConvexRadius = 0.05f; // FLOAT_00f41778
    const float rideHeightFrontMM =
        (float)( db.RideHeightFront > 0 ? db.RideHeightFront : 130 );
    const float rideHeightFrontM = rideHeightFrontMM * kMmToM;
    const float bodyHeightM =
        ( ( db.Height > 0 ? db.Height : 1300 ) - rideHeightFrontMM - 100.0f ) *
        kMmToM;
    const bool bBike = gpDatabase != nullptr && gpDatabase->isBike( &db );
    const float halfW = ( db.Width > 0 ? db.Width : 1800 ) * 0.5f * kMmToM *
                        ( bBike ? 0.5f : 1.0f );
    const float halfH = bodyHeightM * 0.5f;
    const float halfL = ( db.Length > 0 ? db.Length : 4000 ) * 0.5f * kMmToM;

    const HavokToJolt::RigidBody *pChassisMember =
        bHasFactoryShape
            ? FindFactoryMember( *pData, GetIdentifier64bit( "BODY" ) )
            : nullptr;
    const JPH::Mat44 toChassis =
        ( pChassisMember != nullptr )
            ? JPH::Mat44::sRotationTranslation( pChassisMember->Rotation,
                                                pChassisMember->Position )
                  .InversedRotationTranslation()
            : JPH::Mat44::sIdentity();

    JPH::ShapeRefC chassisShape;
    JPH::ShapeSettings::ShapeResult shapeResult;
    JPH::MutableCompoundShapeSettings hull;
    if ( pChassisMember != nullptr ) {
        hull.AddShape( JPH::Vec3::sZero(), JPH::Quat::sIdentity(),
                       pChassisMember->Shape );
    } else if ( bHasFactoryShape ) {
        for ( const HavokToJolt::RigidBody &body : pData->Bodies ) {
            const JPH::Mat44 local =
                toChassis * JPH::Mat44::sRotationTranslation( body.Rotation,
                                                              body.Position );
            hull.AddShape( local.GetTranslation(),
                           local.GetQuaternion().Normalized(), body.Shape );
        }
    } else {
        const auto addBox = [&]( const JPH::Vec3 &coreHalfExtents,
                                 const JPH::Vec3 &offset,
                                 eMaterialType material ) {
            const JPH::Vec3 halfExtents(
                Max( kConvexRadius, coreHalfExtents.GetX() + kConvexRadius ),
                Max( kConvexRadius, coreHalfExtents.GetY() + kConvexRadius ),
                Max( kConvexRadius, coreHalfExtents.GetZ() + kConvexRadius ) );
            hull.AddShape( offset, JPH::Quat::sIdentity(),
                           new JPH::BoxShapeSettings(
                               halfExtents, kConvexRadius,
                               TDUPhysicsMaterials::get(
                                   static_cast<uint32_t>( material ) ) ) );
        };

        if ( bBike ) {
            addBox( JPH::Vec3( halfW - 0.05f, halfH - 0.15f, halfL - 0.05f ),
                    JPH::Vec3( 0.0f, 0.10f, 0.0f ), eMaterialType::MT_Body );
            addBox( JPH::Vec3( 0.30f, 0.425f, 0.30f ),
                    JPH::Vec3( 0.0f, 0.50f, 0.0f ), eMaterialType::MT_Body );
            addBox( JPH::Vec3( 0.01f, 0.20f, halfL - 0.30f ),
                    JPH::Vec3( 0.0f, 0.25f - halfH, 0.0f ),
                    eMaterialType::MT_Chassis );
        } else {
            addBox( JPH::Vec3( halfW - 0.05f, halfH - 0.15f, halfL - 0.55f ),
                    JPH::Vec3( 0.0f, 0.10f, 0.0f ), eMaterialType::MT_Body );
            addBox( JPH::Vec3( halfW - 0.15f, 0.225f, 0.275f ),
                    JPH::Vec3( 0.0f, 0.375f - halfH, halfL - 0.375f ),
                    eMaterialType::MT_BumperF );
            addBox( JPH::Vec3( halfW - 0.15f, 0.350f, 0.275f ),
                    JPH::Vec3( 0.0f, 0.500f - halfH, -( halfL - 0.375f ) ),
                    eMaterialType::MT_BumperR );
            addBox( JPH::Vec3( halfW * 0.8f - 0.05f, 0.20f, halfL - 0.30f ),
                    JPH::Vec3( 0.0f, 0.25f - halfH, 0.0f ),
                    eMaterialType::MT_Chassis );
        }
    }
    shapeResult = hull.Create();
    if ( shapeResult.IsValid() ) {
        chassisShape = shapeResult.Get();
    }
    if ( chassisShape == nullptr ) {
        return nullptr;
    }
    const float gcLengthFrac =
        db.GravityCenterLength > 0 ? db.GravityCenterLength * 0.01f : 0.5f;
    constexpr float kCGHeightCarCoef = 0.75f; // DAT_00f416a4
    const float gcHeightFrac =
        static_cast<float>( db.GravityCenterHeight ) * 0.01f * kCGHeightCarCoef;
    const float bodyLengthM = ( db.Length > 0 ? db.Length : 4000 ) * 0.001f;
    const float comOffsetY = -( 1.0f - gcHeightFrac ) * bodyHeightM * 0.5f;
    const float comOffsetZ = ( gcLengthFrac - 0.5f ) * bodyLengthM;

    const JPH::Vec3 comOffset = JPH::Vec3( 0.0f, comOffsetY, comOffsetZ ) -
                                chassisShape->GetCenterOfMass();
    JPH::OffsetCenterOfMassShapeSettings comShapeSettings( comOffset,
                                                           chassisShape );
    JPH::ShapeSettings::ShapeResult comResult = comShapeSettings.Create();
    if ( !comResult.IsValid() ) {
        return nullptr;
    }

    // World placement (row-major Matrix4x4f_t -> column-major Jolt), rebased
    // into shifted space.
    JPH::Mat44 world = JPH::Mat44::sIdentity();
    for ( int32_t c = 0; c < 4; c++ ) {
        world.SetColumn3( c, JPH::Vec3( worldTransform( c, 0 ),
                                        worldTransform( c, 1 ),
                                        worldTransform( c, 2 ) ) );
    }
    {
        const Eigen::Vector4f shiftOrigin = GetShiftOrigin();
        world.SetTranslation(
            world.GetTranslation() -
            JPH::Vec3( shiftOrigin.x(), shiftOrigin.y(), shiftOrigin.z() ) );
    }

    JPH::BodyCreationSettings settings(
        comResult.Get(), JPH::RVec3( world.GetTranslation() ),
        world.GetQuaternion().Normalized(), JPH::EMotionType::Dynamic,
        TDUPhysLayers::VEHICLE );
    const PhysicsVehicle::PerfPackCoefs packCoefs =
        PhysicsVehicle::BuildPerfPackCoefs( pPack );
    float chassisMass =
        ( db.Weight > 0 ? static_cast<float>( db.Weight ) : 1200.0f ) *
        packCoefs.Weight;
    JPH::Vec3 h( halfW + kConvexRadius, halfH + kConvexRadius,
                 halfL + kConvexRadius );
    uint32_t partGroupID = 0;
    if ( pChassisMember != nullptr ) {
        // FUN_0082fc80 (inlined)
        for ( const HavokToJolt::RigidBody &member : pData->Bodies ) {
            if ( &member != pChassisMember && !member.bFixedMotion ) {
                chassisMass -= member.Mass;
            }
        }
        h = JPH::Vec3( halfW, halfH, halfL );

        partGroupID = gNextPropGroupID++;
        settings.mCollisionGroup =
            JPH::CollisionGroup( GetPropGroupFilter(), partGroupID, 0 );
    }
    settings.mMassPropertiesOverride.mMass = chassisMass;

    const float k = chassisMass / 3.0f;
    JPH::MassProperties mp;
    mp.mMass = chassisMass;
    mp.mInertia = JPH::Mat44::sZero();
    mp.mInertia( 0, 0 ) = k * ( h.GetY() * h.GetY() + h.GetZ() * h.GetZ() );
    mp.mInertia( 1, 1 ) = k * ( h.GetX() * h.GetX() + h.GetZ() * h.GetZ() );
    mp.mInertia( 2, 2 ) = k * ( h.GetX() * h.GetX() + h.GetY() * h.GetY() );
    mp.mInertia( 3, 3 ) = 1.0f;
    settings.mMassPropertiesOverride = mp;

    settings.mOverrideMassProperties =
        JPH::EOverrideMassProperties::MassAndInertiaProvided;

    settings.mEnhancedInternalEdgeRemoval = true;
    settings.mMotionQuality = JPH::EMotionQuality::Discrete;
    settings.mLinearDamping = 0.0f;
    settings.mAngularDamping = 1.0f; // FLOAT_00f4165c
    settings.mFriction = 1.0f;
    settings.mRestitution = 0.25f;
    settings.mMaxLinearVelocity = 138.888885f; // DAT_00f41ee8
    settings.mMaxAngularVelocity = 12.566371f; // DAT_00f41eec
    settings.mUserData = TDUSurface::Vehicle;

    JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();
    lockMutex();
    JPH::Body *pChassis = bodyInterface.CreateBody( settings );
    PhysicsVehicle *pCar = nullptr;
    if ( pChassis != nullptr ) {
        bodyInterface.AddBody( pChassis->GetID(), JPH::EActivation::Activate );
        for ( PhysicsVehicle *it = pPhysicsVehiclesBegin;
              it != pPhysicsVehiclesEnd; ++it ) {
            if ( !it->isActive() ) {
                pCar = it;
                break;
            }
        }
        if ( pCar == nullptr ||
             !pCar->create( pPhysicsSystem, pChassis, db, pTorqueBinary,
                            bHasFactoryShape, pPack, bIsAI, bIsNetwork ) ) {
            bodyInterface.RemoveBody( pChassis->GetID() );
            bodyInterface.DestroyBody( pChassis->GetID() );
            pCar = nullptr;
        } else if ( pChassisMember != nullptr ) {
            createVehicleParts( *pCar, *pChassis, *pData, *pChassisMember,
                                world * toChassis, partGroupID );
        }
    }
    unlockMutex();

    if ( pCar == nullptr ) {
        OTDU_LOG_WARN( "Failed to create vehicle (no chassis or free slot)\n" );
        return nullptr;
    }
    OTDU_LOG_INFO( "Created vehicle: %s chassis, mass=%.0f kg\n",
                   bHasFactoryShape ? "havok" : "db-box",
                   settings.mMassPropertiesOverride.mMass );
    return pCar;
}

void GSPhysics::setVehicleOwner( PhysicsVehicle *pVehicle, void *pOwner ) {
    // FUN_0041f340
    if ( pPhysicsSystem == nullptr || pVehicle == nullptr ||
         !pVehicle->isActive() ) {
        return;
    }
    lockMutex();
    pPhysicsSystem->GetBodyInterface().SetUserData(
        pVehicle->getChassisBody()->GetID(),
        ( reinterpret_cast<uint64_t>( pOwner ) << TDUSurface::kOwnerShift ) |
            TDUSurface::Vehicle );
    unlockMutex();
}

// Cleared by the Secret Island teleporter while it runs
bool gbBackOnTrackFadeGuard = true; // DAT_00f7f1d8

static constexpr int32_t kMaxFadeRequests = 8;
static eFadeRequest gFadeRequests[kMaxFadeRequests]; // DAT_00f8a3b8
static int32_t gLastFadeRequest = -1;                // DAT_00f8a3d8

void GSPhysics::pushFadeRequest( const eFadeRequest request ) {
    // FUN_0081a560
    PhysicsLock lock;
    if ( request == eFadeRequest::FR_FadeOut && gpMngFade != nullptr &&
         ( gpMngFade->getTransitionFlags() >> 3 & 1 ) != 0 ) {
        return;
    }
    if ( gLastFadeRequest >= 0 && gFadeRequests[gLastFadeRequest] == request ) {
        return;
    }
    gLastFadeRequest++;
    if ( gLastFadeRequest < kMaxFadeRequests ) {
        gFadeRequests[gLastFadeRequest] = request;
    }
}

eFadeRequest GSPhysics::peekFadeRequest() {
    // FUN_0081a230
    return ( gLastFadeRequest >= 0 ) ? gFadeRequests[0] : eFadeRequest::FR_Idle;
}

eFadeRequest GSPhysics::popFadeRequest() {
    // FUN_0081a250
    for ( int32_t i = 0; i < kMaxFadeRequests - 1; i++ ) {
        gFadeRequests[i] = gFadeRequests[i + 1];
    }
    if ( gLastFadeRequest > -1 ) {
        gLastFadeRequest--;
    }
    return peekFadeRequest();
}

PhysicsVehicle *GSPhysics::findVehicle( const JPH::Body &chassis ) const {
    for ( PhysicsVehicle *it = pPhysicsVehiclesBegin; it != pPhysicsVehiclesEnd;
          ++it ) {
        if ( it->getChassisBody() == &chassis ) {
            return it;
        }
    }
    return nullptr;
}

void GSPhysics::destroyVehicle( PhysicsVehicle *pVehicle ) {
    if ( pVehicle == nullptr || !pVehicle->isActive() ) {
        return;
    }
    lockMutex();
    // FUN_00843800
    if ( gpCarState != nullptr ) {
        CarState *pCarStates = gpCarState->getCarStates();
        for ( uint32_t i = 0; i < kMaxNumVehicles; i++ ) {
            if ( pCarStates[i].getPhysicsVehicle() == pVehicle ) {
                gGameObjectDico.unregisterObject( &pCarStates[i] );
            }
        }
    }
    pVehicle->destroy();
    unlockMutex();
}

void GSPhysics::drawDebug( float ) {
    ImGui::Checkbox( "Collision wireframe (green=world blue=road orange=props)",
                     &gDrawCollisionVis );
    if ( gDrawCollisionVis ) {
        ImGui::SameLine();
        ImGui::Checkbox( "+terrain", &gDrawCollisionVisTerrain );
    }
    ImGui::Checkbox( "Vehicle chassis collider (cyan)",
                     &gDrawVehicleCollisionVis );
    ImGui::Checkbox( "Vehicle CoM (magenta = CoM, white = body origin)",
                     &gDrawVehicleCoMVis );
    bool bForceFeedbackGraph = ( debugDrawFlags & 0x400 ) != 0;
    if ( ImGui::Checkbox(
             "FFB graph (red = slip, blue = slip factor, magenta = torque)",
             &bForceFeedbackGraph ) ) {
        debugDrawFlags = bForceFeedbackGraph ? ( debugDrawFlags | 0x400 )
                                             : ( debugDrawFlags & ~0x400u );
    }

    extern int32_t gVehicleDrivingHelpOverride;
    extern bool gVehicleAntiRoll;
    extern bool gVehicleEngineStateMachine;
    extern int gVehicleDiffTypeOverride; // 0=per-car, 1 open, 2 LSD, 4 spool
    extern float gVehicleDiffLockCoef;   // LSD/spool lock coef (override)
    extern float gVehicleAeroDragScale;  // debug: aero drag multiplier
    extern float gVehicleCouplingCoefE;  //
    extern float gVehicleCouplingCoefW;  //
    extern float gVehicleMaxCoupling;    // FIXME
    extern bool gVehicleStartChronopack; //
    extern bool gVehicleESPEnabled;      //
    extern int
        gVehicleDrivelineInertiaMode; // -1=per-car, 0 /|k|, 1 /k^2, 2 /sqrt

    ImGui::SeparatorText( "Vehicle" );
    static const char *kDrivingHelpLevels[] = {
        "player setting", "0 SECURE", "1 SPORT", "2 HYPERSPORT", "3 off" };
    int32_t helpCombo = gVehicleDrivingHelpOverride + 1;
    if ( ImGui::Combo( "Driving help", &helpCombo, kDrivingHelpLevels,
                       IM_ARRAYSIZE( kDrivingHelpLevels ) ) ) {
        gVehicleDrivingHelpOverride = helpCombo - 1;
    }
    extern int32_t gVehicleSteerControlOverride;
    static const char *kSteerControlModes[] = {
        "default (Physics.ini UseSteerControl)", "pad/analog",
        "wheel/bar (speed curves + rate limit)" };
    int32_t steerCombo = gVehicleSteerControlOverride + 1;
    if ( ImGui::Combo( "Steer control mode", &steerCombo, kSteerControlModes,
                       IM_ARRAYSIZE( kSteerControlModes ) ) ) {
        gVehicleSteerControlOverride = steerCombo - 1;
    }
    ImGui::SliderFloat( "Aero drag scale", &gVehicleAeroDragScale, 0.0f, 4.0f,
                        "%.2f" );
    ImGui::Checkbox( "Enable ESP", &gVehicleESPEnabled );
    if ( ImGui::Button( "Start accel chrono" ) ) {
        gVehicleStartChronopack =
            true; // stop, then move off: splits go to the log
    }
    ImGui::SliderFloat( "Coupling coef E", &gVehicleCouplingCoefE, -1.0f, 20.0f,
                        "%.2f" );
    ImGui::SliderFloat( "Coupling coef W", &gVehicleCouplingCoefW, -1.0f, 20.0f,
                        "%.2f" );
    ImGui::SliderFloat( "Max coupling (0 = unclamped)", &gVehicleMaxCoupling,
                        0.0f, 20000.0f, "%.0f" );

    ImGui::Checkbox( "Enable anti-roll bars sim", &gVehicleAntiRoll );
    ImGui::Checkbox( "Enable ingition sim", &gVehicleEngineStateMachine );
    ImGui::TextUnformatted( "Differential override" );
    ImGui::SameLine();
    ImGui::RadioButton( "none##diff", &gVehicleDiffTypeOverride, 0 );
    ImGui::SameLine();
    ImGui::RadioButton( "open##diff", &gVehicleDiffTypeOverride, 1 );
    ImGui::SameLine();
    ImGui::RadioButton( "LSD##diff", &gVehicleDiffTypeOverride, 2 );
    ImGui::SameLine();
    ImGui::RadioButton( "spool##diff", &gVehicleDiffTypeOverride, 4 );
    if ( gVehicleDiffTypeOverride == 2 || gVehicleDiffTypeOverride == 4 ) {
        ImGui::SliderFloat( "Lock coef", &gVehicleDiffLockCoef, 0.0f, 200.0f );
    }
    ImGui::TextUnformatted( "Driveline inertia divisor" );
    ImGui::SameLine();
    ImGui::RadioButton( "car##dli", &gVehicleDrivelineInertiaMode, -1 );
    ImGui::SameLine();
    ImGui::RadioButton( "/|k|##dli", &gVehicleDrivelineInertiaMode, 0 );
    ImGui::SameLine();
    ImGui::RadioButton( "/k^2##dli", &gVehicleDrivelineInertiaMode, 1 );
    ImGui::SameLine();
    ImGui::RadioButton( "/sqrt##dli", &gVehicleDrivelineInertiaMode, 2 );
    ImGui::Checkbox( "Hardcore mode", &gbSimuModeEnabled );

    ImGui::Separator();

    int32_t carIndex = 0;
    if ( pPhysicsVehiclesBegin != nullptr ) {
        for ( PhysicsVehicle *it = pPhysicsVehiclesBegin;
              it != pPhysicsVehiclesEnd; ++it ) {
            if ( !it->isActive() ) {
                continue;
            }

            const float speedKmh = it->getChassisSpeed() * 3.6f;
            const float rpm = it->getEngineAngularVelocity() * 9.549296f;
            ImGui::Text( "Vehicle %d", carIndex );
            ImGui::Text( "Speed %6.1f km/h   Gear %d   RPM %5.0f", speedKmh,
                         it->getCurrentGear(), rpm );
            ImGui::Text( "Lateral G %+5.2f   Longitudinal G %+5.2f",
                         it->getLateralG(), it->getLongitudinalG() );

            const Matrix4x4f_t chassis = it->getChassisWorldMatrix();
            const float rollDeg =
                atan2f( chassis( 1, 0 ), chassis( 1, 1 ) ) * 57.29578f;
            const float pitchDeg =
                atan2f( chassis( 1, 2 ), chassis( 1, 1 ) ) * 57.29578f;
            ImGui::Text( "Roll %+5.1f deg   Pitch %+5.1f deg", rollDeg,
                         pitchDeg );
            ImGui::Text( "Pos  %.1f, %.1f, %.1f", chassis( 3, 0 ),
                         chassis( 3, 1 ), chassis( 3, 2 ) );
            ImGui::Text(
                "ESP %-4s  gripDiff %+6.2f / thr %5.2f   mag F %4.2f R %4.2f",
                it->areElectronicsEnabled()
                    ? ( it->isEspIntervening() ? "ON" : "idle" )
                    : "off",
                it->getEspGripDiff(), it->getEspThreshold(),
                it->getEspMagFront(), it->getEspMagRear() );

            const char *pWheelNames[4] = { "FL", "FR", "RL", "RR" };
            if ( ImGui::BeginTable( "wheels", 11,
                                    ImGuiTableFlags_Borders |
                                        ImGuiTableFlags_RowBg ) ) {
                ImGui::TableSetupColumn( "Wheel" );
                ImGui::TableSetupColumn( "Ground" );
                ImGui::TableSetupColumn( "Load" );
                ImGui::TableSetupColumn( "Bump" );
                ImGui::TableSetupColumn( "Susp" );
                ImGui::TableSetupColumn( "Droop" );
                ImGui::TableSetupColumn( "SlipLong" );
                ImGui::TableSetupColumn( "SlipLat" );
                ImGui::TableSetupColumn( "ESP" );
                ImGui::TableSetupColumn( "Tesp" );
                ImGui::TableSetupColumn( "Tbrk" );
                ImGui::TableHeadersRow();

                static float smoothedLoad[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
                const uint32_t numWheels = it->getNumWheels();
                for ( uint32_t w = 0; w < numWheels; w++ ) {
                    const PhysicsVehicle::WheelReadout wheel =
                        it->getWheelReadout( w );
                    if ( w < 4 ) {
                        smoothedLoad[w] +=
                            ( wheel.SuspensionLoad - smoothedLoad[w] ) * 0.1f;
                    }
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::Text( "%s", w < 4 ? pWheelNames[w] : "?" );
                    ImGui::TableNextColumn();
                    ImGui::Text( "%s", wheel.bGrounded ? "yes" : "AIR" );
                    ImGui::TableNextColumn();
                    ImGui::Text( "%.0f", w < 4 ? smoothedLoad[w]
                                               : wheel.SuspensionLoad );
                    ImGui::TableNextColumn();
                    ImGui::Text( "%.3f", wheel.SuspensionBumpStop );
                    ImGui::TableNextColumn();
                    ImGui::Text( "%.3f", wheel.SuspensionLength );
                    ImGui::TableNextColumn();
                    ImGui::Text( "%.3f", wheel.SuspensionDroopStop );
                    ImGui::TableNextColumn();
                    ImGui::Text( "%+.2f", wheel.LongitudinalSlip );
                    ImGui::TableNextColumn();
                    ImGui::Text( "%+.2f", wheel.LateralSlip );
                    ImGui::TableNextColumn();
                    ImGui::Text( "%d", wheel.EspState );
                    ImGui::TableNextColumn();
                    ImGui::Text( "%+.0f", wheel.EspTorqueOut );
                    ImGui::TableNextColumn();
                    ImGui::Text( "%+.0f", wheel.BrakeTorqueOut );
                }
                ImGui::EndTable();
            }
            ImGui::Separator();
            carIndex++;
        }
    }

    if ( carIndex == 0 ) {
        ImGui::Text( "No active vehicle" );
    }
}

void GSPhysics::addInfinitePlane( float planeHeight ) {
    // FUN_00844c60
    // TODO: Use a box shape for now (as Jolt plane shape does not seem to match
    // 1:1 with Havok); but might want to revisit this in the future.
    if ( pPhysicsSystem == nullptr )
        return;

    const Eigen::Vector4f shiftOrigin = GetShiftOrigin();
    JPH::BodyCreationSettings settings(
        new JPH::BoxShapeSettings(
            JPH::Vec3( 10000.0f, 1.0f, 10000.0f ), JPH::cDefaultConvexRadius,
            TDUPhysicsMaterials::get(
                static_cast<uint32_t>( eMaterialType::MT_Asphalt ) ) ),
        JPH::RVec3( 0.0f, planeHeight - 1.0f - shiftOrigin.y(), 0.0f ),
        JPH::Quat::sIdentity(), JPH::EMotionType::Static,
        TDUPhysLayers::NON_MOVING );
    settings.mFriction = 1.0f; // FLOAT_00f4165c
    settings.mUserData = TDUSurface::Paved;

    lockMutex();
    JPH::BodyID id = pPhysicsSystem->GetBodyInterface().CreateAndAddBody(
        settings, JPH::EActivation::DontActivate );
    infinitePlaneBodyID = id.GetIndexAndSequenceNumber();
    unlockMutex();
}

void GSPhysics::removeInfinitePlane() {
    // FUN_00841b60
    if ( pPhysicsSystem == nullptr ||
         infinitePlaneBodyID == JPH::BodyID::cInvalidBodyID ) {
        return;
    }

    lockMutex();
    JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();
    const JPH::BodyID planeID( infinitePlaneBodyID );
    bodyInterface.RemoveBody( planeID );
    bodyInterface.DestroyBody( planeID );
    unlockMutex();

    infinitePlaneBodyID = JPH::BodyID::cInvalidBodyID;
}

void GSPhysics::removeAndDestroyFactory( PhysFactory *param_2 ) {
    // FUN_008427b0
    if ( param_2 == nullptr ) {
        return;
    }
    lockMutex();
    propFactories.erase(
        std::remove( propFactories.begin(), propFactories.end(), param_2 ),
        propFactories.end() );
    if ( pSceneryFactory == param_2 ) {
        pSceneryFactory = nullptr;
    }
    HavokFactoryData *pData = param_2->getHavokData();
    if ( pData != nullptr && pPhysicsSystem != nullptr ) {
        JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();
        for ( uint32_t idValue : pData->LiveBodyIDs ) {
            const JPH::BodyID id( idValue );
            bodyInterface.RemoveBody( id );
            bodyInterface.DestroyBody( id );
        }
    }
    param_2->reset();
    unlockMutex();
}

RigidBody_t GSPhysics::createSphere( uint32_t collisionGroup,
                                     const Eigen::Vector4f &worldPosition,
                                     float radius,
                                     float mass,
                                     float massScale,
                                     bool registerNow,
                                     bool bPhantom ) {
    // FUN_0081d660
    (void)collisionGroup;

    if ( pPhysicsSystem == nullptr )
        return nullptr;

    const Eigen::Vector4f origin = GetShiftOrigin();
    JPH::RVec3 position( worldPosition.x() - origin.x(),
                         worldPosition.y() - origin.y(),
                         worldPosition.z() - origin.z() );

    JPH::BodyCreationSettings settings( new JPH::SphereShapeSettings( radius ),
                                        position, JPH::Quat::sIdentity(),
                                        JPH::EMotionType::Kinematic,
                                        TDUPhysLayers::MOVING );
    settings.mIsSensor = bPhantom;
    settings.mFriction = 1.0f; // FLOAT_00f4165c
    settings.mOverrideMassProperties =
        JPH::EOverrideMassProperties::CalculateInertia;
    settings.mMassPropertiesOverride.mMass = mass * massScale;

    JPH::EActivation activation = registerNow ? JPH::EActivation::Activate
                                              : JPH::EActivation::DontActivate;

    lockMutex();
    JPH::Body *pBody =
        pPhysicsSystem->GetBodyInterface().CreateBody( settings );
    if ( pBody != nullptr ) {
        pPhysicsSystem->GetBodyInterface().AddBody( pBody->GetID(),
                                                    activation );
    }
    unlockMutex();

    return pBody;
}

void GSPhysics::destroyRigidBody( RigidBody_t pRigidBody ) {
    // NOTE: In the original code, this is done on the Havok rigid body
    // instance directly. The logic was moved to GSPhysics to give flexibility
    // to adopt any physics engine in the future.
    if ( pRigidBody == nullptr || pPhysicsSystem == nullptr )
        return;

    PhysicsLock lock;

    // A prop member may be jointed to its siblings; drop those joints first so
    // Jolt never references a destroyed body.
    removePropConstraintsFor( pRigidBody );

    JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();
    JPH::Body *pBody = static_cast<JPH::Body *>( pRigidBody );
    bodyInterface.RemoveBody( pBody->GetID() );
    bodyInterface.DestroyBody( pBody->GetID() );
}

RigidBody_t GSPhysics::addStaticMesh( const void *pKey,
                                      const Eigen::Vector3f *pVertices,
                                      uint32_t numVertices,
                                      const uint32_t *pIndices,
                                      uint32_t numIndices,
                                      uint32_t surfaceClass ) {
    if ( pPhysicsSystem == nullptr || pKey == nullptr || pVertices == nullptr ||
         pIndices == nullptr || numVertices < 3 || numIndices < 3 ) {
        return nullptr;
    }

    removeStaticMesh( pKey );

    const Eigen::Vector4f origin = GetShiftOrigin();

    JPH::VertexList vertices;
    vertices.reserve( numVertices );
    for ( uint32_t i = 0; i < numVertices; i++ ) {
        vertices.push_back( JPH::Float3( pVertices[i].x() - origin.x(),
                                         pVertices[i].y() - origin.y(),
                                         pVertices[i].z() - origin.z() ) );
    }

    JPH::IndexedTriangleList triangles;
    triangles.reserve( numIndices / 3 );
    for ( uint32_t i = 0; i + 2 < numIndices; i += 3 ) {
        const uint32_t i0 = pIndices[i + 0];
        const uint32_t i1 = pIndices[i + 1];
        const uint32_t i2 = pIndices[i + 2];
        if ( i0 >= numVertices || i1 >= numVertices || i2 >= numVertices ) {
            continue;
        }
        triangles.push_back( JPH::IndexedTriangle( i0, i1, i2, 0 ) );
    }
    if ( triangles.empty() ) {
        return nullptr;
    }

    JPH::MeshShapeSettings meshSettings( std::move( vertices ),
                                         std::move( triangles ) );
    JPH::ShapeSettings::ShapeResult shapeResult = meshSettings.Create();
    if ( !shapeResult.IsValid() ) {
        return nullptr;
    }

    JPH::BodyCreationSettings settings(
        shapeResult.Get(), JPH::RVec3::sZero(), JPH::Quat::sIdentity(),
        JPH::EMotionType::Static, TDUPhysLayers::NON_MOVING );
    settings.mUserData = surfaceClass;
    settings.mFriction = 1.0f; // FLOAT_00f4165c

    PhysicsLock lock;

    JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();
    JPH::Body *pBody = bodyInterface.CreateBody( settings );
    if ( pBody == nullptr ) {
        OTDU_LOG_ERROR( "Static mesh: body creation failed\n" );
        return nullptr;
    }

    bodyInterface.AddBody( pBody->GetID(), JPH::EActivation::DontActivate );
    staticMeshBodies.push_back( { pKey, pBody } );

    OTDU_LOG_INFO( "Static mesh installed: %u vertices, %u triangles\n",
                   numVertices, numIndices / 3 );
    return pBody;
}

void GSPhysics::removeStaticMesh( const void *pKey ) {
    if ( pKey == nullptr || pPhysicsSystem == nullptr ) {
        return;
    }

    for ( auto it = staticMeshBodies.begin(); it != staticMeshBodies.end(); ) {
        if ( it->first == pKey ) {
            destroyRigidBody( it->second );
            it = staticMeshBodies.erase( it );
        } else {
            ++it;
        }
    }
}

CharacterController_t GSPhysics::createCharacter(
    const Eigen::Vector4f &worldPosition, float radius, float cylinderHeight ) {
    if ( pPhysicsSystem == nullptr ) {
        return nullptr;
    }

    JPH::ShapeSettings::ShapeResult shapeResult =
        JPH::RotatedTranslatedShapeSettings(
            JPH::Vec3( 0.0f, 0.5f * cylinderHeight + radius, 0.0f ),
            JPH::Quat::sIdentity(),
            new JPH::CapsuleShape( 0.5f * cylinderHeight, radius ) )
            .Create();
    if ( !shapeResult.IsValid() ) {
        return nullptr;
    }

    JPH::CharacterVirtualSettings settings;
    settings.mShape = shapeResult.Get();
    settings.mMaxSlopeAngle = JPH::DegreesToRadians( 50.0f );
    settings.mMass = 80.0f;
    settings.mBackFaceMode = JPH::EBackFaceMode::IgnoreBackFaces;
    settings.mSupportingVolume = JPH::Plane( JPH::Vec3::sAxisY(), -radius );

    const Eigen::Vector4f origin = GetShiftOrigin();
    const JPH::RVec3 position( worldPosition.x() - origin.x(),
                               worldPosition.y() - origin.y(),
                               worldPosition.z() - origin.z() );

    PhysicsLock lock;

    PhysicsCharacter *pCharacter = new PhysicsCharacter();
    pCharacter->pCharacter = new JPH::CharacterVirtual(
        &settings, position, JPH::Quat::sIdentity(), 0, pPhysicsSystem );

    return pCharacter;
}

void GSPhysics::setCharacterCollisionRestriction(
    CharacterController_t pCharacter, RigidBody_t pOnlyBody ) {
    if ( pCharacter == nullptr ) {
        return;
    }
    pCharacter->RestrictToBody =
        ( pOnlyBody != nullptr ) ? pOnlyBody->GetID() : JPH::BodyID();
}

void GSPhysics::setCharacterExtraCollisionBodies(
    CharacterController_t pCharacter,
    const std::vector<const JPH::Body *> &bodies ) {
    if ( pCharacter == nullptr ) {
        return;
    }

    PhysicsLock lock;
    pCharacter->ExtraBodies.clear();
    for ( const JPH::Body *pBody : bodies ) {
        if ( pBody != nullptr ) {
            pCharacter->ExtraBodies.push_back( pBody->GetID() );
        }
    }
}

int32_t
GSPhysics::rayCastBodyBounds( const Eigen::Vector3f &worldOrigin,
                              const Eigen::Vector3f &direction,
                              float maxDistance,
                              const std::vector<const JPH::Body *> &bodies,
                              float *pHitDistance ) {
    if ( direction.squaredNorm() <= 0.0f || maxDistance <= 0.0f ) {
        return -1;
    }

    PhysicsLock lock;

    const Eigen::Vector4f origin = GetShiftOrigin();
    const JPH::RVec3 rayOrigin( worldOrigin.x() - origin.x(),
                                worldOrigin.y() - origin.y(),
                                worldOrigin.z() - origin.z() );
    const Eigen::Vector3f rayDirection = direction.normalized() * maxDistance;
    const JPH::Vec3 rayVector( rayDirection.x(), rayDirection.y(),
                               rayDirection.z() );

    int32_t closestIndex = -1;
    float closestFraction = 1.0f;
    for ( size_t i = 0; i < bodies.size(); i++ ) {
        const JPH::Body *pBody = bodies[i];
        if ( pBody == nullptr ) {
            continue;
        }

        // Shape bounds are expressed relative to the center of mass
        const JPH::RMat44 worldToBody =
            pBody->GetCenterOfMassTransform().InversedRotationTranslation();
        const JPH::Vec3 localOrigin( worldToBody * rayOrigin );
        const JPH::Vec3 localVector = worldToBody.Multiply3x3( rayVector );

        const JPH::AABox bounds = pBody->GetShape()->GetLocalBounds();
        const float fraction =
            JPH::RayAABox( localOrigin, JPH::RayInvDirection( localVector ),
                           bounds.mMin, bounds.mMax );
        if ( fraction == FLT_MAX ) {
            continue;
        }

        // Negative fraction: the ray starts inside the box
        const float clampedFraction = Max( fraction, 0.0f );
        if ( clampedFraction <= closestFraction ) {
            closestFraction = clampedFraction;
            closestIndex = static_cast<int32_t>( i );
        }
    }

    if ( pHitDistance != nullptr && closestIndex != -1 ) {
        *pHitDistance = closestFraction * maxDistance;
    }

    return closestIndex;
}

void GSPhysics::destroyCharacter( CharacterController_t pCharacter ) {
    if ( pCharacter == nullptr ) {
        return;
    }

    PhysicsLock lock;
    pCharacter->pCharacter = nullptr;
    delete pCharacter;
}

void GSPhysics::updateCharacter( CharacterController_t pCharacter,
                                 const Eigen::Vector3f &desiredVelocity,
                                 float deltaTime ) {
    if ( pCharacter == nullptr || pCharacter->pCharacter == nullptr ||
         pPhysicsSystem == nullptr || deltaTime <= 0.0f ) {
        return;
    }

    PhysicsLock lock;

    JPH::CharacterVirtual *pCV = pCharacter->pCharacter;
    const JPH::Vec3 up = pCV->GetUp();
    const JPH::Vec3 gravity = pPhysicsSystem->GetGravity();
    const JPH::Vec3 verticalVelocity = pCV->GetLinearVelocity().Dot( up ) * up;
    const JPH::Vec3 groundVelocity = pCV->GetGroundVelocity();

    JPH::Vec3 newVelocity;
    if ( pCV->GetGroundState() ==
             JPH::CharacterVirtual::EGroundState::OnGround &&
         ( verticalVelocity - groundVelocity ).Dot( up ) < 0.1f ) {
        newVelocity = groundVelocity;
    } else {
        newVelocity = verticalVelocity;
    }
    newVelocity += gravity * deltaTime;

    const float verticalSpeed = newVelocity.Dot( up );
    if ( verticalSpeed < -kMaxCharacterFallSpeed ) {
        newVelocity -= up * ( verticalSpeed + kMaxCharacterFallSpeed );
    }

    newVelocity += JPH::Vec3( desiredVelocity.x(), 0.0f, desiredVelocity.z() );

    pCV->SetLinearVelocity( newVelocity );

    const AllowListBodyFilter restrictFilter( *pCharacter );
    const JPH::BodyFilter defaultFilter;
    const JPH::BodyFilter &bodyFilter =
        pCharacter->RestrictToBody.IsInvalid()
            ? defaultFilter
            : static_cast<const JPH::BodyFilter &>( restrictFilter );

    JPH::CharacterVirtual::ExtendedUpdateSettings updateSettings;
    pCV->ExtendedUpdate(
        deltaTime, gravity, updateSettings,
        pPhysicsSystem->GetDefaultBroadPhaseLayerFilter(
            TDUPhysLayers::MOVING ),
        pPhysicsSystem->GetDefaultLayerFilter( TDUPhysLayers::MOVING ),
        bodyFilter, {}, *pTempAllocator );
}

Eigen::Vector4f
GSPhysics::getCharacterPosition( CharacterController_t pCharacter ) {
    if ( pCharacter == nullptr || pCharacter->pCharacter == nullptr ) {
        return Eigen::Vector4f( 0.0f, 0.0f, 0.0f, 1.0f );
    }

    const Eigen::Vector4f origin = GetShiftOrigin();
    const JPH::RVec3 position = pCharacter->pCharacter->GetPosition();
    return Eigen::Vector4f( (float)position.GetX() + origin.x(),
                            (float)position.GetY() + origin.y(),
                            (float)position.GetZ() + origin.z(), 1.0f );
}

void GSPhysics::setCharacterPosition( CharacterController_t pCharacter,
                                      const Eigen::Vector4f &worldPosition ) {
    if ( pCharacter == nullptr || pCharacter->pCharacter == nullptr ||
         pPhysicsSystem == nullptr ) {
        return;
    }

    PhysicsLock lock;

    const Eigen::Vector4f origin = GetShiftOrigin();
    pCharacter->pCharacter->SetPosition( JPH::RVec3(
        worldPosition.x() - origin.x(), worldPosition.y() - origin.y(),
        worldPosition.z() - origin.z() ) );
    pCharacter->pCharacter->SetLinearVelocity( JPH::Vec3::sZero() );

    const AllowListBodyFilter restrictFilter( *pCharacter );
    const JPH::BodyFilter defaultFilter;
    const JPH::BodyFilter &bodyFilter =
        pCharacter->RestrictToBody.IsInvalid()
            ? defaultFilter
            : static_cast<const JPH::BodyFilter &>( restrictFilter );

    pCharacter->pCharacter->RefreshContacts(
        pPhysicsSystem->GetDefaultBroadPhaseLayerFilter(
            TDUPhysLayers::MOVING ),
        pPhysicsSystem->GetDefaultLayerFilter( TDUPhysLayers::MOVING ),
        bodyFilter, {}, *pTempAllocator );
}

const char *
GSPhysics::getCharacterGroundState( CharacterController_t pCharacter,
                                    Eigen::Vector3f &outGroundNormal ) {
    outGroundNormal = Eigen::Vector3f::Zero();
    if ( pCharacter == nullptr || pCharacter->pCharacter == nullptr ) {
        return "<none>";
    }

    const JPH::Vec3 normal = pCharacter->pCharacter->GetGroundNormal();
    outGroundNormal =
        Eigen::Vector3f( normal.GetX(), normal.GetY(), normal.GetZ() );
    return JPH::CharacterBase::sToString(
        pCharacter->pCharacter->GetGroundState() );
}

Eigen::Vector3f
GSPhysics::getCharacterVelocity( CharacterController_t pCharacter ) {
    if ( pCharacter == nullptr || pCharacter->pCharacter == nullptr ) {
        return Eigen::Vector3f::Zero();
    }
    const JPH::Vec3 velocity = pCharacter->pCharacter->GetLinearVelocity();
    return Eigen::Vector3f( velocity.GetX(), velocity.GetY(), velocity.GetZ() );
}

bool GSPhysics::isCharacterOnGround( CharacterController_t pCharacter ) {
    if ( pCharacter == nullptr || pCharacter->pCharacter == nullptr ) {
        return false;
    }
    return pCharacter->pCharacter->GetGroundState() ==
           JPH::CharacterVirtual::EGroundState::OnGround;
}

void GSPhysics::moveSphereBody( RigidBody_t body,
                                const Eigen::Vector4f &position ) {
    if ( body == nullptr || pPhysicsSystem == nullptr )
        return;

    const Eigen::Vector4f origin = GetShiftOrigin();
    JPH::RVec3 joltPos( position.x() - origin.x(), position.y() - origin.y(),
                        position.z() - origin.z() );

    lockMutex();
    pPhysicsSystem->GetBodyInterface().SetPosition(
        body->GetID(), joltPos, JPH::EActivation::Activate );
    unlockMutex();
}

int32_t
GSPhysics::queryPhantomOverlaps( RigidBody_t phantom,
                                 const Eigen::Vector4f &center,
                                 float radius,
                                 std::vector<CameraPhantomOverlap> &overlaps ) {
    // FUN_00842bd0
    overlaps.clear();
    if ( pPhysicsSystem == nullptr )
        return 0;

    JPH::Ref<JPH::SphereShape> sphere = new JPH::SphereShape( radius );
    JPH::CollideShapeSettings settings;
    JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;

    const Eigen::Vector4f origin = GetShiftOrigin();
    JPH::RVec3 joltCenter( center.x() - origin.x(), center.y() - origin.y(),
                           center.z() - origin.z() );

    JPH::BodyID phantomID =
        ( phantom != nullptr ) ? phantom->GetID() : JPH::BodyID();
    JPH::IgnoreSingleBodyFilter bodyFilter( phantomID );

    lockMutex();
    pPhysicsSystem->GetNarrowPhaseQuery().CollideShape(
        sphere, JPH::Vec3::sReplicate( 1.0f ),
        JPH::RMat44::sTranslation( joltCenter ), settings, JPH::RVec3::sZero(),
        collector, {}, NonSensorLayerFilter(), bodyFilter );

    overlaps.reserve( collector.mHits.size() );
    for ( const JPH::CollideShapeResult &hit : collector.mHits ) {
        JPH::BodyLockRead lock( pPhysicsSystem->GetBodyLockInterfaceNoLock(),
                                hit.mBodyID2 );
        if ( !lock.Succeeded() ) {
            continue;
        }

        const JPH::Body &body = lock.GetBody();
        const JPH::Shape *pShape = body.GetShape();
        if ( pShape->GetSubType() == JPH::EShapeSubType::RotatedTranslated ) {
            pShape = static_cast<const JPH::RotatedTranslatedShape *>( pShape )
                         ->GetInnerShape();
        }

        CameraPhantomOverlap overlap;
        if ( pShape->GetSubType() == JPH::EShapeSubType::Sphere ) {
            overlap.Shape = eCameraOverlapShape::Sphere;
            overlap.Radius =
                static_cast<const JPH::SphereShape *>( pShape )->GetRadius();
        } else if ( pShape->GetSubType() == JPH::EShapeSubType::Capsule ) {
            overlap.Shape = eCameraOverlapShape::Capsule;
            overlap.Radius =
                static_cast<const JPH::CapsuleShape *>( pShape )->GetRadius();
        }

        const JPH::RVec3 bodyCenter = body.GetWorldSpaceBounds().GetCenter();
        overlap.Center = Eigen::Vector4f(
            static_cast<float>( bodyCenter.GetX() ) + origin.x(),
            static_cast<float>( bodyCenter.GetY() ) + origin.y(),
            static_cast<float>( bodyCenter.GetZ() ) + origin.z(), 1.0f );
        overlaps.push_back( overlap );
    }
    unlockMutex();

    return static_cast<int32_t>( overlaps.size() );
}

int32_t GSPhysics::castCameraSegment( const Eigen::Vector4f &from,
                                      const Eigen::Vector4f &to,
                                      std::vector<CameraSegmentHit> &hits ) {
    hits.clear();
    if ( pPhysicsSystem == nullptr ) {
        return 0;
    }

    const Eigen::Vector4f origin = GetShiftOrigin();
    const Eigen::Vector3f delta = ( to - from ).head<3>();
    if ( delta.squaredNorm() <= 0.0f ) {
        return 0;
    }

    const JPH::RRayCast ray( JPH::RVec3( from.x() - origin.x(),
                                         from.y() - origin.y(),
                                         from.z() - origin.z() ),
                             JPH::Vec3( delta.x(), delta.y(), delta.z() ) );

    JPH::RayCastSettings settings;
    settings.mTreatConvexAsSolid = false;
    settings.mBackFaceModeTriangles = JPH::EBackFaceMode::CollideWithBackFaces;

    JPH::AllHitCollisionCollector<JPH::CastRayCollector> collector;

    lockMutex();
    pPhysicsSystem->GetNarrowPhaseQuery().CastRay( ray, settings, collector,
                                                   JPH::BroadPhaseLayerFilter(),
                                                   NonSensorLayerFilter() );

    hits.reserve( collector.mHits.size() );
    for ( const JPH::RayCastResult &result : collector.mHits ) {
        const JPH::RVec3 point = ray.GetPointOnRay( result.mFraction );

        CameraSegmentHit hit;
        hit.Fraction = result.mFraction;
        hit.Point = Eigen::Vector4f(
            static_cast<float>( point.GetX() ) + origin.x(),
            static_cast<float>( point.GetY() ) + origin.y(),
            static_cast<float>( point.GetZ() ) + origin.z(), 1.0f );
        hit.Normal = Eigen::Vector4f( 0.0f, 1.0f, 0.0f, 0.0f );

        JPH::BodyLockRead lock( pPhysicsSystem->GetBodyLockInterfaceNoLock(),
                                result.mBodyID );
        if ( lock.Succeeded() ) {
            const JPH::Body &body = lock.GetBody();
            const JPH::Vec3 normal =
                body.GetWorldSpaceSurfaceNormal( result.mSubShapeID2, point );
            hit.Normal = Eigen::Vector4f( normal.GetX(), normal.GetY(),
                                          normal.GetZ(), 0.0f );
            hit.SurfaceClass = static_cast<uint32_t>( body.GetUserData() &
                                                      TDUSurface::kClassMask );
            const JPH::MotionProperties *pMotion = body.GetMotionProperties();
            if ( body.IsDynamic() && pMotion != nullptr &&
                 pMotion->GetInverseMass() != 0.0f ) {
                hit.Mass = 1.0f / pMotion->GetInverseMass();
            }
        }
        hits.push_back( hit );
    }
    unlockMutex();

    return static_cast<int32_t>( hits.size() );
}

bool GSPhysics::probeSurface( const Eigen::Vector4f &position,
                              bool bSearchAbove,
                              float &outHeight,
                              Eigen::Vector4f &outNormal,
                              bool bStaticAndRoadOnly ) {
    // FUN_00848cb0
    outHeight = kNoSurfaceSentinel;
    outNormal = Eigen::Vector4f( 0.0f, 1.0f, 0.0f, 0.0f );
    if ( pPhysicsSystem == nullptr ) {
        return false;
    }

    constexpr float kProbeMargin = 1.0f;  // the AABB's short axis
    constexpr float kProbeSpan = 1000.0f; // the AABB's tall axis

    const Eigen::Vector4f origin = GetShiftOrigin();
    const float startY = bSearchAbove ? ( position.y() - kProbeMargin )
                                      : ( position.y() + kProbeMargin );
    const float span = kProbeSpan + kProbeMargin;

    const JPH::RRayCast ray(
        JPH::RVec3( position.x() - origin.x(), startY - origin.y(),
                    position.z() - origin.z() ),
        JPH::Vec3( 0.0f, bSearchAbove ? span : -span, 0.0f ) );

    JPH::RayCastSettings settings;
    settings.mTreatConvexAsSolid = false;
    settings.mBackFaceModeTriangles = JPH::EBackFaceMode::CollideWithBackFaces;

    JPH::AllHitCollisionCollector<JPH::CastRayCollector> collector;
    const StaticOrRoadLayerFilter staticOrRoad;
    const NonSensorLayerFilter anyLayer;
    const JPH::ObjectLayerFilter &layerFilter =
        bStaticAndRoadOnly
            ? static_cast<const JPH::ObjectLayerFilter &>( staticOrRoad )
            : anyLayer;

    lockMutex();
    pPhysicsSystem->GetNarrowPhaseQuery().CastRay(
        ray, settings, collector, JPH::BroadPhaseLayerFilter(), layerFilter );

    bool bFound = false;
    float bestY = kNoSurfaceSentinel;
    JPH::BodyID bestBody;
    JPH::SubShapeID bestSubShape;
    JPH::RVec3 bestPoint = JPH::RVec3::sZero();
    for ( const JPH::RayCastResult &hit : collector.mHits ) {
        const JPH::RVec3 point = ray.GetPointOnRay( hit.mFraction );
        const float worldY = static_cast<float>( point.GetY() ) + origin.y();
        if ( !bFound || bestY < worldY ) {
            bFound = true;
            bestY = worldY;
            bestBody = hit.mBodyID;
            bestSubShape = hit.mSubShapeID2;
            bestPoint = point;
        }
    }

    if ( bFound ) {
        JPH::BodyLockRead lock( pPhysicsSystem->GetBodyLockInterfaceNoLock(),
                                bestBody );
        if ( lock.Succeeded() ) {
            const JPH::Vec3 normal = lock.GetBody().GetWorldSpaceSurfaceNormal(
                bestSubShape, bestPoint );
            outNormal = Eigen::Vector4f( normal.GetX(), normal.GetY(),
                                         normal.GetZ(), 0.0f );
        }
        outHeight = bestY;
    }
    unlockMutex();

    return bFound;
}

float GSPhysics::getGroundHeight( float x, float z ) {
    if ( pPhysicsSystem == nullptr ) {
        return -3.0e38f;
    }

    const float kTop = 10000.0f;
    const float kLength = 20000.0f;
    const Eigen::Vector4f origin = GetShiftOrigin();
    JPH::RRayCast ray(
        JPH::RVec3( x - origin.x(), kTop - origin.y(), z - origin.z() ),
        JPH::Vec3( 0.0f, -kLength, 0.0f ) );
    JPH::RayCastResult hit;

    lockMutex();
    const bool bHit = pPhysicsSystem->GetNarrowPhaseQuery().CastRay(
        ray, hit, JPH::BroadPhaseLayerFilter(), StaticOrRoadLayerFilter() );
    unlockMutex();

    if ( !bHit ) {
        return -3.0e38f;
    }
    return kTop - kLength * hit.mFraction;
}

void GSPhysics::forwardInput( PhysicsVehicle &vehicle, float /*deltaTime*/ ) {
    // FUN_00849380
    if ( vehicle.isInCinematicMode() ) {
        return;
    }

    uint8_t drivingMode = PhysicsVehicle::DM_Player;
    if ( bForcePlayerSlowDown ) {
        drivingMode = PhysicsVehicle::DM_SlowDown;
    } else if ( bForcePlayerParked ) {
        drivingMode = PhysicsVehicle::DM_Parked;
    }
    vehicle.setDrivingMode( drivingMode );
}

float GSPhysics::castWorldRay( const Eigen::Vector4f &origin,
                               const Eigen::Vector4f &direction,
                               const float length ) {
    // FUN_00849300
    static constexpr float kNoRayHit = -99999.99f; // DAT_00f418fc
    if ( pPhysicsSystem == nullptr ) {
        return kNoRayHit;
    }

    const Eigen::Vector4f shiftOrigin = GetShiftOrigin();
    const Eigen::Vector4f ray = direction * length;
    JPH::RRayCast joltRay( JPH::RVec3( origin.x() - shiftOrigin.x(),
                                       origin.y() - shiftOrigin.y(),
                                       origin.z() - shiftOrigin.z() ),
                           JPH::Vec3( ray.x(), ray.y(), ray.z() ) );
    JPH::RayCastResult hit;

    lockMutex();
    const bool bHit = pPhysicsSystem->GetNarrowPhaseQuery().CastRay(
        joltRay, hit, JPH::BroadPhaseLayerFilter(), StaticOrRoadLayerFilter() );
    unlockMutex();

    if ( !bHit ) {
        return kNoRayHit;
    }
    return hit.mFraction * length;
}

int32_t GSPhysics::gatherRoadShadowTriangles( const Eigen::Vector4f &bbMin,
                                              const Eigen::Vector4f &bbMax,
                                              Eigen::Vector4f *pOutVerts,
                                              int32_t maxVerts,
                                              int32_t *pOutNumBodies ) {
    // FUN_00845c20
    if ( pOutNumBodies != nullptr ) {
        *pOutNumBodies = 0;
    }
    if ( pPhysicsSystem == nullptr || pOutVerts == nullptr || maxVerts <= 0 ) {
        return 0;
    }

    const Eigen::Vector4f shiftOrigin = GetShiftOrigin();
    const JPH::Vec3 joltOrigin( shiftOrigin.x(), shiftOrigin.y(),
                                shiftOrigin.z() );
    const JPH::Vec3 p0 =
        JPH::Vec3( bbMin.x(), bbMin.y(), bbMin.z() ) - joltOrigin;
    const JPH::Vec3 p1 =
        JPH::Vec3( bbMax.x(), bbMax.y(), bbMax.z() ) - joltOrigin;
    const JPH::AABox queryBox( JPH::Vec3::sMin( p0, p1 ),
                               JPH::Vec3::sMax( p0, p1 ) );

    struct RoadBody {
        JPH::RefConst<JPH::Shape> shape;
        JPH::Mat44 com;
    };
    std::vector<RoadBody> roadBodies;

    lockMutex();
    JPH::BodyIDVector bodyIDs;
    pPhysicsSystem->GetBodies( bodyIDs );
    const JPH::BodyInterface &bi = pPhysicsSystem->GetBodyInterfaceNoLock();
    roadBodies.reserve( bodyIDs.size() );
    for ( const JPH::BodyID id : bodyIDs ) {
        const uint32_t surf = static_cast<uint32_t>( bi.GetUserData( id ) &
                                                     TDUSurface::kClassMask );
        if ( surf != TDUSurface::Paved && surf != TDUSurface::RoadMesh ) {
            continue;
        }
        JPH::RefConst<JPH::Shape> shape = bi.GetShape( id );
        if ( shape.GetPtr() == nullptr ||
             shape->GetSubType() != JPH::EShapeSubType::Mesh ) {
            continue;
        }
        roadBodies.push_back( { shape, bi.GetCenterOfMassTransform( id ) } );
    }
    unlockMutex();

    int32_t numVerts = 0;
    constexpr int32_t kTriBatch = 64;
    JPH::Float3 tri[3 * kTriBatch];
    for ( const RoadBody &rb : roadBodies ) {
        const int32_t numVertsBefore = numVerts;
        JPH::Shape::GetTrianglesContext ctx;
        rb.shape->GetTrianglesStart( ctx, queryBox, rb.com.GetTranslation(),
                                     rb.com.GetQuaternion(),
                                     JPH::Vec3::sReplicate( 1.0f ) );
        for ( ;; ) {
            const int32_t n = rb.shape->GetTrianglesNext( ctx, kTriBatch, tri );
            if ( n == 0 ) {
                break;
            }
            for ( int32_t t = 0; t < n; t++ ) {
                for ( int32_t v = 0; v < 3; v++ ) {
                    if ( numVerts >= maxVerts ) {
                        return numVerts;
                    }
                    const JPH::Float3 &f = tri[3 * t + v];
                    pOutVerts[numVerts++] = Eigen::Vector4f(
                        f.x + shiftOrigin.x(), f.y + shiftOrigin.y(),
                        f.z + shiftOrigin.z(), 1.0f );
                }
            }
        }

        if ( pOutNumBodies != nullptr && numVerts > numVertsBefore ) {
            ( *pOutNumBodies )++;
        }
    }
    return numVerts;
}

void GSPhysics::createHeightfields() {
    // FUN_00842b10
    for ( auto &hf : pHeightfields ) {
        hf.pBody = nullptr;
        hf.pKey = nullptr;
    }
}

bool GSPhysics::isRegionCovered( float minX,
                                 float minZ,
                                 float maxX,
                                 float maxZ,
                                 float centreY,
                                 float halfY ) {
    if ( pPhysicsSystem == nullptr ) {
        return false;
    }

    const Eigen::Vector4f shiftOrigin = GetShiftOrigin();
    const JPH::Vec3 halfExtent( 0.5f * ( maxX - minX ), halfY,
                                0.5f * ( maxZ - minZ ) );
    const JPH::Vec3 centre( 0.5f * ( minX + maxX ) - shiftOrigin.x(),
                            centreY - shiftOrigin.y(),
                            0.5f * ( minZ + maxZ ) - shiftOrigin.z() );

    JPH::BoxShape probe( halfExtent, 0.0f );
    probe.SetEmbedded();

    JPH::AnyHitCollisionCollector<JPH::CollideShapeCollector> hit;
    pPhysicsSystem->GetNarrowPhaseQuery().CollideShape(
        &probe, JPH::Vec3::sReplicate( 1.0f ),
        JPH::Mat44::sTranslation( centre ), JPH::CollideShapeSettings(),
        JPH::RVec3::sZero(), hit, JPH::BroadPhaseLayerFilter(),
        StaticOrRoadLayerFilter(), NonTerrainBodyFilter() );
    return hit.HadHit();
}

bool GSPhysics::hasCollisionBelow( float worldX,
                                   float worldZ,
                                   float startY,
                                   float depth ) {
    if ( pPhysicsSystem == nullptr ) {
        return false;
    }

    const Eigen::Vector4f shiftOrigin = GetShiftOrigin();
    const JPH::RRayCast ray{ JPH::RVec3( worldX - shiftOrigin.x(),
                                         startY - shiftOrigin.y(),
                                         worldZ - shiftOrigin.z() ),
                             JPH::Vec3( 0.0f, -depth, 0.0f ) };

    JPH::RayCastResult result;
    if ( !pPhysicsSystem->GetNarrowPhaseQuery().CastRay(
             ray, result, JPH::BroadPhaseLayerFilter(),
             StaticOrRoadLayerFilter(), NonTerrainBodyFilter() ) ) {
        return false;
    }
    return true;
}

RigidBody_t GSPhysics::addHeightfield( const void *pKey,
                                       const float *pHeights,
                                       int32_t stride,
                                       float cellSize,
                                       float originX,
                                       float originZ,
                                       const float *pTileErrors,
                                       int32_t numTileX,
                                       int32_t numTileZ ) {
    // FUN_0084a390 -> FUN_00848b50
    if ( pPhysicsSystem == nullptr || pHeights == nullptr || stride <= 1 ||
         cellSize <= 0.0f )
        return nullptr;

    // Guard against a malformed heightmap descriptor before sizing buffers off
    // it (a garbage stride would allocate gigabytes and crash inside Jolt).
    if ( stride > 1025 ) {
        OTDU_LOG_WARN( "Skipping terrain heightfield: implausible stride %d\n",
                       stride );
        return nullptr;
    }

    constexpr uint32_t kBlockSize = 2;
    uint32_t sampleCount = kBlockSize;
    while ( sampleCount < static_cast<uint32_t>( stride ) )
        sampleCount <<= 1;

    std::vector<float> samples(
        static_cast<size_t>( sampleCount ) * sampleCount,
        JPH::HeightFieldShapeConstants::cNoCollisionValue );
    for ( int32_t row = 0; row < stride; row++ ) {
        for ( int32_t col = 0; col < stride; col++ ) {
            float h = pHeights[row * stride + col];
            // Drop non-finite samples and terrain holes (orig block flag 0xFE
            // -> -100000) to no-collision cells.
            if ( !std::isfinite( h ) || h < -50000.0f || h > 50000.0f )
                h = JPH::HeightFieldShapeConstants::cNoCollisionValue;
            samples[static_cast<size_t>( row ) * sampleCount + col] = h;
        }
    }

    const int32_t quadsPerTile =
        ( numTileX > 0 ) ? ( stride - 1 ) / numTileX : 0;
    if ( pTileErrors != nullptr && numTileX > 0 && numTileZ > 0 &&
         quadsPerTile > 0 ) {
        int32_t numBakedHoles = 0;
        int32_t numSubmergedHoles = 0;
        int32_t numKeptTiles = 0;
        const int32_t numCells = stride - 1;
        std::vector<uint8_t> covered(
            static_cast<size_t>( numCells ) * numCells, 0u );
        for ( int32_t tileZ = 0; tileZ < numTileZ; tileZ++ ) {
            for ( int32_t tileX = 0; tileX < numTileX; tileX++ ) {
                const float *pErrors =
                    pTileErrors + ( tileZ * numTileX + tileX ) * 5;
                float error0 = pErrors[0];
                const bool bBaked = error0 < 0.0f;
                if ( pErrors[4] < kHeightmapSeaLevel &&
                     error0 < kHeightmapFlatThreshold ) {
                    error0 = kHeightmapSeaLevel;
                }
                if ( error0 >= 0.0f ) {
                    continue;
                }
                if ( bBaked ) {
                    numBakedHoles++;
                } else {
                    numSubmergedHoles++;
                }

                const int32_t firstCell = tileX * quadsPerTile;
                const int32_t firstCellZ = tileZ * quadsPerTile;
                const float tileX0 = originX + firstCell * cellSize;
                const float tileZ0 = originZ + firstCellZ * cellSize;
                const float tileSize = quadsPerTile * cellSize;

                // Cheap whole-tile reject first
                if ( !isRegionCovered( tileX0, tileZ0, tileX0 + tileSize,
                                       tileZ0 + tileSize, pErrors[4],
                                       kTileProbeHalfHeight ) ) {
                    numKeptTiles++;
                    continue;
                }

                for ( int32_t r = 0; r < quadsPerTile; r++ ) {
                    for ( int32_t c = 0; c < quadsPerTile; c++ ) {
                        const int32_t cellX = firstCell + c;
                        const int32_t cellZ = firstCellZ + r;
                        // Start just above this cell's own terrain surface
                        float surfaceY = -1.0e30f;
                        for ( int32_t dz = 0; dz <= 1; dz++ ) {
                            for ( int32_t dx = 0; dx <= 1; dx++ ) {
                                const float h =
                                    pHeights[( cellZ + dz ) * stride + cellX +
                                             dx];
                                if ( std::isfinite( h ) && h > surfaceY ) {
                                    surfaceY = h;
                                }
                            }
                        }
                        if ( surfaceY <= -1.0e29f ) {
                            continue;
                        }
                        const float cx = originX + ( cellX + 0.5f ) * cellSize;
                        const float cz = originZ + ( cellZ + 0.5f ) * cellSize;
                        if ( hasCollisionBelow( cx, cz,
                                                surfaceY + kSupportProbeRise,
                                                kSupportProbeDepth ) ) {
                            covered[static_cast<size_t>( cellZ ) * numCells +
                                    cellX] = 1u;
                        }
                    }
                }
            }
        }

        for ( int32_t row = 0; row < stride; row++ ) {
            for ( int32_t col = 0; col < stride; col++ ) {
                bool bDrop = true;
                for ( int32_t dr = -1; dr <= 0 && bDrop; dr++ ) {
                    for ( int32_t dc = -1; dc <= 0; dc++ ) {
                        const int32_t cr = row + dr;
                        const int32_t cc = col + dc;
                        const bool bInside = cr >= 0 && cr < numCells &&
                                             cc >= 0 && cc < numCells;
                        if ( !bInside ||
                             !covered[static_cast<size_t>( cr ) * numCells +
                                      cc] ) {
                            bDrop = false;
                            break;
                        }
                    }
                }
                if ( bDrop ) {
                    samples[static_cast<size_t>( row ) * sampleCount + col] =
                        JPH::HeightFieldShapeConstants::cNoCollisionValue;
                }
            }
        }

        if ( numBakedHoles != 0 || numSubmergedHoles != 0 ) {
            OTDU_LOG_DEBUG(
                "Terrain heightfield: %d/%d tile(s) marked (%d authored, %d "
                "submerged); %d kept whole for lack of authored cover\n",
                numBakedHoles + numSubmergedHoles, numTileX * numTileZ,
                numBakedHoles, numSubmergedHoles, numKeptTiles );
        }
    }

    JPH::HeightFieldShapeSettings shapeSettings(
        samples.data(), JPH::Vec3::sZero(),
        JPH::Vec3( cellSize, 1.0f, cellSize ), sampleCount );
    shapeSettings.mBlockSize = kBlockSize;

    shapeSettings.mMaterials.push_back( TDUPhysicsMaterials::get(
        static_cast<uint32_t>( eMaterialType::MT_Asphalt ) ) );

    JPH::ShapeSettings::ShapeResult shapeResult = shapeSettings.Create();
    if ( !shapeResult.IsValid() ) {
        OTDU_LOG_WARN( "Failed to build terrain heightfield shape\n" );
        return nullptr;
    }

    PhysicsHeightfield *pSlot = nullptr;
    for ( auto &hf : pHeightfields ) {
        if ( hf.pBody == nullptr ) {
            pSlot = &hf;
            break;
        }
    }
    if ( pSlot == nullptr ) {
        OTDU_LOG_WARN( "No free physics heightfield slot\n" );
        return nullptr;
    }

    const Eigen::Vector4f shiftOrigin = GetShiftOrigin();
    JPH::BodyCreationSettings bodySettings(
        shapeResult.Get(),
        JPH::RVec3( originX - shiftOrigin.x(), -shiftOrigin.y(),
                    originZ - shiftOrigin.z() ),
        JPH::Quat::sIdentity(), JPH::EMotionType::Static,
        TDUPhysLayers::NON_MOVING );
    bodySettings.mUserData = TDUSurface::Terrain;
    bodySettings.mFriction = 0.5f;

    lockMutex();
    JPH::Body *pBody =
        pPhysicsSystem->GetBodyInterface().CreateBody( bodySettings );
    if ( pBody != nullptr ) {
        pPhysicsSystem->GetBodyInterface().AddBody(
            pBody->GetID(), JPH::EActivation::DontActivate );
        pSlot->pBody = pBody;
        pSlot->pKey = pKey;
    }
    unlockMutex();

    return pBody;
}

void GSPhysics::refreshHeightmapCollision() {
    // FUN_00844e60
    // NOP (for Jolt)
}

void GSPhysics::removeHeightfield( const void *pKey ) {
    if ( pPhysicsSystem == nullptr || pKey == nullptr )
        return;

    for ( auto &hf : pHeightfields ) {
        if ( hf.pKey == pKey && hf.pBody != nullptr ) {
            JPH::Body *pBody = static_cast<JPH::Body *>( hf.pBody );
            lockMutex();
            JPH::BodyInterface &bodyInterface =
                pPhysicsSystem->GetBodyInterface();
            bodyInterface.RemoveBody( pBody->GetID() );
            bodyInterface.DestroyBody( pBody->GetID() );
            unlockMutex();
            hf.pBody = nullptr;
            hf.pKey = nullptr;
            return;
        }
    }
}

static uint32_t PhysTypeForShape( bool bBvTreeShape, bool bRoad ) {
    // FUN_00844f00
    if ( !bBvTreeShape ) {
        return TDUSurface::Generic;
    }
    return bRoad ? TDUSurface::RoadMesh : TDUSurface::Paved;
}

void GSPhysics::addSectorCollision( const void *pKey,
                                    void *pBlob,
                                    uint32_t blobSize,
                                    float baseX,
                                    float baseZ,
                                    bool bRoadBlob ) {
    // Sector static world collision
    if ( pPhysicsSystem == nullptr || pBlob == nullptr || pKey == nullptr )
        return;

    HavokPackfile packfile;
    bool bParsed = false;
    HavokContainer container;
    if ( container.parse( pBlob, blobSize ) ) {
        bParsed = packfile.parse( container.getHavokScene(),
                                  container.getHavokSceneSize() );
    }
    if ( !bParsed ) {
        bParsed =
            packfile.parse( static_cast<const uint8_t *>( pBlob ), blobSize );
    }
    if ( !bParsed ) {
        OTDU_LOG_WARN( "Sector collision: blob is neither a physics container "
                       "nor a Havok packfile (size=%u)\n",
                       blobSize );
        return;
    }

    std::vector<HavokToJolt::RigidBody> bodies;
    HavokToJolt::buildBodies( packfile, bodies );
    if ( bodies.empty() ) {
        // Expected until the MOPP/triangle-mesh converter lands: world/road
        // shells are hkMoppBvTreeShape-wrapped meshes (type 0x18) which
        // HavokToJolt::buildShape does not yet handle.
        OTDU_LOG_WARN( "Sector collision: Havok scene produced no collidable "
                       "bodies (mesh converter pending?)\n" );
        return;
    }

    const Eigen::Vector4f shiftOrigin = GetShiftOrigin();

    JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();
    lockMutex();

    PhysicsSHK *pSlot = nullptr;
    for ( PhysicsSHK *it = pSectorsBegin; it != pSectorsEnd; ++it ) {
        if ( it->pKey == nullptr && it->NumObjects == 0 ) {
            pSlot = it;
            pSlot->pKey = pKey;
            break;
        }
    }
    if ( pSlot == nullptr ) {
        unlockMutex();
        OTDU_LOG_WARN( "Sector collision: no free sector slot\n" );
        return;
    }

    for ( const HavokToJolt::RigidBody &body : bodies ) {
        if ( pSlot->NumObjects >= pSlot->MaxNumObjects )
            break;

        const JPH::RVec3 position(
            baseX + body.Position.GetX() - shiftOrigin.x(),
            body.Position.GetY() - shiftOrigin.y(),
            baseZ + body.Position.GetZ() - shiftOrigin.z() );

        const uint32_t physType =
            PhysTypeForShape( body.bBvTreeShape, bRoadBlob );

        JPH::BodyCreationSettings settings( body.Shape, position, body.Rotation,
                                            JPH::EMotionType::Static,
                                            TDUPhysLayers::NON_MOVING );
        settings.mUserData = physType;
        settings.mFriction = body.Friction;
        settings.mRestitution = body.Restitution;

        JPH::Body *pBody = bodyInterface.CreateBody( settings );
        if ( pBody != nullptr ) {
            bodyInterface.AddBody( pBody->GetID(),
                                   JPH::EActivation::DontActivate );
            PhysicsSectorObj &obj = pSlot->pObjects[pSlot->NumObjects++];
            obj.pRigidBody = pBody;
            obj.pShape = body.Shape;

            // TEMP: REMOVEME:
            if ( !body.Rotation.IsClose( JPH::Quat::sIdentity(), 1.0e-4f ) ) {
                const JPH::Vec3 sz = pBody->GetWorldSpaceBounds().GetSize();
                // Bucket by vertical (Y) size: >8m = an upright wall/building.
                if ( sz.GetY() > 8.0f )
                    gTallBodyCount++;
                else
                    gFlatBodyCount++;
                if ( sz.GetY() > gMaxBodyY )
                    gMaxBodyY = sz.GetY();
            }
        }
    }
    unlockMutex();

    // TEMP: REMOVEME: absolute AABB of body positions + how many carry a
    // non-identity rotation (road-surface meshes are axis-aligned; .shk world
    // objects rotate — a placement/rotation regression shows up here).
    JPH::Vec3 lo( 1e18f, 1e18f, 1e18f ), hi( -1e18f, -1e18f, -1e18f );
    int32_t nonIdentityRot = 0;
    for ( const HavokToJolt::RigidBody &body : bodies ) {
        const JPH::Vec3 p( baseX + body.Position.GetX(), body.Position.GetY(),
                           baseZ + body.Position.GetZ() );
        lo = JPH::Vec3::sMin( lo, p );
        hi = JPH::Vec3::sMax( hi, p );
        if ( !body.Rotation.IsClose( JPH::Quat::sIdentity(), 1.0e-4f ) )
            nonIdentityRot++;
    }
    OTDU_LOG_DEBUG(
        "Sector collision: %d bodies; areaBase (%.0f,%.0f); body0 "
        "abs (%.0f,%.0f,%.0f); AABB (%.0f,%.0f,%.0f)..(%.0f,%.0f,"
        "%.0f); %d/%d rotated\n",
        pSlot->NumObjects, baseX, baseZ, baseX + bodies[0].Position.GetX(),
        bodies[0].Position.GetY(), baseZ + bodies[0].Position.GetZ(), lo.GetX(),
        lo.GetY(), lo.GetZ(), hi.GetX(), hi.GetY(), hi.GetZ(), nonIdentityRot,
        (int)bodies.size() );
    OTDU_LOG_DEBUG( "  orientation tally so far: %d tall(>8m Y) / %d flat; max "
                    "Y-size %.1f m\n",
                    gTallBodyCount, gFlatBodyCount, gMaxBodyY );
}

void GSPhysics::removeSectorCollision( const void *pKey ) {
    if ( pPhysicsSystem == nullptr || pKey == nullptr )
        return;

    // A sector may own more than one collision slot (its .shk world blob and
    // its -R.shk road blob are both keyed by the Sector), so tear down every
    // match.
    lockMutex();
    // FUN_00842290
    roadPhantomManager.removeSectorRoadPhantoms( pPhysicsSystem, pKey );
    roadCurves.erase( std::remove_if( roadCurves.begin(), roadCurves.end(),
                                      [pKey]( const PhysicsRoadCurve &curve ) {
                                          return curve.pKey == pKey;
                                      } ),
                      roadCurves.end() );
    unlockMutex();

    for ( PhysicsSHK *it = pSectorsBegin; it != pSectorsEnd; ++it ) {
        if ( it->pKey != pKey )
            continue;

        lockMutex();
        JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();
        for ( int32_t i = 0; i < it->NumObjects; i++ ) {
            PhysicsSectorObj &obj = it->pObjects[i];
            if ( obj.pRigidBody != nullptr ) {
                JPH::Body *pBody = static_cast<JPH::Body *>( obj.pRigidBody );
                bodyInterface.RemoveBody( pBody->GetID() );
                bodyInterface.DestroyBody( pBody->GetID() );
                obj.pRigidBody = nullptr;
            }
            obj.pShape = nullptr; // release the shape ref
        }
        it->NumObjects = 0;
        it->pKey = nullptr;
        unlockMutex();
    }
}

PhysFactory *
GSPhysics::installStaticCollision( void *pBlob,
                                   uint32_t blobSize,
                                   uint64_t factoryHashcode,
                                   const Matrix4x4f_t *pTransform ) {
    // FUN_00848a20 + FUN_008487e0
    if ( pBlob == nullptr || blobSize == 0 ) {
        return nullptr;
    }
    if ( pPhysicsSystem == nullptr ) {
        OTDU_LOG_WARN( "Static collision: physics is not up yet (blob %u "
                       "bytes dropped)\n",
                       blobSize );
        return nullptr;
    }

    PhysicsLock lock;

    // Never install the same blob twice, and claim a free slot.
    PhysFactory *pFactory = nullptr;
    for ( PhysFactory &factory : pFactories ) {
        if ( factory.isInitialized() ) {
            if ( factory.getSourceBlob() == pBlob ) {
                return nullptr;
            }
        } else if ( pFactory == nullptr ) {
            pFactory = &factory;
        }
    }
    if ( pFactory == nullptr ) {
        OTDU_LOG_WARN( "Static collision: no free physics factory slot\n" );
        return nullptr;
    }

    HavokPackfile packfile;
    bool bParsed = false;
    HavokContainer container;
    if ( container.parse( pBlob, blobSize ) ) {
        bParsed = packfile.parse( container.getHavokScene(),
                                  container.getHavokSceneSize() );
    }
    if ( !bParsed ) {
        bParsed =
            packfile.parse( static_cast<const uint8_t *>( pBlob ), blobSize );
    }
    if ( !bParsed ) {
        OTDU_LOG_WARN( "Static collision: blob is neither a physics container "
                       "nor a Havok packfile (size=%u)\n",
                       blobSize );
        return nullptr;
    }

    HavokFactoryData *pData = new HavokFactoryData();
    HavokToJolt::buildBodies( packfile, pData->Bodies );
    if ( pData->Bodies.empty() ) {
        OTDU_LOG_WARN(
            "Static collision: Havok scene produced no collidable bodies\n" );
        delete pData;
        return nullptr;
    }

    const Eigen::Vector4f shiftOrigin = GetShiftOrigin();
    JPH::Quat worldRot = JPH::Quat::sIdentity();
    JPH::Vec3 worldTrans( -shiftOrigin.x(), -shiftOrigin.y(),
                          -shiftOrigin.z() );
    if ( pTransform != nullptr ) {
        const Eigen::Quaternionf q(
            Eigen::Matrix3f( pTransform->block<3, 3>( 0, 0 ).transpose() ) );
        worldRot = JPH::Quat( q.x(), q.y(), q.z(), q.w() ).Normalized();
        worldTrans +=
            JPH::Vec3( pTransform->row( 3 ).x(), pTransform->row( 3 ).y(),
                       pTransform->row( 3 ).z() );
    }

    JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();
    int32_t numPlaced = 0;
    for ( const HavokToJolt::RigidBody &body : pData->Bodies ) {
        if ( body.Shape.GetPtr() == nullptr ) {
            continue;
        }

        const uint32_t physType = PhysTypeForShape( body.bBvTreeShape, false );

        const JPH::RVec3 position( worldRot * body.Position + worldTrans );
        const JPH::Quat rotation = ( worldRot * body.Rotation ).Normalized();

        JPH::BodyCreationSettings settings( body.Shape, position, rotation,
                                            JPH::EMotionType::Static,
                                            TDUPhysLayers::NON_MOVING );
        settings.mUserData = physType;
        settings.mFriction = body.Friction;
        settings.mRestitution = body.Restitution;

        JPH::Body *pBody = bodyInterface.CreateBody( settings );
        if ( pBody != nullptr ) {
            bodyInterface.AddBody( pBody->GetID(),
                                   JPH::EActivation::DontActivate );
            pData->LiveBodyIDs.push_back(
                pBody->GetID().GetIndexAndSequenceNumber() );
            numPlaced++;
        }
    }

    pFactory->initialize( pData, factoryHashcode, pBlob );
    const JPH::Vec3 origin =
        worldTrans +
        JPH::Vec3( shiftOrigin.x(), shiftOrigin.y(), shiftOrigin.z() );
    OTDU_LOG_DEBUG( "Static collision: %d bodies at (%.0f,%.0f,%.0f) "
                    "(hashcode 0x%016llx)\n",
                    numPlaced, (double)origin.GetX(), (double)origin.GetY(),
                    (double)origin.GetZ(),
                    (unsigned long long)factoryHashcode );
    return pFactory;
}

void GSPhysics::addSectorRoadGround( const void *pKey,
                                     void *pBlob,
                                     uint32_t blobSize ) {
    // FUN_00848530
    if ( pPhysicsSystem == nullptr || pBlob == nullptr || pKey == nullptr ||
         blobSize < 0x124u )
        return;

    const uint8_t *blob = static_cast<const uint8_t *>( pBlob );
    auto readF = []( const uint8_t *p ) -> float {
        float v;
        memcpy( &v, p, 4 );
        return v;
    };
    auto readU = []( const uint8_t *p ) -> uint32_t {
        uint32_t v;
        memcpy( &v, p, 4 );
        return v;
    };
    auto readVec3 = [&]( const uint8_t *p ) -> JPH::Vec3 {
        return JPH::Vec3( readF( p ), readF( p + 4 ), readF( p + 8 ) );
    };

    struct RoadPhantomRecord {
        Eigen::Vector3f BoundsMin;
        Eigen::Vector3f BoundsMax;
        int32_t RoadID;
        int32_t EntityIndex;
        int32_t CurveIndex;
    };
    std::vector<RoadPhantomRecord> phantomRecords;
    std::vector<PhysicsRoadCurve> curveRecords;

    JPH::TriangleList triangles;
    JPH::Vec3 originRef = JPH::Vec3::sZero(); // rebase for float precision
    bool bHaveOrigin = false;
    JPH::Vec3 prevLat( 0, 0, 1 ); // fallback lateral if the tangent degenerates
    int32_t recordCount = 0;

    uint32_t off = 0;
    while ( off + 0x124u <= blobSize ) {
        const uint8_t *rec = blob + off;
        if ( readU( rec ) != kSR20Magic )
            break;
        const int32_t N = static_cast<int32_t>( readU( rec + 0xac ) );
        if ( N < 2 || N > 256 )
            break;
        const uint32_t e0Off = readU( rec + 0xe0 ); // heights[N]
        const uint32_t e4Off = readU( rec + 0xe4 ); // lateral[N]
        const uint32_t footer = off + e4Off + static_cast<uint32_t>( N ) * 4u;
        if ( off + e0Off + static_cast<uint32_t>( N ) * 4u > blobSize ||
             footer + 4u > blobSize )
            break;
        if ( readU( blob + footer ) != kER20Magic )
            break;

        const JPH::Vec3 P0 = readVec3( rec + 0x10 );
        const JPH::Vec3 P1 = readVec3( rec + 0x20 );
        const JPH::Vec3 P2 = readVec3( rec + 0x30 );
        const JPH::Vec3 P3 = readVec3( rec + 0x40 );
        const float *heights = reinterpret_cast<const float *>( rec + e0Off );
        const float *lateral = reinterpret_cast<const float *>( rec + e4Off );
        if ( !bHaveOrigin ) {
            originRef = P0;
            bHaveOrigin = true;
        }

        std::vector<JPH::Vec3> grid;
        grid.reserve( static_cast<size_t>( kLongSteps + 1 ) * N );
        for ( int32_t j = 0; j <= kLongSteps; ++j ) {
            const float u = static_cast<float>( j ) / kLongSteps;
            const float iu = 1.0f - u;
            const JPH::Vec3 S = P0 * ( iu * iu * iu ) +
                                P1 * ( 3.0f * u * iu * iu ) +
                                P2 * ( 3.0f * u * u * iu ) + P3 * ( u * u * u );
            const JPH::Vec3 T = ( P1 - P0 ) * ( 3.0f * iu * iu ) +
                                ( P2 - P1 ) * ( 6.0f * u * iu ) +
                                ( P3 - P2 ) * ( 3.0f * u * u );
            JPH::Vec3 latDir( -T.GetZ(), 0.0f, T.GetX() );
            const float latLen = latDir.Length();
            if ( latLen > 1.0e-4f )
                prevLat = latDir / latLen;
            for ( int32_t k = 0; k < N; ++k ) {
                grid.push_back( ( S + prevLat * lateral[k] +
                                  JPH::Vec3( 0.0f, heights[k], 0.0f ) ) -
                                originRef );
            }
        }

        auto pushTri = [&]( const JPH::Vec3 &v0, const JPH::Vec3 &v1,
                            const JPH::Vec3 &v2 ) {
            const JPH::Vec3 n = ( v1 - v0 ).Cross( v2 - v0 );
            if ( n.LengthSq() < 1.0e-8f )
                return;
            if ( n.GetY() < 0.0f )
                triangles.push_back( JPH::Triangle( v0, v2, v1 ) );
            else
                triangles.push_back( JPH::Triangle( v0, v1, v2 ) );
        };
        for ( int32_t j = 0; j < kLongSteps; ++j ) {
            for ( int32_t k = 0; k < N - 1; ++k ) {
                const JPH::Vec3 &a = grid[j * N + k];
                const JPH::Vec3 &b = grid[( j + 1 ) * N + k];
                const JPH::Vec3 &c = grid[( j + 1 ) * N + k + 1];
                const JPH::Vec3 &d = grid[j * N + k + 1];
                pushTri( a, b, c );
                pushTri( a, c, d );
            }
        }
        // FUN_00820cd0
        static constexpr float kRoadPhantomMargin = 5.0f;
        RoadPhantomRecord phantom;
        phantom.BoundsMin =
            Eigen::Vector3f( readF( rec + 0xb0 ), readF( rec + 0xb4 ),
                             readF( rec + 0xb8 ) ) -
            Eigen::Vector3f::Constant( kRoadPhantomMargin );
        phantom.BoundsMax =
            Eigen::Vector3f( readF( rec + 0xc0 ), readF( rec + 0xc4 ),
                             readF( rec + 0xc8 ) ) +
            Eigen::Vector3f::Constant( kRoadPhantomMargin );
        phantom.RoadID = rec[0xd0] | ( rec[0xd1] << 8 );
        phantom.CurveIndex = rec[0xd2];
        phantom.EntityIndex = rec[0xd3];
        phantomRecords.push_back( phantom );

        PhysicsRoadCurve curve;
        curve.pKey = pKey;
        const JPH::Vec3 controlPoints[4] = { P0, P1, P2, P3 };
        for ( int32_t i = 0; i < 4; i++ ) {
            curve.ControlPoints[i] = Eigen::Vector4f(
                controlPoints[i].GetX(), controlPoints[i].GetY(),
                controlPoints[i].GetZ(), 1.0f );
        }
        curve.bCubic = ( readU( rec + 0xc ) != 0u );
        curve.RoadID = static_cast<uint16_t>( phantom.RoadID );
        curve.CurveIndex = rec[0xd2];
        curve.EntityIndex = rec[0xd3];
        curveRecords.push_back( curve );

        recordCount++;
        off = ( footer + 4u + 0xfu ) & ~0xfu;
    }

    if ( triangles.empty() ) {
        OTDU_LOG_WARN( "SR20 road ground: no triangles produced (size=%u)\n",
                       blobSize );
        return;
    }

    JPH::MeshShapeSettings meshSettings( triangles );
    meshSettings.Sanitize();
    JPH::ShapeSettings::ShapeResult meshResult = meshSettings.Create();
    if ( !meshResult.IsValid() ) {
        OTDU_LOG_WARN( "SR20 road ground: mesh build failed: %s\n",
                       meshResult.GetError().c_str() );
        return;
    }

    const Eigen::Vector4f shiftOrigin = GetShiftOrigin();
    JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();
    lockMutex();

    PhysicsSHK *pSlot = nullptr;
    for ( PhysicsSHK *it = pSectorsBegin; it != pSectorsEnd; ++it ) {
        if ( it->pKey == nullptr && it->NumObjects == 0 ) {
            pSlot = it;
            pSlot->pKey = pKey;
            break;
        }
    }
    if ( pSlot == nullptr ) {
        unlockMutex();
        OTDU_LOG_WARN( "SR20 road ground: no free sector slot\n" );
        return;
    }

    JPH::BodyCreationSettings settings(
        meshResult.Get(),
        JPH::RVec3( originRef - JPH::Vec3( shiftOrigin.x(), shiftOrigin.y(),
                                           shiftOrigin.z() ) ),
        JPH::Quat::sIdentity(), JPH::EMotionType::Static,
        TDUPhysLayers::ROAD_GROUND );
    settings.mUserData = TDUSurface::RoadGround;
    settings.mFriction = 1.0f; // FUN_00848530 (FLOAT_00f4165c)
    JPH::Body *pBody = ( pSlot->NumObjects < pSlot->MaxNumObjects )
                           ? bodyInterface.CreateBody( settings )
                           : nullptr;
    if ( pBody != nullptr ) {
        bodyInterface.AddBody( pBody->GetID(), JPH::EActivation::DontActivate );
        PhysicsSectorObj &obj = pSlot->pObjects[pSlot->NumObjects++];
        obj.pRigidBody = pBody;
        obj.pShape = meshResult.Get();
    }

    // FUN_00820b00 + FUN_00819ec0
    const Eigen::Vector3f shift = shiftOrigin.head<3>();
    for ( const RoadPhantomRecord &phantom : phantomRecords ) {
        roadPhantomManager.createRoadPhantom(
            pPhysicsSystem, pKey, phantom.BoundsMin - shift,
            phantom.BoundsMax - shift, phantom.RoadID, phantom.EntityIndex,
            phantom.CurveIndex );
    }
    roadCurves.insert( roadCurves.end(), curveRecords.begin(),
                       curveRecords.end() );
    unlockMutex();

    OTDU_LOG_DEBUG( "SR20 road ground: %d records, %u triangles\n", recordCount,
                    (uint32_t)triangles.size() );
}

void GSPhysics::queuePropCollisionFactory( const void *pBlob,
                                           uint32_t blobSize,
                                           PhysFactory **ppOwnerSlot ) {
    // FUN_00845240
    PhysicsLock lock;

    if ( pBlob == nullptr || blobSize == 0 ) {
        return;
    }
    if ( ppOwnerSlot != nullptr ) {
        for ( const PendingPropFactory &pending : pendingPropFactories ) {
            if ( pending.ppOwnerSlot == ppOwnerSlot ) {
                return;
            }
        }
    }

    PendingPropFactory &pending = pendingPropFactories.emplace_back();
    pending.Blob.assign( static_cast<const uint8_t *>( pBlob ),
                         static_cast<const uint8_t *>( pBlob ) + blobSize );
    pending.ppOwnerSlot = ppOwnerSlot;
}

void GSPhysics::cancelPendingPropFactory( PhysFactory **ppOwnerSlot ) {
    if ( ppOwnerSlot == nullptr ) {
        return;
    }
    PhysicsLock lock;

    for ( auto it = pendingPropFactories.begin();
          it != pendingPropFactories.end(); ) {
        if ( it->ppOwnerSlot == ppOwnerSlot ) {
            it = pendingPropFactories.erase( it );
        } else {
            ++it;
        }
    }
}

// The two hedge assets ("V_HAIE_M" / "V_HAIE_O") are authored with their
// collision box transposed relative to their render instance: the original
// compensates by swapping the X and Z scale components before instancing them
// (see FUN_00847d20).
static constexpr uint64_t kHedgeMediumHashcode = 0x4d5f454941485f56ull;
static constexpr uint64_t kHedgeOtherHashcode = 0x4f5f454941485f56ull;

void GSPhysics::placeSceneryInstance( const void *pSectorKey,
                                      uint64_t hashcode,
                                      const Matrix4x4f_t &worldTransform,
                                      const Eigen::Vector3f &objectOrigin,
                                      const Eigen::Vector3f &instanceScale ) {
    // FUN_00847d20
    PhysicsLock lock;

    static constexpr size_t kMaxSceneryBodies = 20000;
    if ( hashcode == 0 || sceneryInstanceBodies.size() >= kMaxSceneryBodies ) {
        return;
    }

    if ( pSceneryFactory == nullptr && propFactories.empty() ) {
        if ( pendingSceneryInstances.size() < kMaxSceneryBodies ) {
            pendingSceneryInstances.push_back( { pSectorKey, hashcode,
                                                 worldTransform, objectOrigin,
                                                 instanceScale } );
        }
        return;
    }
    if ( !isPropCollisionType( hashcode ) ) {
        return;
    }

    Eigen::Vector3f shapeScale = instanceScale;
    if ( hashcode == kHedgeMediumHashcode || hashcode == kHedgeOtherHashcode ) {
        std::swap( shapeScale.x(), shapeScale.z() );
    }

    std::vector<std::pair<uint64_t, RigidBody_t>> outBodies;
    placeProp( hashcode, worldTransform, /*bDynamic=*/false, 0.0f, 0.0f,
               outBodies, /*outPrimaryLocalInv=*/nullptr,
               /*bSceneryInstance=*/true, objectOrigin, instanceScale,
               shapeScale );
    for ( const auto &ob : outBodies ) {
        sceneryInstanceBodies.emplace_back( pSectorKey, ob.second );
    }
}

void GSPhysics::removeSceneryInstances( const void *pSectorKey ) {
    if ( pSectorKey == nullptr || pPhysicsSystem == nullptr ) {
        return;
    }

    PhysicsLock lock;

    for ( auto it = sceneryInstanceBodies.begin();
          it != sceneryInstanceBodies.end(); ) {
        if ( it->first == pSectorKey ) {
            destroyRigidBody( it->second );
            it = sceneryInstanceBodies.erase( it );
        } else {
            ++it;
        }
    }

    for ( auto it = pendingSceneryInstances.begin();
          it != pendingSceneryInstances.end(); ) {
        if ( it->pSectorKey == pSectorKey ) {
            it = pendingSceneryInstances.erase( it );
        } else {
            ++it;
        }
    }
}

void GSPhysics::queueSceneryCollisionFactory( const void *pBlob,
                                              uint32_t blobSize ) {
    PhysicsLock lock;

    if ( pBlob == nullptr || blobSize == 0 || !pendingSceneryBlob.empty() ||
         pSceneryFactory != nullptr ) {
        return;
    }
    pendingSceneryBlob.assign( static_cast<const uint8_t *>( pBlob ),
                               static_cast<const uint8_t *>( pBlob ) +
                                   blobSize );
}

bool GSPhysics::isPropCollisionType( uint64_t hashcode ) const {
    if ( hashcode == 0 ) {
        return false;
    }
    std::vector<const PhysFactory *> factories( propFactories.begin(),
                                                propFactories.end() );
    factories.push_back( pSceneryFactory );
    for ( const PhysFactory *pF : factories ) {
        if ( pF == nullptr ) {
            continue;
        }
        HavokFactoryData *pData = pF->getHavokData();
        if ( pData == nullptr ) {
            continue;
        }
        for ( const auto &m : pData->MechaMembers ) {
            if ( m.first == hashcode ) {
                return true;
            }
        }
    }
    return false;
}

void GSPhysics::buildProcessPendingProps() {
    if ( !bJoltInit ) {
        return;
    }

    if ( !pendingPropFactories.empty() ) {
        std::vector<PendingPropFactory> pending;
        pending.swap( pendingPropFactories );
        for ( PendingPropFactory &request : pending ) {
            PhysFactory *pFactory =
                createFromBinary( request.Blob.data(),
                                  static_cast<int32_t>( request.Blob.size() ) );
            if ( pFactory == nullptr ) {
                OTDU_LOG_WARN( "Prop collision: factory parse failed\n" );
                continue;
            }
            propFactories.push_back( pFactory );
            if ( request.ppOwnerSlot != nullptr ) {
                *request.ppOwnerSlot = pFactory;
            }
        }
    }

    if ( propFactories.empty() ) {
        return;
    }

    // Build the full CommonWorld scenery library.
    if ( pSceneryFactory == nullptr && !pendingSceneryBlob.empty() ) {
        pSceneryFactory = createFromBinary(
            pendingSceneryBlob.data(),
            static_cast<int32_t>( pendingSceneryBlob.size() ) );
        pendingSceneryBlob.clear();
        pendingSceneryBlob.shrink_to_fit();

        if ( pSceneryFactory != nullptr ) {
            HavokFactoryData *pD = pSceneryFactory->getHavokData();
            OTDU_LOG_DEBUG(
                "Scenery collision library: %zu mechas, %zu bodies\n",
                pD ? pD->MechaMembers.size() : 0, pD ? pD->Bodies.size() : 0 );

        } else {
            OTDU_LOG_WARN( "Scenery collision library parse failed\n" );
        }
    }

    // Drain scenery instances parked while the factory was still building.
    if ( !pendingSceneryInstances.empty() &&
         ( pSceneryFactory != nullptr || !propFactories.empty() ) ) {
        std::vector<PendingSceneryInstance> pending;
        pending.swap( pendingSceneryInstances );
        for ( const PendingSceneryInstance &p : pending ) {
            placeSceneryInstance( p.pSectorKey, p.hashcode, p.transform,
                                  p.objectOrigin, p.instanceScale );
        }
        OTDU_LOG_DEBUG(
            "Scenery collision: placed %zu deferred instances (%zu bodies)\n",
            pending.size(), sceneryInstanceBodies.size() );
    }
}

void GSPhysics::placeProp(
    uint64_t hashcode,
    const Matrix4x4f_t &worldTransform,
    bool bDynamic,
    float linearDamping,
    float angularDamping,
    std::vector<std::pair<uint64_t, RigidBody_t>> &outBodies,
    Matrix4x4f_t *outPrimaryLocalInv,
    bool bSceneryInstance,
    const Eigen::Vector3f &objectOrigin,
    const Eigen::Vector3f &offsetScale,
    const Eigen::Vector3f &shapeScale ) {
    // FUN_00453690
    if ( pPhysicsSystem == nullptr || hashcode == 0 ) {
        return;
    }

    HavokFactoryData *pData = nullptr;
    {
        std::vector<PhysFactory *> factories( propFactories.begin(),
                                              propFactories.end() );
        factories.push_back( pSceneryFactory );
        for ( PhysFactory *pF : factories ) {
            if ( pF == nullptr ) {
                continue;
            }
            HavokFactoryData *pd = pF->getHavokData();
            if ( pd == nullptr ) {
                continue;
            }
            bool bHas = false;
            for ( const auto &mm : pd->MechaMembers ) {
                if ( mm.first == hashcode ) {
                    bHas = true;
                    break;
                }
            }
            if ( !bHas ) {
                for ( const HavokToJolt::RigidBody &b : pd->Bodies ) {
                    if ( b.Hashcode == hashcode &&
                         b.Shape.GetPtr() != nullptr ) {
                        bHas = true;
                        break;
                    }
                }
            }
            if ( bHas ) {
                pData = pd;
                break;
            }
        }
    }
    if ( pData == nullptr ) {
        return;
    }

    // Resolve the entity's own-ref hashcode to the list of rigid-body hashcodes
    // to place: a Mecha expands to its member bodies; otherwise the hashcode is
    // itself a bare rigid body.
    const std::vector<HavokToJolt::MechaMember> *pMembers = nullptr;
    for ( const auto &mm : pData->MechaMembers ) {
        if ( mm.first == hashcode ) {
            pMembers = &mm.second;
            break;
        }
    }
    HavokToJolt::MechaMember single;
    single.BodyHashcode = hashcode;
    const HavokToJolt::MechaMember *pList =
        pMembers ? pMembers->data() : &single;
    const size_t count = pMembers ? pMembers->size() : 1;

    struct GatheredMember {
        const HavokToJolt::RigidBody *pBody;
        uint64_t RenderKey;
    };
    std::vector<GatheredMember> members;
    members.reserve( count );
    for ( size_t i = 0; i < count; i++ ) {
        for ( const HavokToJolt::RigidBody &b : pData->Bodies ) {
            if ( b.Hashcode == pList[i].BodyHashcode &&
                 b.Shape.GetPtr() != nullptr ) {
                const uint64_t renderKey = pList[i].LogicIDs.empty()
                                               ? b.Hashcode
                                               : pList[i].LogicIDs.front();
                members.push_back( { &b, renderKey } );
                break;
            }
        }
    }
    if ( members.empty() ) {
        return;
    }

    // Matrix4x4f_t row-major -> Jolt column-major, rebased into shifted space.
    JPH::Mat44 world = JPH::Mat44::sIdentity();
    for ( int32_t c = 0; c < 4; c++ ) {
        world.SetColumn3( c, JPH::Vec3( worldTransform( c, 0 ),
                                        worldTransform( c, 1 ),
                                        worldTransform( c, 2 ) ) );
    }
    {
        const Eigen::Vector4f shiftOrigin = GetShiftOrigin();
        world.SetTranslation(
            world.GetTranslation() -
            JPH::Vec3( shiftOrigin.x(), shiftOrigin.y(), shiftOrigin.z() ) );
    }

    const float linDamp = std::min( std::max( linearDamping, 0.0f ), 1.0f );
    const float angDamp = std::min( std::max( angularDamping, 0.05f ), 1.0f );

    lockMutex();
    JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();

    if ( bDynamic ) {
        const uint32_t groupID = gNextPropGroupID++;
        JPH::GroupFilterTable *pFilter = GetPropGroupFilter();

        bool bPrimarySet = false;
        std::vector<JPH::Body *> placed( members.size(), nullptr );
        for ( size_t i = 0; i < members.size(); i++ ) {
            const HavokToJolt::RigidBody *m = members[i].pBody;
            const JPH::Mat44 local =
                JPH::Mat44::sRotationTranslation( m->Rotation, m->Position );
            const JPH::Mat44 placedM = world * local;
            const bool bAnchor = m->bFixedMotion;

            JPH::BodyCreationSettings settings(
                m->Shape, placedM.GetTranslation(),
                placedM.GetQuaternion().Normalized(),
                bAnchor ? JPH::EMotionType::Static : JPH::EMotionType::Dynamic,
                bAnchor ? TDUPhysLayers::NON_MOVING : TDUPhysLayers::MOVING );
            if ( !bAnchor ) {
                settings.mOverrideMassProperties =
                    JPH::EOverrideMassProperties::CalculateInertia;
                // Asset mass (hkMotion 1/mass @+0xc0); fall back to a default
                // if the motion carried none.
                settings.mMassPropertiesOverride.mMass =
                    m->Mass > 0.0f ? m->Mass : kDynamicPropMass;
                settings.mLinearDamping = linDamp;
                settings.mAngularDamping = angDamp;
            }
            settings.mCollisionGroup = JPH::CollisionGroup(
                pFilter, groupID,
                static_cast<uint32_t>( i % kMaxPropMembers ) );
            settings.mUserData = TDUSurface::Generic;
            settings.mFriction = m->Friction;
            settings.mRestitution = m->Restitution;

            JPH::Body *pBody = bodyInterface.CreateBody( settings );
            if ( pBody != nullptr ) {
                bodyInterface.AddBody( pBody->GetID(),
                                       JPH::EActivation::DontActivate );
                placed[i] = pBody;
                outBodies.emplace_back( members[i].RenderKey, pBody );

                if ( !bPrimarySet && outPrimaryLocalInv != nullptr ) {
                    Matrix4x4f_t localRow;
                    for ( int32_t c = 0; c < 4; c++ ) {
                        const JPH::Vec3 v = local.GetColumn3( c );
                        localRow( c, 0 ) = v.GetX();
                        localRow( c, 1 ) = v.GetY();
                        localRow( c, 2 ) = v.GetZ();
                        localRow( c, 3 ) = ( c == 3 ) ? 1.0f : 0.0f;
                    }
                    *outPrimaryLocalInv = localRow.inverse();
                    bPrimarySet = true;
                }
            }
        }

        for ( const HavokToJolt::Constraint &jc : pData->Constraints ) {
            JPH::Body *bA = nullptr;
            JPH::Body *bB = nullptr;
            for ( size_t i = 0; i < members.size(); i++ ) {
                if ( placed[i] == nullptr ) {
                    continue;
                }
                if ( members[i].pBody->Hashcode == jc.BodyAHashcode ) {
                    bA = placed[i];
                }
                if ( members[i].pBody->Hashcode == jc.BodyBHashcode ) {
                    bB = placed[i];
                }
            }
            if ( bA == nullptr || bB == nullptr || bA == bB ) {
                continue;
            }
            if ( bA->GetMotionType() == JPH::EMotionType::Static &&
                 bB->GetMotionType() == JPH::EMotionType::Static ) {
                continue; // nothing to simulate
            }

            JPH::FixedConstraintSettings cs;
            cs.mAutoDetectPoint = true; // weld at the current relative pose
            JPH::TwoBodyConstraint *pC = cs.Create( *bA, *bB );
            if ( pC == nullptr ) {
                continue;
            }
            pPhysicsSystem->AddConstraint( pC );

            PropConstraintRecord rec;
            rec.pConstraint = static_cast<JPH::FixedConstraint *>( pC );
            rec.BodyA = bA->GetID();
            rec.BodyB = bB->GetID();
            rec.Threshold = jc.Threshold;
            rec.bBreakable = jc.bBreakable;
            gPropConstraints.push_back( rec );
        }
    } else {
        // Static prop.
        const JPH::Vec3 base =
            bSceneryInstance ? JPH::Vec3( objectOrigin.x(), objectOrigin.y(),
                                          objectOrigin.z() )
                             : JPH::Vec3::sZero();
        const JPH::Vec3 posScale( offsetScale.x(), offsetScale.y(),
                                  offsetScale.z() );
        const JPH::Vec3 shpScale( shapeScale.x(), shapeScale.y(),
                                  shapeScale.z() );
        const JPH::Vec3 kUnitScale = JPH::Vec3::sReplicate( 1.0f );
        const bool bScalePos =
            bSceneryInstance && !posScale.IsClose( kUnitScale );
        const bool bScaleShape =
            bSceneryInstance && !shpScale.IsClose( kUnitScale );
        for ( const GatheredMember &gm : members ) {
            const HavokToJolt::RigidBody *m = gm.pBody;
            JPH::Vec3 memberPos =
                bSceneryInstance ? ( m->Position - base ) : m->Position;
            JPH::RefConst<JPH::Shape> shape = m->Shape;
            if ( bScalePos ) {
                memberPos = memberPos * posScale;
            }
            if ( bScaleShape ) {
                shape = HavokToJolt::buildScaledShape( *m, shpScale );
            }
            const JPH::Mat44 local =
                JPH::Mat44::sRotationTranslation( m->Rotation, memberPos );
            const JPH::Mat44 placedM = world * local;
            JPH::BodyCreationSettings settings(
                shape, placedM.GetTranslation(),
                placedM.GetQuaternion().Normalized(), JPH::EMotionType::Static,
                TDUPhysLayers::NON_MOVING );
            settings.mUserData = TDUSurface::Generic;
            settings.mFriction = m->Friction;
            settings.mRestitution = m->Restitution;

            JPH::Body *pBody = bodyInterface.CreateBody( settings );
            if ( pBody != nullptr ) {
                bodyInterface.AddBody( pBody->GetID(),
                                       JPH::EActivation::DontActivate );
                outBodies.emplace_back( gm.RenderKey, pBody );
            }
        }
    }
    unlockMutex();
}

void GSPhysics::removePropConstraintsFor( RigidBody_t pBody ) {
    if ( pBody == nullptr || pPhysicsSystem == nullptr ) {
        return;
    }
    const JPH::BodyID id = static_cast<JPH::Body *>( pBody )->GetID();
    lockMutex();
    for ( auto it = gPropConstraints.begin(); it != gPropConstraints.end(); ) {
        if ( it->BodyA == id || it->BodyB == id ) {
            if ( it->pConstraint != nullptr ) {
                pPhysicsSystem->RemoveConstraint( it->pConstraint );
            }
            it = gPropConstraints.erase( it );
        } else {
            ++it;
        }
    }
    unlockMutex();
}

void GSPhysics::moveVehicleParts( const PhysicsVehicle &vehicle,
                                  const JPH::RMat44 &newChassisWorld,
                                  const bool bActivate ) {
    // FUN_00455090 (mode 2)
    const std::vector<PhysicsVehicle::AttachedPart> &parts =
        vehicle.getAttachedParts();
    if ( parts.empty() || pPhysicsSystem == nullptr ) {
        return;
    }

    lockMutex();
    JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();

    std::vector<JPH::BodyID> connected;
    connected.push_back( vehicle.getChassisBody()->GetID() );
    for ( size_t i = 0; i < connected.size(); i++ ) {
        for ( const PropConstraintRecord &record : gPropConstraints ) {
            if ( record.bBroken || record.pConstraint == nullptr ) {
                continue;
            }
            JPH::BodyID other;
            if ( record.BodyA == connected[i] ) {
                other = record.BodyB;
            } else if ( record.BodyB == connected[i] ) {
                other = record.BodyA;
            } else {
                continue;
            }
            if ( std::find( connected.begin(), connected.end(), other ) ==
                 connected.end() ) {
                connected.push_back( other );
            }
        }
    }

    const JPH::RMat44 delta =
        newChassisWorld * bodyInterface.GetWorldTransform( connected.front() )
                              .InversedRotationTranslation();
    for ( const PhysicsVehicle::AttachedPart &part : parts ) {
        const JPH::BodyID id = part.pBody->GetID();
        if ( std::find( connected.begin(), connected.end(), id ) ==
             connected.end() ) {
            continue;
        }
        const JPH::RMat44 m = delta * bodyInterface.GetWorldTransform( id );
        bodyInterface.SetPositionAndRotation(
            id, m.GetTranslation(), m.GetQuaternion().Normalized(),
            ( bActivate && bodyInterface.IsAdded( id ) )
                ? JPH::EActivation::Activate
                : JPH::EActivation::DontActivate );
    }
    unlockMutex();
}

bool GSPhysics::hasBrokenVehiclePart( const PhysicsVehicle &vehicle ) {
    // FUN_0083db70 (inlined)
    const std::vector<PhysicsVehicle::AttachedPart> &parts =
        vehicle.getAttachedParts();
    bool bBroken = false;
    lockMutex();
    for ( const PropConstraintRecord &record : gPropConstraints ) {
        if ( !record.bBreakable || !record.bBroken ) {
            continue;
        }
        for ( const PhysicsVehicle::AttachedPart &part : parts ) {
            if ( record.BodyA == part.pBody->GetID() ||
                 record.BodyB == part.pBody->GetID() ) {
                bBroken = true;
                break;
            }
        }
        if ( bBroken ) {
            break;
        }
    }
    unlockMutex();
    return bBroken;
}

GSPhysics::PropMemberAttachment
GSPhysics::getPropMemberAttachment( RigidBody_t pBody ) {
    // FUN_00454660 + FUN_004546d0
    PropMemberAttachment attachment;
    if ( pBody == nullptr ) {
        return attachment;
    }

    const JPH::BodyID id = static_cast<JPH::Body *>( pBody )->GetID();

    lockMutex();
    for ( const PropConstraintRecord &record : gPropConstraints ) {
        if ( record.BodyA != id && record.BodyB != id ) {
            continue;
        }
        if ( !record.bBreakable ) {
            attachment.NumUnbreakable++;
            continue;
        }
        attachment.NumBreakable++;
        if ( record.bBroken ) {
            attachment.NumBroken++;
        }
    }
    unlockMutex();

    return attachment;
}

bool GSPhysics::getBodyPose( RigidBody_t body,
                             Matrix4x4f_t &outMatrix,
                             bool &outActive ) {
    outActive = false;
    if ( body == nullptr || pPhysicsSystem == nullptr ) {
        return false;
    }
    JPH::Body *pBody = static_cast<JPH::Body *>( body );
    lockMutex();
    const JPH::BodyID id = pBody->GetID();
    JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();
    outActive = bodyInterface.IsActive( id );
    const JPH::Mat44 t = bodyInterface.GetWorldTransform( id );
    unlockMutex();

    const Eigen::Vector4f shiftOrigin = GetShiftOrigin();
    for ( int32_t c = 0; c < 4; c++ ) {
        const JPH::Vec3 v = t.GetColumn3( c );
        outMatrix( c, 0 ) = v.GetX();
        outMatrix( c, 1 ) = v.GetY();
        outMatrix( c, 2 ) = v.GetZ();
        outMatrix( c, 3 ) = ( c == 3 ) ? 1.0f : 0.0f;
    }
    outMatrix( 3, 0 ) += shiftOrigin.x();
    outMatrix( 3, 1 ) += shiftOrigin.y();
    outMatrix( 3, 2 ) += shiftOrigin.z();
    return true;
}

void GSPhysics::physicsStep( float dt, float accTime ) {
    // FUN_0081dfd0
    // Equivalent of one Havok world step + per-vehicle integration pass.
    lockMutex();

    // Clamp runaway delta times (e.g. after a breakpoint / window focus loss).
    if ( dt > kPhysicsDtClampThreshold )
        dt = kPhysicsMaxDt;

    // Apply global time-scale for slow-motion / replay speed control.
    dt *= gPhysicsTimeScale;

    buildProcessPendingProps();

    if ( pPhysicsSystem != nullptr && !bPauseJoltTick && !bFreezeSimulation &&
         dt > 0.0f ) {
        // Original: hkpWorld::stepDelta (Havok) — Jolt equivalent.
        // numCollisionSteps=1 matches the original single-pass Havok
        // broadphase.
        pPhysicsSystem->Update( dt, 1, pTempAllocator, pJobSystem );

        if ( !gPropConstraints.empty() ) {
            JPH::BodyInterface &bi = pPhysicsSystem->GetBodyInterface();
            for ( auto it = gPropConstraints.begin();
                  it != gPropConstraints.end(); ) {
                if ( it->bBreakable && it->pConstraint != nullptr ) {
                    const float impulse =
                        it->pConstraint->GetTotalLambdaPosition().Length();
                    if ( impulse > it->Threshold * kBreakThresholdScale ) {
                        pPhysicsSystem->RemoveConstraint( it->pConstraint );
                        it->pConstraint = nullptr;
                        it->bBroken = true;
                        // Wake the freed pieces so they respond to gravity.
                        bi.ActivateBody( it->BodyA );
                        bi.ActivateBody( it->BodyB );
                    }
                }
                ++it;
            }
        }
    }

    // Tick PhysicsVehicle vehicle dynamics (pushes driver input; the Jolt
    for ( PhysicsVehicle *it = pPhysicsVehiclesBegin; it != pPhysicsVehiclesEnd;
          ++it ) {
        it->tick( dt );
        if ( it->isActive() ) {
            it->publishStepPose();
        }
    }

    if ( gpCamera != nullptr ) {
        gpCamera->updateActiveCamera( dt, accTime );
    }

    updateForceFeedback( dt );

    unlockMutex();
}

static float RemapClamped( const float value,
                           const float inMin,
                           const float inMax,
                           const float outMin,
                           const float outMax ) {
    // FUN_0040b760
    const float remapped =
        ( value - inMin ) * ( outMax - outMin ) / ( inMax - inMin ) + outMin;
    return std::clamp( remapped, std::min( outMin, outMax ),
                       std::max( outMin, outMax ) );
}

static float
EvalFFBCurve( const float value, const float *pInputs, const float *pOutputs ) {
    if ( value < pInputs[1] ) {
        return RemapClamped( value, pInputs[0], pInputs[1], pOutputs[0],
                             pOutputs[1] );
    }
    return RemapClamped( value, pInputs[1], pInputs[2], pOutputs[1],
                         pOutputs[2] );
}

static bool IsForceFeedbackGameMode() {
    if ( gpActiveGameMode == nullptr ) {
        return true;
    }
    const eGameMode gameMode = gpActiveGameMode->getCurrentGameMode();
    return gameMode == GM_FreeRide || gameMode == GM_Mission ||
           gameMode == GM_TestCar;
}

void GSPhysics::updateForceFeedback( float dt ) {
    // FUN_0081cd10
    static constexpr float kDamperAtRest = 0.5f;            // DAT_00fab840
    static constexpr float kDamperSpeedMax = 14.0f;         // DAT_00fab844
    static constexpr float kDamperSpeedMin = 1.0f;          // DAT_00fab848
    static constexpr float kSlipFactorMin = 0.5f;           // DAT_00fab84c
    static constexpr float kArcadeTorqueScale = 1.0f;       // DAT_00fab850
    static constexpr float kSimuTorqueScale = 0.75f;        // DAT_00fab854
    static constexpr float kTorqueDivisorMaxStr = 3000.0f;  // DAT_00fab858
    static constexpr float kTorqueDivisorMinStr = 10000.0f; // DAT_00fab85c
    static constexpr float kTorqueMassScale = 0.0008f;      // DAT_00f41de0
    static constexpr float kSlideSpeedMin = 2.5f;           // DAT_00fab83c
    static constexpr float kSlideSpeedMax = 20.0f;          // DAT_00fab838
    static constexpr float kImpactEnergyMin = 50000.0f;     // DAT_00fab830
    static constexpr float kImpactEnergyMax = 100000.0f;    // DAT_00fab82c
    static constexpr float kImpactDecay = 0.95f;            // DAT_00fab828
    static constexpr float kImpactDecayScale = 1.1111112f;  // DAT_00f41ddc

    static constexpr float kDamperAtSpeed = 0.0f;

    static constexpr float kGraphScale = 10.0f; // FLOAT_00f4166c

    static float sImpactRumble = 0.0f; // DAT_016a6a0c
    static float sGraphTime = 0.0f;    // DAT_010e7c0c

    sGraphTime += dt;

    ForceFeedbackOutput &output = gForceFeedbackOutput;
    output.SecondaryForce = 0.0f;
    output.ConstantForce = 0.0f;
    output.Damper = 0.0f;

    CarState *pCarState = gpScenaric != nullptr
                              ? gpScenaric->getLocalPlayer().getCarState()
                              : nullptr;
    PhysicsVehicle *pVehicle =
        pCarState != nullptr ? pCarState->getPhysicsVehicle() : nullptr;
    if ( pVehicle == nullptr || !pVehicle->isActive() ||
         pVehicle->isPhysicsPaused() || pVehicle->isKeyframed() ) {
        return;
    }

    OTDU_UNIMPLEMENTED_ONCE; // TODO: REPLAY (DAT_00fa97dc == 5 disables FFB)

    float strength = 0.0f;
    if ( !bPauseJoltTick && !bPausePhysicsThread && bForceFeedbackAllowed &&
         IsForceFeedbackGameMode() ) {
        strength = pVehicle->getRumbleMax();
    }

    // Front axle
    const uint32_t numWheels = pVehicle->getNumWheels();
    const PhysicsWheelState &frontLeft = pVehicle->getWheelState( 0 );
    const PhysicsWheelState &frontRight =
        pVehicle->getWheelState( numWheels / 4 );

    const float speed = pVehicle->getChassisVelocity().head<3>().norm();
    const float longitudinalG = pVehicle->getLongitudinalG();
    const float slip =
        ( frontLeft.NormLatClamped + frontRight.NormLatClamped ) * 0.5f;

    const float speedInputs[3] = { gFFBSpeedFactor[1][0], gFFBSpeedFactor[2][0],
                                   gFFBSpeedFactor[3][0] };
    const float speedOutputs[3] = {
        gFFBSpeedFactor[1][1], gFFBSpeedFactor[2][1], gFFBSpeedFactor[3][1] };
    const float speedFactor = EvalFFBCurve( speed, speedInputs, speedOutputs );
    const float loadFactor =
        EvalFFBCurve( longitudinalG, gFFBLoadFromAccel, gFFBLoadFactor );
    const float modeScale =
        gbSimuModeEnabled ? kSimuTorqueScale : kArcadeTorqueScale;
    const float torqueDivisor = RemapClamped(
        strength, 0.0f, 1.0f, kTorqueDivisorMinStr, kTorqueDivisorMaxStr );
    const float slipFactor =
        RemapClamped( std::fabs( slip ), 1.0f, 2.0f, 1.0f, kSlipFactorMin );

    float steeringTorque =
        ( slipFactor *
          ( ( frontLeft.TireForceLat + frontRight.TireForceLat ) * -0.5f ) *
          loadFactor * speedFactor ) /
        ( torqueDivisor * modeScale * pVehicle->getVehicleMass() *
          kTorqueMassScale );
    if ( !bForceFeedbackAllowed || strength == 0.0f ) {
        steeringTorque = 0.0f;
    }
    if ( gpPlayerData->getLocalPlayerData().bFFBInvert ) {
        steeringTorque = 0.0f - steeringTorque;
    }

    if ( ( debugDrawFlags & 0x400 ) != 0 && !gPhysicsGraph.isFrozen() ) {
        const int32_t numCurves = gPhysicsGraph.getNumCurves();
        if ( 0 < numCurves ) {
            const float slipAngleMax = gFFBSlipAngle[2][0];
            const float clampedSlip =
                std::max( -slipAngleMax, std::min( slip, slipAngleMax ) );
            gPhysicsGraph.getCurve( 0 ).addPoint( sGraphTime,
                                                  clampedSlip * kGraphScale );
        }
        if ( 2 < numCurves && !gPhysicsGraph.isFrozen() ) {
            gPhysicsGraph.getCurve( 2 ).addPoint( sGraphTime,
                                                  slipFactor * kGraphScale );
        }
        if ( 3 < numCurves && !gPhysicsGraph.isFrozen() ) {
            gPhysicsGraph.getCurve( 3 ).addPoint( sGraphTime, steeringTorque *
                                                                  kGraphScale );
        }
    }

    const float damper = RemapClamped( speed, kDamperSpeedMin, kDamperSpeedMax,
                                       kDamperAtRest, kDamperAtSpeed );

    if ( gpMngFade != nullptr && gpMngFade->isRenderingLogo() ) {
        strength = 0.0f;
    }

    float slideSpeed = 0.0f;
    if ( gpFlashMessageBox->getNumOpenMsgBox() != 0 ||
         gpFlashMessageBox->getUnknownFlag() ) {
        strength = 0.0f;
    } else if ( strength > 0.0f && !pVehicle->isAirborne() ) {
        slideSpeed = std::fabs( pVehicle->getLateralSpeed() );
    }
    const float slideRumble =
        RemapClamped( slideSpeed, kSlideSpeedMin, kSlideSpeedMax, 0.0f, 1.0f );

    float impactRumble = 0.0f;
    if ( strength > 0.0f ) {
        impactRumble =
            RemapClamped( pVehicle->getMaxCollisionEnergy(), kImpactEnergyMin,
                          kImpactEnergyMax, 0.0f, 1.0f );
        impactRumble = std::max( impactRumble, pVehicle->getLandingImpact() );
        if ( impactRumble == 0.0f ) {
            sImpactRumble = ( impactRumble - sImpactRumble ) *
                                ( 1.0f - kImpactDecay ) * kImpactDecayScale +
                            sImpactRumble;
        } else {
            sImpactRumble = impactRumble;
        }
    } else {
        sImpactRumble = 0.0f;
    }

    if ( gpConfig->pPlayerConfig == nullptr ) {
        return;
    }
    DeviceAction *pSteerAction = pVehicle->isBike()
                                     ? gpConfig->pPlayerConfig->pBikeRight
                                     : gpConfig->pPlayerConfig->pCarRight;
    if ( pSteerAction == nullptr || pSteerAction->NumAddedChannels == 0 ) {
        return;
    }
    InputChannel *pChannel = pSteerAction->pChannel->pChannel;
    Device *pDevice = pChannel != nullptr ? pChannel->pParent : nullptr;
    if ( pDevice == nullptr ) {
        return;
    }

    if ( gpPlayerData->getLocalPlayerData().bUseWheel ) {
        output.Mode = eForceFeedbackMode::FFBM_Wheel;
        output.ConstantForce = steeringTorque;
        output.Damper = damper * strength;
        return;
    }

    if ( pDevice->getNumEffects() != 0 ) {
        output.Mode = eForceFeedbackMode::FFBM_Pad;
        if ( pDevice->getChannelIndex( int32_t( eInputChannelType::ICT_Effect ),
                                       1 ) >= 0 ) {
            output.SecondaryForce = slideRumble * strength;
        }
        if ( pDevice->getChannelIndex( int32_t( eInputChannelType::ICT_Effect ),
                                       0 ) >= 0 ) {
            output.ConstantForce = impactRumble * strength;
        }
        output.latchPadPeaks();
    }
}

void GSPhysics::initializeJolt() {
    // FUN_0081d960
    lockMutex();

    pBpLayerInterface = new TDUBPLayerInterface();
    pObjectVsBPLayerFilter = new TDUObjectVsBroadPhaseLayerFilter();
    pObjectLayerPairFilter = new TDUObjectLayerPairFilter();

    pTempAllocator = new JPH::TempAllocatorImpl( 32 * 1024 * 1024 );
    pJobSystem = new JPH::JobSystemThreadPool(
        JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers,
        ( getenv( "OTDU_PHYSICS_THREADS" ) ? atoi( getenv( "OTDU_PHYSICS_THREADS" ) ) : 3 ) );

    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();
    HavokHeightFieldAgent::Register();

    pPhysicsSystem = new JPH::PhysicsSystem();
    pPhysicsSystem->Init( 65536, 0, 65536, 20480, *pBpLayerInterface,
                          *pObjectVsBPLayerFilter, *pObjectLayerPairFilter );

    JPH::PhysicsSettings physicsSettings = pPhysicsSystem->GetPhysicsSettings();
    physicsSettings.mNumVelocitySteps = 24;
    physicsSettings.mNumPositionSteps = 4;
    physicsSettings.mPointVelocitySleepThreshold = 0.1f;
    pPhysicsSystem->SetPhysicsSettings( physicsSettings );

    pPhysicsSystem->SetGravity( JPH::Vec3( 0.0f, -9.81f, 0.0f ) );

    pContactListener = new PhysicsFXContactListener();
    pPhysicsSystem->SetContactListener( pContactListener );
    pPhysicsSystem->SetSimShapeFilter( &gWheelSphereFilter );
    pPhysicsSystem->SetSimCollideBodyVsBody( &TDUSimCollideBodyVsBody );
    pPhysicsSystem->SetCombineFriction( &CombineFriction );
    pPhysicsSystem->SetCombineRestitution( &CombineRestitution );

    // Create world-reference body (equivalent of the Havok sphere at origin
    // stored in DAT_010e749c; used as a stable anchor in the physics world).
    Eigen::Vector4f origin = Eigen::Vector4f::Zero();
    worldRefBody = createSphere( 2, origin, 0.5f, 1.0f, 1.0f, false );

    broadPhaseOrigin = Eigen::Vector4f::Zero();

    bJoltInit = true;
    unlockMutex();
}

void GSPhysics::setWorldOrigin( const Eigen::Vector4f *pPosition ) {
    // FUN_00843cc0
    if ( gpWorldStreaming != nullptr && gpWorldStreaming->isEnabled() ) {
        return;
    }

    const Eigen::Vector4f shiftDistance = *pPosition - broadPhaseOrigin;
    broadPhaseOrigin += shiftDistance;
}

Eigen::Vector4f GSPhysics::GetShiftOrigin() {
    if ( gpWorldStreaming != nullptr && gpWorldStreaming->isEnabled() ) {
        Eigen::Vector4f origin = gpWorldStreaming->getPosition();
        origin.w() = 0.0f;
        return origin;
    }
    return Eigen::Vector4f::Zero();
}

bool GSPhysics::onWorldStreamed( Eigen::Vector4f *pOffset,
                                 Eigen::Vector4f *pLastPosition,
                                 Eigen::Vector4f *pPosition ) {
    // FUN_00847730
    (void)pLastPosition;
    (void)pPosition;

    if ( pOffset == nullptr || pPhysicsSystem == nullptr ) {
        return true;
    }

    // Content moves the opposite way to the origin.
    const JPH::Vec3 shift( -pOffset->x(), -pOffset->y(), -pOffset->z() );
    if ( shift.IsNearZero() ) {
        return true;
    }

    JPH::BodyIDVector bodyIDs;
    pPhysicsSystem->GetBodies( bodyIDs );

    JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterface();
    for ( const JPH::BodyID id : bodyIDs ) {
        bodyInterface.SetPosition( id, bodyInterface.GetPosition( id ) + shift,
                                   JPH::EActivation::DontActivate );
    }

    if ( pPhysicsVehiclesBegin != nullptr ) {
        const Eigen::Vector3f vehicleShift( shift.GetX(), shift.GetY(),
                                            shift.GetZ() );
        for ( PhysicsVehicle *it = pPhysicsVehiclesBegin;
              it != pPhysicsVehiclesEnd; ++it ) {
            if ( it->isActive() ) {
                it->onWorldShifted( vehicleShift );
            }
        }
    }

    OTDU_LOG_DEBUG( "World origin stepped by (%.1f, %.1f, %.1f) -> "
                    "(%.1f, %.1f, %.1f); moved %zu physics bodies\n",
                    pOffset->x(), pOffset->y(), pOffset->z(), pPosition->x(),
                    pPosition->y(), pPosition->z(), bodyIDs.size() );

    return true;
}
