#include "shared.h"
#include "birds.h"

#include "audio/gs_ambiance.h"
#include "core/color.h"

#include "render/camera/camera_game.h"
#include "render/camera/gs_camera.h"
#include "render/gs_render.h"
#include "render/material.h"
#include "render/render_api.h"
#include "render/render_context.h"
#include "render/render_scene.h"

#include "world/gs_world_streaming.h"
#include "world/weather/gs_weather.h"

#include <cmath>
#include <cstdlib>

static constexpr float kFacingThreshold = 0.04f;       // DAT_00f41980
static constexpr float kVisibilityReset = 0.3f;        // FLOAT_00f416bc
static constexpr float kWingRotTimeScale = 2.0f;       // DAT_00f416b0
static constexpr float kWingRotInitRange = 44.683f;    // DAT_00f42020
static constexpr float kWingRotInitBias = 22.342f;     // DAT_00f4201c
static constexpr float kWingFlapMax = 0.389931f;       // DAT_00f42010
static constexpr float kWingFlapMin = -0.389931f;      // DAT_00f42014
static constexpr float kWingFlapThreshold = 0.139642f; // DAT_00f42018
static constexpr float kWingSpeedScaleA = 0.134407f;   // DAT_00f42028
static constexpr float kWingSpeedScaleB = 2.094395f;   // DAT_00f42024
static constexpr float kWingSpeedBase = 1.047197f;     // DAT_00f417dc
static constexpr float kSpeedRange = 7.44f;            // DAT_00f4202c
static constexpr float kSpeedBase = 6.16f;             // DAT_00f42030
static constexpr float kGroupSpread = 12.0f;           // DAT_00f41850
static constexpr float kGroupSpreadHalf = 6.0f;        // DAT_00f41840
static constexpr float kHeightLookAhead = 50.0f;       // DAT_00f41848
static constexpr uint32_t kBirdVertexCount = 22;
static constexpr uint32_t kBirdPrimitiveCount = 5;
static constexpr float kBirdWingTipU = 0.984375f;   // FLOAT_00f41c64
static constexpr float kBirdWingPanelU = 0.2f;      // DAT_00f41730
static constexpr float kBirdWingSpanInner = -0.16f; // _DAT_00f41c60
static constexpr float kBirdWingSpanOuter = 0.16f;  // DAT_00f41c44

// The detail's billboard material's first parameter carries this GUID.
static constexpr uint64_t kBirdMaterialParamHash = 0x73078865ca37336dULL;

// DAT_00f7f240
// NOTE: Deliberately not constexpr (material instance is bound at
// runtime)
BirdType gBirdTypes[5] = {
    {"FAUCON",  1.0f, 0.7f, 0.1f,   8.0f, 30.0f, 5.0f, 2, 1, nullptr},
    {"PIGEON",  0.4f, 0.3f, 0.065f, 4.5f, 20.0f, 0.0f, 5, 0, nullptr},
    {"MOUETTE", 0.7f, 0.5f, 0.1f,   5.0f, 30.0f, 2.0f, 6, 3, nullptr},
    {"MOINEAU", 0.4f, 0.3f, 0.06f,  4.5f, 20.0f, 0.0f, 6, 0, nullptr},
    {"NOIR",    0.8f, 0.6f, 0.09f,  4.5f, 25.0f, 1.0f, 5, 1, nullptr},
};

static constexpr int32_t kRegionUrban[] = { 1, 3, 1 };         // DAT_00f7f308
static constexpr int32_t kRegionCountry[] = { 0, 4, 3, 4, 3 }; // DAT_00f7f334
static constexpr int32_t kRegionCoast[] = { 2, 1, 2 };         // DAT_00f7f360
static constexpr int32_t kRegionUrbanCount = 3;
static constexpr int32_t kRegionCountryCount = 5;
static constexpr int32_t kRegionCoastCount = 3;

Birds::Birds()
    : TDUDetail(),
      render2DBs{ Render2DB(), Render2DB(), Render2DB(), Render2DB(),
                  Render2DB() },
      render2DM(),
      drawList(),
      materialOT(),
      birds{},
      numBirds( 0 ),
      numBirdsToSpawn( 2 ),
      minVisibleBirds( 3 ),
      minSpawnTime( 0.5f ),
      maxSpawnTime( 0.8f ),
      nextSpawnTime( 0.0f ),
      minDistance( 20.0f ),
      mediumDistance( 65.0f ),
      maxDistance( 110.0f ) {
    // FUN_009d5590
    for ( int32_t i = 0; i < kMaxBirds; i++ ) {
        birds[i].Position = Eigen::Vector4f::Zero();
        birds[i].Direction = Eigen::Vector4f::Zero();
        birds[i].VisibilityTime = -1.0f; // FLOAT_00f41664
    }
    pName = "Birds";
    hash = 0x62697264; // "bird"
}

Birds::~Birds() {
    // FUN_004145f0
}

bool Birds::initialize() {
    static const bool bOtduOff = getenv( "OTDU_NO_BIRDS" ) != nullptr;
    if ( bOtduOff ) {
        return false;
    }
    // FUN_009d6820
    loadBank( "Birds.bnk" );
    if ( !bank.isLoaded() ) {
        return true;
    }

    load2DResources( render2DBs, &render2DM, 1, kNumMaterials );

    DrawStreams streams;
    streams.Diffuse = 1;
    streams.UVMap = 1;
    streams.bIndex16 = 1;
    if ( !drawList.initialize( kMaxBirds * kBirdPrimitiveCount,
                               kMaxBirds * kBirdVertexCount,
                               kMaxBirds * kBirdVertexCount, false, &streams,
                               "BirdsRList", true ) ) {
        return true;
    }

    for ( int32_t i = 0; i < kNumMaterials; i++ ) {
        BirdType &type = gBirdTypes[i];
        Material *pMat = render2DM.getMaterial( type.pMaterialName );
        type.pMaterial = pMat;
        if ( pMat == nullptr ) {
            continue;
        }
        pMat->AlphaTest = CF_Never;
        if ( pMat->NumParams == 0 ) {
            continue;
        }

        MaterialParameter *pParam = pMat->getParameterByIndex( 0 );
        if ( pParam->Hashcode == kBirdMaterialParamHash ) {
            pParam->Flags = MaterialParameterFlags{};
            pParam->Flags.NumUVMaps = 1;
            pParam->Flags.NumTextures = 1;
            pParam->Flags.bUsingNormals = 0;
            pParam->Flags.NumPrelight = 1;
            gpRenderContext->retrieveShadersForMaterial( pMat, 0 );
        }

        MaterialLayer *pLayer = pParam->getLayer( 0 );
        if ( pLayer != nullptr ) {
            pLayer->pLayerTextures[0].SamplerAddress[0] = MTM_Wrap;
            pLayer->pLayerTextures[0].SamplerAddress[1] = MTM_Wrap;
        }
    }

    nextSpawnTime = 0.0f;
    numBirds = 0;
    bInitialized = true;
    return true;
}

void Birds::terminate() {
    // FUN_009d4bc0
    bInitialized = false;
    if ( drawList.isInitialized() ) {
        drawList.destroy();
    }
    unload2DResources();
    if ( bank.isLoaded() ) {
        bank.unload();
    }
}

void Birds::draw() {
    static const bool bOtduOff = getenv( "OTDU_NO_BIRDS" ) != nullptr;
    if ( bOtduOff ) {
        return;
    }
    // FUN_009d54e0
    if ( !drawList.isInitialized() || drawList.getNumPrimitives() == 0 ) {
        return;
    }
    gpRender->getRenderPass<eRenderPass::RP_Near>()
        .pScene->enqueueDynamicDrawList( &drawList );
}

void Birds::tick( float deltaTime ) {
    static const bool bOtduOff = getenv( "OTDU_NO_BIRDS" ) != nullptr;
    if ( bOtduOff ) {
        return;
    }
    // FUN_009d6640
    Eigen::Vector4f shift = Eigen::Vector4f::Zero();
    if ( gpWorldStreaming != nullptr && gpWorldStreaming->isEnabled() ) {
        shift = gpWorldStreaming->getPosition();
    }
    shift *= -1.0f; // FLOAT_00f41664

    Matrix4x4f_t transform = Matrix4x4f_t::Identity();
    transform( 3, 0 ) = shift.x();
    transform( 3, 1 ) = shift.y();
    transform( 3, 2 ) = shift.z();

    drawList.reset();
    drawList.setActiveTransformMatrix( transform );

    updateBirds( deltaTime );

    if ( gpWeather != nullptr && gpWeather->isConfigDirty() ) {
        gpWeather->updateActiveConfig();
    }

    const ColorRGBA diffuseColor = ComputeWeatherSunTintColor();

    // Emit one batch per distinct species material.
    for ( int32_t i = 0; i < kNumMaterials; i++ ) {
        MaterialEntry &entry = materialOT.pEntries[i];
        if ( entry.NumUsers == 0 ) {
            continue;
        }
        drawList.setActiveMaterial( entry.pMaterial );
        for ( MaterialUser *pUser = entry.pFirst; pUser != nullptr;
              pUser = pUser->pNext ) {
            appendBirdGeometry( *static_cast<Bird *>( pUser ), diffuseColor );
        }
    }
}

void Birds::updateBirds( float deltaTime ) {
    // FUN_009d5740
    if ( gpCamera == nullptr ) {
        return;
    }

    OTDU_UNIMPLEMENTED_ONCE; // TODO: REPLAY (will use frame dt in the meantime)
    const float scaledDt = deltaTime;

    nextSpawnTime -= scaledDt;
    int32_t spawnCount;
    if ( nextSpawnTime <= 0.0f || numBirds < minVisibleBirds ) {
        spawnCount = numBirdsToSpawn;
        nextSpawnTime =
            DetailRandom() * ( maxSpawnTime - minSpawnTime ) + minSpawnTime;
    } else {
        spawnCount = 0;
    }

    const Matrix4x4f_t &camMat = gpCamera->getDefaultCamera()->getCamToWorld();
    const Eigen::Vector3f camPos( camMat( 3, 0 ), camMat( 3, 1 ),
                                  camMat( 3, 2 ) );
    const Eigen::Vector3f camFwd( -camMat( 2, 0 ), -camMat( 2, 1 ),
                                  -camMat( 2, 2 ) );

    // Pick the species table for the currently active ambiance region.
    const int32_t *pRegionTypes = nullptr;
    int32_t regionCount = 0;
    if ( spawnCount != 0 ) {
        if ( isAmbianceRegionActive( 2 ) || isAmbianceRegionActive( 1 ) ||
             isAmbianceRegionActive( 6 ) || isAmbianceRegionActive( 0 ) ) {
            pRegionTypes = kRegionUrban;
            regionCount = kRegionUrbanCount;
        } else if ( isAmbianceRegionActive( 3 ) ||
                    isAmbianceRegionActive( 5 ) ) {
            pRegionTypes = kRegionCountry;
            regionCount = kRegionCountryCount;
        } else if ( isAmbianceRegionActive( 4 ) ) {
            pRegionTypes = kRegionCoast;
            regionCount = kRegionCoastCount;
        } else {
            spawnCount = 0;
        }
    }

    int32_t groupRemaining = 0;
    Bird *pGroupLeader = nullptr;
    numBirds = 0;

    for ( int32_t i = 0; i < kMaxBirds; i++ ) {
        Bird &bird = birds[i];

        // Retire visible birds that drift too far or fall behind the camera.
        if ( bird.VisibilityTime >= 0.0f ) {
            const Eigen::Vector3f delta = bird.Position.head<3>() - camPos;
            const float dist = delta.norm();
            if ( dist > minDistance ) {
                const Eigen::Vector3f dir = delta * ( 1.0f / dist );
                const float facing = camFwd.dot( dir );
                if ( ( facing >= kFacingThreshold && dist >= maxDistance ) ||
                     facing < kFacingThreshold ) {
                    bird.VisibilityTime -= deltaTime;
                    if ( bird.VisibilityTime < 0.0f &&
                         bird.pEntry != nullptr ) {
                        unregisterMaterialUser( &bird );
                    }
                }
            }
        }

        // Respawn dead slots (negative, non-zero visibility) using the spawn
        // budget, or top up an in-progress group.
        if ( bird.VisibilityTime <= 0.0f && bird.VisibilityTime != 0.0f ) {
            bool bSpawned = false;
            if ( spawnCount < 1 ) {
                if ( groupRemaining >= 1 ) {
                    spawnGroupMember( bird, *pGroupLeader );
                    groupRemaining--;
                    bSpawned = true;
                }
            } else if ( groupRemaining > 0 ) {
                spawnGroupMember( bird, *pGroupLeader );
                groupRemaining--;
                bSpawned = true;
            } else {
                computeSpawnPosition( minDistance, maxDistance, mediumDistance,
                                      bird );

                Eigen::Vector4f testPos = bird.Position;
                testPos.w() = 1.0f;
                float groundHeight;
                if ( !SampleDetailGroundHeight( testPos, groundHeight ) ) {
                    groundHeight = camPos.y();
                    bird.pType = &gBirdTypes[2]; // MOUETTE fallback
                } else {
                    const int32_t idx = static_cast<int32_t>(
                        DetailRandom() *
                        static_cast<float>( regionCount - 1 ) );
                    bird.pType = &gBirdTypes[pRegionTypes[idx]];
                }

                const float maxHeight =
                    bird.pType->MaxHeightOffset + groundHeight;
                const float minHeight =
                    bird.pType->MinHeightOffset + groundHeight;
                bird.Position.y() =
                    DetailRandom() * ( maxHeight - minHeight ) + minHeight;

                const float speed = DetailRandom() * kSpeedRange + kSpeedBase;
                bird.Speed = speed;
                bird.WingRotationSpeed = ( speed - kSpeedBase ) *
                                             kWingSpeedScaleA *
                                             kWingSpeedScaleB +
                                         kWingSpeedBase;

                const int32_t maxInGroup = bird.pType->MaxInGroup;
                if ( maxInGroup < 2 ) {
                    groupRemaining = 0;
                } else {
                    groupRemaining = static_cast<int32_t>(
                        DetailRandom() * static_cast<float>( maxInGroup - 1 ) );
                    pGroupLeader = &bird;
                }
                spawnCount--;
                bSpawned = true;
            }

            if ( bSpawned ) {
                BirdType *pType = bird.pType;
                bird.VisibilityTime = kVisibilityReset;
                bird.WingRotationTime = DetailRandom() *
                                        pType->WingRotationTime *
                                        kWingRotTimeScale;
                bird.NumWingRotation = 0;
                bird.NextHeightTest = 0.0f;
                float wingRotation = kWingFlapThreshold;
                if ( pType->NumWingRotation == 0 ) {
                    wingRotation =
                        DetailRandom() * kWingRotInitRange - kWingRotInitBias;
                }
                bird.WingRotation = wingRotation;
                materialOT.registerUser( &bird, pType->pMaterial );
            }
        }

        // Integrate motion and animate the wings for live birds.
        if ( bird.VisibilityTime >= 0.0f ) {
            const float displacement = bird.Speed * scaledDt;
            bird.Position.head<3>() += bird.Direction.head<3>() * displacement;
            bird.Position.w() = 0.0f;
            bird.NextHeightTest -= displacement;

            if ( bird.NextHeightTest <= 0.0f ) {
                Eigen::Vector4f ahead = Eigen::Vector4f::Zero();
                ahead.head<3>() = bird.Position.head<3>() +
                                  bird.Direction.head<3>() * kHeightLookAhead;
                ahead.w() = 1.0f;
                float groundHeight;
                if ( SampleDetailGroundHeight( ahead, groundHeight ) ) {
                    groundHeight += bird.pType->MinHeightOffset;
                    if ( ahead.y() < groundHeight ) {
                        ahead.y() = groundHeight;
                        const Eigen::Vector3f newDir =
                            ahead.head<3>() - bird.Position.head<3>();
                        bird.Direction.head<3>() = newDir.normalized();
                    }
                }
                bird.NextHeightTest = kHeightLookAhead;
            }

            if ( bird.NumWingRotation >= 0 ) {
                BirdType *pType = bird.pType;
                if ( bird.WingRotationTime < pType->WingRotationTime ) {
                    bird.WingRotationTime += scaledDt;
                } else {
                    const float previous = bird.WingRotation;
                    const float current =
                        scaledDt * bird.WingRotationSpeed + previous;
                    bird.WingRotation = current;
                    if ( current >= kWingFlapMin ) {
                        if ( current <= kWingFlapMax ) {
                            if ( pType->NumWingRotation > 0 &&
                                 current >= kWingFlapThreshold &&
                                 previous < kWingFlapThreshold ) {
                                bird.NumWingRotation++;
                                if ( bird.NumWingRotation >=
                                     pType->NumWingRotation ) {
                                    bird.WingRotationTime = 0.0f;
                                    bird.NumWingRotation = 0;
                                }
                            }
                        } else {
                            bird.WingRotation = kWingFlapMax;
                            bird.WingRotationSpeed = -bird.WingRotationSpeed;
                        }
                    } else {
                        bird.WingRotation = kWingFlapMin;
                        bird.WingRotationSpeed = -bird.WingRotationSpeed;
                    }
                }
            }

            numBirds++;
        }
    }
}

void Birds::spawnGroupMember( Bird &bird, const Bird &leader ) {
    // FUN_009d5740
    bird.pType = leader.pType;
    bird.Direction = leader.Direction;
    bird.Speed = leader.Speed;
    bird.WingRotationSpeed = leader.WingRotationSpeed;
    const float offsetY = DetailRandom() - 0.5f; // FLOAT_00f41674
    const float offsetX = DetailRandom() * kGroupSpread - kGroupSpreadHalf;
    const float offsetZ = DetailRandom() * kGroupSpread - kGroupSpreadHalf;
    bird.Position.x() = offsetX + leader.Position.x();
    bird.Position.y() = offsetY + leader.Position.y();
    bird.Position.z() = offsetZ + leader.Position.z();
    bird.Position.w() = 0.0f;
}

bool Birds::isAmbianceRegionActive( int32_t regionId ) {
    // FUN_00901c10
    return gpAmbiance != nullptr &&
           gpAmbiance->isAmbianceRegionActive( regionId );
}

void Birds::computeSpawnPosition( float minDistance,
                                  float maxDistance,
                                  float mediumDistance,
                                  Bird &bird ) {
    // FUN_008a7700
    ComputeDetailSpawnPosition( minDistance, maxDistance, mediumDistance,
                                bird.Position, bird.Direction );
}

void Birds::appendBirdGeometry( const Bird &bird,
                                const ColorRGBA &diffuseColor ) {
    // FUN_009d4c30
    const BirdType *pType = bird.pType;
    const float cosFlap = std::cos( bird.WingRotation );
    const float sinFlap = std::sin( bird.WingRotation );

    const Eigen::Vector3f position = bird.Position.head<3>();
    const Eigen::Vector3f direction = bird.Direction.head<3>();

    const Eigen::Vector3f along( direction.x() * pType->Length, 0.0f,
                                 direction.z() * pType->Length );
    const float wingScale = cosFlap * pType->Width;
    const Eigen::Vector3f span( direction.z() * wingScale, 0.0f,
                                -direction.x() * wingScale );
    const Eigen::Vector3f lift( 0.0f, sinFlap, 0.0f );
    const float height = pType->Height;

    auto pushVertex = [&]( const Eigen::Vector3f &p, float u, float v ) {
        drawList.setTexCoords( 0, Eigen::Vector2f( u, v ) );
        drawList.pushVertex( p.x(), p.y(), p.z() );
    };

    // Body: both wings as one flat strip, dihedral given by the flap lift.
    if ( drawList.beginPrimitive( PT_TriangleStrip, 6 ) ) {
        drawList.setDiffuse( 0, diffuseColor );
        pushVertex( position - along - span + lift, kBirdWingTipU, 1.0f );
        pushVertex( position + along - span + lift, kBirdWingTipU, 0.0f );
        pushVertex( position - along, 0.0f, 1.0f );
        pushVertex( position + along, 0.0f, 0.0f );
        pushVertex( position - along + span + lift, kBirdWingTipU, 1.0f );
        pushVertex( position + along + span + lift, kBirdWingTipU, 0.0f );
        drawList.commitPrimitive();
    }

    // Four wing panels: top/bottom surfaces at the two spanwise scales.
    static constexpr float kPanelSpan[4] = {
        kBirdWingSpanInner, kBirdWingSpanOuter, kBirdWingSpanInner,
        kBirdWingSpanOuter };
    static constexpr float kPanelHeight[4] = { 1.0f, 1.0f, -1.0f, -1.0f };

    for ( int32_t p = 0; p < 4; p++ ) {
        const Eigen::Vector3f tip = position + span * kPanelSpan[p];
        const Eigen::Vector3f apex( position.x(),
                                    position.y() + height * kPanelHeight[p],
                                    position.z() );
        if ( drawList.beginPrimitive( PT_TriangleStrip, 4 ) ) {
            drawList.setDiffuse( 0, diffuseColor );
            pushVertex( position - along, 0.0f, 1.0f );
            pushVertex( tip, kBirdWingPanelU, 0.5f );
            pushVertex( apex, 0.0f, 0.5f );
            pushVertex( position + along, 0.0f, 0.0f );
            drawList.commitPrimitive();
        }
    }
}
