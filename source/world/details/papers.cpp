#include "shared.h"
#include "papers.h"

#include "audio/gs_ambiance.h"
#include "physics/gs_cloth_system.h"
#include "game/vehicle/car_state.h"
#include "game/vehicle/gs_car_state.h"
#include "game/vehicle/vehicle_base.h"

#include "render/2dm.h"
#include "render/camera/camera_game.h"
#include "render/camera/gs_camera.h"
#include "render/particles/gs_particles.h"

#include "world/weather/gs_weather.h"
#include "world/weather/weather_config.h"

static PaperDesc gPaperDescs[Papers::kNumPapers] = {
    // DAT_00f7f3b8
    {"J_01", 3, 2, 0.95f, 0.75f, nullptr},
    {"J_02", 3, 2, 0.95f, 0.75f, nullptr},
    {"J_04", 3, 2, 0.90f, 0.70f, nullptr},
    {"J_05", 2, 3, 0.60f, 0.90f, nullptr},
    {"J_06", 2, 3, 0.60f, 0.90f, nullptr},
    {"J_07", 2, 3, 0.40f, 0.70f, nullptr},
    {"J_08", 2, 3, 0.40f, 0.70f, nullptr},
    {"J_09", 2, 3, 0.40f, 0.70f, nullptr},
};

static constexpr int32_t kPaperNumNeighbors = 4;
static constexpr int32_t kPaperSolverIterations = 2;
static constexpr float kPaperLifetime = 0.5f;       // FLOAT_00f41674
static constexpr float kPaperHeightBias = 1.0f;     // FLOAT_00f4165c
static constexpr float kPaperHeightScale = 0.5f;    // FLOAT_00f41674
static constexpr float kPaperWindLift = 0.5f;       // FLOAT_00f41674
static constexpr float kLeafHeightRange = 1.5f;     // DAT_00f416b8
static constexpr float kLeafHeightBias = 0.5f;      // FLOAT_00f41674
static constexpr float kLeafScale = 1.3f;           // DAT_00f41dfc
static constexpr float kWakeMinDistanceSq = 0.001f; // FLOAT_00f4167c
static constexpr float kWakeMaxDistanceSq = 25.0f;  // DAT_00f41834
static constexpr float kWakeLift = 0.5f;            // FLOAT_00f41674
static constexpr float kWakeSpeedToKmh = 3.6f;      // DAT_00f41768
static constexpr float kWakeSlowSpeed = 20.0f;      // DAT_00f03ee0
static constexpr float kWakeFastSpeed = 40.0f;      // DAT_00f41b70
static constexpr float kWakeSlowStrength = 0.5f;    // DAT_00f416dc
static constexpr float kWakeMediumStrength = 1.5f;  // DAT_00f416b8
static constexpr float kWakeFastStrength = 2.5f;    // DAT_00f416c0

static Matrix4x4f_t ComputeMatrixFromTarget( const Eigen::Vector4f &position,
                                             const Eigen::Vector4f &target ) {
    // FUN_00504580
    static const Eigen::Vector3f kWorldUp( 0.0f, 1.0f, 0.0f );

    const Eigen::Vector3f forward =
        ( target.head<3>() - position.head<3>() ).normalized();
    const Eigen::Vector3f right = kWorldUp.cross( forward ).normalized();
    const Eigen::Vector3f up = forward.cross( right );

    Matrix4x4f_t result;
    result.row( 0 ) << right.x(), right.y(), right.z(), 0.0f;
    result.row( 1 ) << up.x(), up.y(), up.z(), 0.0f;
    result.row( 2 ) << forward.x(), forward.y(), forward.z(), 0.0f;
    result.row( 3 ) << position.x(), position.y(), position.z(), 1.0f;
    return result;
}

Papers::Papers()
    : TDUDetail(),
      render2DBs{ Render2DB(), Render2DB(), Render2DB(), Render2DB(),
                  Render2DB(), Render2DB(), Render2DB(), Render2DB() },
      render2DM(),
      bTestAmbiance( true ),
      papers{},
      numPapersToSpawn( 2 ),
      paperWindFactor( 0.06f ), // DAT_00f41e08
      paperSpawnTime( 0.9f ),   // DAT_00f416a0
      paperNextSpawnTime( 0.0f ),
      paperMinDistance( 20.0f ), // DAT_00f03ee0
      paperMedDistance( 50.0f ), // DAT_00f41848
      paperMaxDistance( 90.0f ), // FLOAT_00f4180c
      leftMinDistance( 20.0f ),  // DAT_00f03ee0
      leafMaxDistance( 60.0f ),  // FLOAT_00f41744
      pLeafParticle( nullptr ) {
    // FUN_008a69b0
    for ( int32_t i = 0; i < kNumPapers; i++ ) {
        papers[i].lifetime = -1.0f; // FLOAT_00f41664
        papers[i].pDesc = nullptr;
        papers[i].pCloth = nullptr;
    }
    pName = "Papers";
    hash = 0x70617072; // "papr"
}

Papers::~Papers() {
    // FUN_00414740
}

bool Papers::initialize() {
    static const bool bOtduOff = getenv( "OTDU_NO_PAPERS" ) != nullptr;
    if ( bOtduOff ) {
        return false;
    }
    // FUN_008a74a0
    loadBank( "Papers.bnk" );
    if ( !bank.isLoaded() ) {
        return true;
    }

    load2DResources( render2DBs, &render2DM, 1, kNumPapers );

    for ( int32_t i = 0; i < kNumPapers; i++ ) {
        gPaperDescs[i].pMaterial =
            render2DM.getMaterial( gPaperDescs[i].pName );
        papers[i].pDesc = &gPaperDescs[i];
    }

    paperNextSpawnTime = 0.0f;
    bInitialized = true;
    return true;
}

void Papers::tick( float deltaTime ) {
    static const bool bOtduOff = getenv( "OTDU_NO_PAPERS" ) != nullptr;
    if ( bOtduOff ) {
        return;
    }
    // FUN_008a7480
    updatePapers( deltaTime );
    updateLeaf();
}

void Papers::draw() {
    static const bool bOtduOff = getenv( "OTDU_NO_PAPERS" ) != nullptr;
    if ( bOtduOff ) {
        return;
    }
    // FUN_008a63c0
    // NOP
}

void Papers::terminate() {
    // FUN_008a6410
    bInitialized = false;
    releasePapers();
    unload2DResources();
    if ( bank.isLoaded() ) {
        bank.unload();
    }
}

void Papers::setEnabled( bool bEnable ) {
    // FUN_008a63d0
    bEnabled = bEnable;
    if ( !bEnable && pLeafParticle != nullptr ) {
        // Release the flying-leaf particle instance back to GSParticles.
        pLeafParticle->State = eParticleState::PS_Dead;
        pLeafParticle->stop();
        pLeafParticle = nullptr;
    }
}

void Papers::updatePapers( float deltaTime ) {
    // FUN_008a7070
    if ( gpCamera == nullptr ) {
        return;
    }

    if ( gpWeather != nullptr && gpWeather->isConfigDirty() ) {
        gpWeather->updateActiveConfig();
    }

    OTDU_UNIMPLEMENTED_ONCE; // TODO: REPLAY (will use frame dt in the meantime)
    const float scaledDt = deltaTime;

    const Matrix4x4f_t &camToWorld =
        gpCamera->getDefaultCamera()->getCamToWorld();
    const Eigen::Vector4f cameraForward = -camToWorld.row( 2 ).transpose();
    const Eigen::Vector4f cameraPosition = camToWorld.row( 3 ).transpose();

    Eigen::Vector4f wind = Eigen::Vector4f::Zero();
    if ( gpWeather != nullptr ) {
        const WeatherConfig &weather =
            gpWeather->getActiveRandomizedWeatherConfig();
        wind = weather.getWindDirection();
        wind.y() = kPaperWindLift;
        wind.head<3>() *= paperWindFactor * weather.getWindSpeed();
    }

    paperNextSpawnTime -= scaledDt;
    int32_t spawnBudget = 0;
    if ( paperNextSpawnTime <= 0.0f ) {
        if ( !bTestAmbiance || isAmbianceRegionActive( 6 ) ) {
            spawnBudget = numPapersToSpawn;
        }
        paperNextSpawnTime = paperSpawnTime;
    }

    for ( int32_t i = 0; i < kNumPapers; i++ ) {
        Paper &paper = papers[i];
        ClothInstance *pCloth = static_cast<ClothInstance *>( paper.pCloth );

        if ( paper.lifetime >= 0.0f && pCloth != nullptr &&
             pCloth->bVisible == 0 ) {
            bool bRelease = ( pCloth->State == 0 );
            if ( bRelease ) {
                paper.lifetime = -1.0f; // FLOAT_00f41664
            } else {
                paper.lifetime -= scaledDt;
                bRelease = ( paper.lifetime < 0.0f );
            }

            if ( bRelease ) {
                pCloth->State = 0;
                pCloth->bVisible = 0;
                pCloth->pMaterial = nullptr;
                paper.pCloth = nullptr;
                pCloth = nullptr;
            }
        }

        if ( paper.lifetime < 0.0f && spawnBudget > 0 &&
             gpClothSystem != nullptr && paper.pDesc != nullptr ) {
            pCloth = gpClothSystem->allocateCloth();
            paper.pCloth = pCloth;
            if ( pCloth != nullptr ) {
                paper.lifetime = kPaperLifetime;

                Eigen::Vector4f spawnPosition;
                Eigen::Vector4f spawnDirection;
                ComputeDetailSpawnPosition( paperMinDistance, paperMaxDistance,
                                            paperMedDistance, spawnPosition,
                                            spawnDirection );

                Eigen::Vector4f groundProbe = spawnPosition;
                groundProbe.w() = 1.0f;
                float groundHeight;
                if ( !SampleDetailGroundHeight( groundProbe, groundHeight ) ) {
                    groundHeight = cameraPosition.y();
                }
                spawnPosition.y() =
                    ( DetailRandom() + kPaperHeightBias ) * kPaperHeightScale +
                    groundHeight;

                const Matrix4x4f_t transform = ComputeMatrixFromTarget(
                    spawnPosition, spawnPosition + cameraForward );

                const PaperDesc &desc = *paper.pDesc;
                pCloth->setup( desc.GridWidth, desc.GridHeight,
                               desc.PhysicalWidth, desc.PhysicalHeight,
                               transform, desc.pMaterial, kPaperNumNeighbors );
                pCloth->SolverIterations = kPaperSolverIterations;
                pCloth->bGroundCollision = 1;

                spawnBudget--;
            }
        }

        if ( paper.lifetime >= 0.0f ) {
            applyWind( paper, wind );
        }
    }
}

void Papers::applyWind( Paper &paper, const Eigen::Vector4f &wind ) {
    // FUN_008a6b90
    ClothInstance *pCloth = static_cast<ClothInstance *>( paper.pCloth );
    if ( pCloth == nullptr ) {
        return;
    }

    const Eigen::Vector4f sheetPosition = pCloth->BoundsCenter + pCloth->Origin;

    Eigen::Vector4f wake = Eigen::Vector4f::Zero();
    if ( gpCarState != nullptr ) {
        CarState *pCarStates = gpCarState->getCarStates();
        for ( uint32_t i = 0; i < kMaxNumVehicles; i++ ) {
            CarState &carState = pCarStates[i];
            if ( !carState.isInstanceAlive() ) {
                continue;
            }

            const Eigen::Vector4f delta =
                sheetPosition - carState.getWorldPosition();
            const float distanceSq = delta.head<3>().squaredNorm();
            if ( distanceSq < kWakeMinDistanceSq ||
                 distanceSq > kWakeMaxDistanceSq ) {
                continue;
            }

            wake =
                Eigen::Vector4f( delta.x(), std::fabs( delta.y() ) + kWakeLift,
                                 delta.z(), 0.0f );
            wake.normalize();

            const float speedKmh =
                carState.getBody()->SpeedInKmh * kWakeSpeedToKmh;
            float strength = kWakeSlowStrength;
            if ( speedKmh >= kWakeSlowSpeed ) {
                strength = ( speedKmh >= kWakeFastSpeed ) ? kWakeFastStrength
                                                          : kWakeMediumStrength;
            }
            wake.head<3>() *= strength;
            break;
        }
    }

    pCloth->Acceleration = Eigen::Vector4f(
        wind.x() + wake.x(), wind.y() + wake.y(), wind.z() + wake.z(), 0.0f );
}

void Papers::updateLeaf() {
    // FUN_008a6e50
    if ( gpCamera == nullptr ) {
        return;
    }

    const Eigen::Vector4f cameraPosition =
        gpCamera->getDefaultCamera()->getCamToWorld().row( 3 ).transpose();

    const bool bAmbianceActive =
        !bTestAmbiance || isAmbianceRegionActive( 3 ) ||
        isAmbianceRegionActive( 5 ) || isAmbianceRegionActive( 4 ) ||
        isAmbianceRegionActive( 0 );

    if ( !bAmbianceActive ) {
        if ( pLeafParticle != nullptr ) {
            pLeafParticle->State = eParticleState::PS_Killed;
            pLeafParticle->stop();
            pLeafParticle = nullptr;
        }
        return;
    }

    if ( pLeafParticle == nullptr ||
         pLeafParticle->State != eParticleState::PS_Alive ) {
        pLeafParticle = ( gpParticles != nullptr )
                            ? gpParticles->activateParticles(
                                  eParticleType::PT_Leaf2, true )
                            : nullptr;
    }

    if ( pLeafParticle == nullptr ) {
        return;
    }

    Eigen::Vector4f leafPosition;
    Eigen::Vector4f leafDirection;
    ComputeDetailSpawnPosition( leftMinDistance, leafMaxDistance,
                                leftMinDistance, leafPosition, leafDirection );

    Eigen::Vector4f groundProbe = leafPosition;
    groundProbe.w() = 1.0f;
    float groundHeight;
    if ( !SampleDetailGroundHeight( groundProbe, groundHeight ) ) {
        groundHeight = cameraPosition.y();
    }
    leafPosition.y() =
        DetailRandom() * kLeafHeightRange + kLeafHeightBias + groundHeight;

    Matrix4x4f_t transform = Matrix4x4f_t::Identity();
    transform.row( 3 ) << leafPosition.x(), leafPosition.y(), leafPosition.z(),
        1.0f;

    pLeafParticle->Matrix = transform;
    pLeafParticle->Scale = kLeafScale;
}

void Papers::releasePapers() {
    // FUN_008a6410
    for ( int32_t i = 0; i < kNumPapers; i++ ) {
        Paper &paper = papers[i];
        paper.lifetime = -1.0f; // FLOAT_00f41664
        if ( paper.pCloth != nullptr ) {
            ClothInstance *pCloth =
                static_cast<ClothInstance *>( paper.pCloth );
            pCloth->State = 0;
            pCloth->bVisible = 0;
            pCloth->pMaterial = nullptr;
            paper.pCloth = nullptr;
        }
    }
}

bool Papers::isAmbianceRegionActive( int32_t regionId ) {
    // FUN_00901c10
    return gpAmbiance != nullptr &&
           gpAmbiance->isAmbianceRegionActive( regionId );
}
