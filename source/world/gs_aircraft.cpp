#include "shared.h"
#include "gs_aircraft.h"

#include "audio/gs_ambiance.h"
#include "audio/gs_audio.h"
#include "audio/implementation/audio_dispatcher_playback.h"
#include "core/maths_helper.h"
#include "core/random/bcpl.h"
#include "render/camera/gs_camera.h"

#include "game/cinematic/mng_cinematics.h"
#include "game/scenaric/gs_scenaric.h"
#include "game/scenaric/scenaric_bank.h"
#include "game/scenaric/entity/base_entity.h"
#include "game/scenaric/entity/entity_aircraft.h"
#include "game/scenaric/entity/entity_aircraft_path.h"
#include "game/scenaric/entity/entity_cinematic.h"
#include "game/scenaric/entity/entity_factory.h"
#include "world/gs_world.h"
#include "world/gs_world_streaming.h"
#include "render/instance.h"
#include "render/material.h"
#include "render/serialized_skinned_model.h"

static constexpr uint16_t kAircraftMapViewOT = 0x26;
static constexpr uint16_t kAircraftDefaultOT = 0x8;
static constexpr uint64_t kAircraftMapViewMaterialHashcode = 0x183157b81c6cff57;

GSAircraft *gpAircraft = nullptr; // DAT_010e742c

static constexpr float kCurveHandleFactor = 1.0f / 3.0f; // DAT_00f41764
static constexpr float kArcLengthStep = 0.1f;

static constexpr float kSpeedRateOfChange = 8.0f; // DAT_00f41ac4 / DAT_00f420d8

// Pitch profile, keyed on the distance left before the runway
static constexpr float kPitchLevelDistance = 3000.0f;  // DAT_00f41b3c
static constexpr float kPitchDiveDistance = 1500.0f;   // DAT_00f41ad0
static constexpr float kPitchFlareDistance = 500.0f;   // DAT_00f417f4
static constexpr float kPitchDiveAngle = -0.34909061f; // DAT_00f41e58 (-20 deg)

// Roll (banking) profile
static constexpr float kRollFromTurnRate = 30.0f;    // DAT_00f416e0
static constexpr float kRollMaxAngle = 0.4f;         // DAT_00f4175c
static constexpr float kRollRateOfChange = 0.1f;     // FLOAT_00f41668
static constexpr float kRollCullDistanceSq = 9.0e6f; // DAT_00f420d4
static constexpr float kCineArmDistance = 1000.0f;   // DAT_00f41718
static constexpr float kCinePauseRatio = 0.75f;      // DAT_00f416a4

// Spatial sound envelope, per aircraft state
static constexpr float kSoundVolumeScale = 1.0f;      // DAT_00f7c588
static constexpr float kSoundGroundClipNear = 50.0f;  // DAT_00f7c58c
static constexpr float kSoundGroundClipFar = 470.0f;  // DAT_00f7c590
static constexpr float kSoundAirClipNear = 50.0f;     // DAT_00f7c594
static constexpr float kSoundAirClipFar = 2000.0f;    // DAT_00f7c598
static constexpr float kSoundRefractionFactor = 0.7f; // DAT_00f7c59c
static constexpr float kSoundFadeIn = 0.1f;

#ifdef OTDU_USE_IMGUI
#include "world/streaming_manager.h"

static int32_t gFocusedAircraft = 0;
static float gFocusDistance = 60.0f;
static bool gFollowFocusedAircraft = false;

static void FocusCameraOnAircraft( Entity *pAircraft,
                                   bool bRecenterStreaming ) {
    if ( pAircraft == nullptr || gpCamera == nullptr ) {
        return;
    }

    const Matrix4x4f_t &modelMatrix = pAircraft->getModelMatrix();
    const Eigen::Vector4f position( modelMatrix( 3, 0 ), modelMatrix( 3, 1 ),
                                    modelMatrix( 3, 2 ), 1.0f );
    gpCamera->focusOnPoint( position, gFocusDistance );

    if ( bRecenterStreaming && gpWorld != nullptr &&
         gpWorld->getStreamingManager() != nullptr ) {
        gpWorld->getStreamingManager()->requestDebugJump( position );
    }
}
#endif

// Get the world position an ET_VELU path marker sits at
static Eigen::Vector4f GetVolumePosition( const Entity *pVolume ) {
    const Matrix4x4f_t &modelMatrix = pVolume->getModelMatrix();
    return Eigen::Vector4f( modelMatrix( 3, 0 ), modelMatrix( 3, 1 ),
                            modelMatrix( 3, 2 ), modelMatrix( 3, 3 ) );
}

static float GetAmbianceVolumeFactor() {
    return gpAmbiance != nullptr ? gpAmbiance->getInternalViewVolumeFactor()
                                 : 1.0f;
}

static Eigen::Vector3f GetWorldShift() {
    if ( gpWorldStreaming != nullptr && gpWorldStreaming->isEnabled() ) {
        return gpWorldStreaming->getPosition().head<3>();
    }
    return Eigen::Vector3f::Zero();
}

float AircraftPath::getCurveU( uint32_t index ) const {
    if ( index == kInvalidIndex || index >= NumPathPoints ) {
        return -1.0f;
    }
    return pPathCurvesU[index];
}

void AircraftPath::addPathPoint( const Eigen::Vector4f &position ) {
    if ( NumPathPoints >= kMaxPathPoints ) {
        OTDU_ASSERT_ONCE( NumPathPoints < kMaxPathPoints );
        return;
    }
    pPathPoints[NumPathPoints] = position;
    NumPathPoints++;
}

void AircraftPath::setLandingTrack( const Eigen::Vector4f &start,
                                    const Eigen::Vector4f &end,
                                    uint32_t endIndex ) {
    LandingTrack.PointStart = start;
    LandingTrack.PointEnd = end;
    LandingTrack.Length = ( end - start ).norm();
    IndiceLandingEnd = endIndex;
}

void AircraftPath::buildCurves() {
    // FUN_00903c20
    NumPathCurves = NumPathPoints;
    if ( NumPathCurves == 0 ) {
        return;
    }

    Eigen::Vector4f inTangent = Normalize( pPathPoints[1] - pPathPoints[0] );
    Eigen::Vector4f previous = pPathPoints[0];

    for ( uint32_t i = 0; i < NumPathCurves; i++ ) {
        const uint32_t currentID = ( i + 1 ) % NumPathCurves;
        const uint32_t nextID = ( i + 2 ) % NumPathCurves;

        const Eigen::Vector4f current = pPathPoints[currentID];
        Eigen::Vector4f outTangent =
            Normalize( pPathPoints[nextID] - previous );

        if ( IndiceLandingEnd != kInvalidIndex &&
             IndiceLandingEnd < NumPathPoints && i == IndiceLandingEnd - 1 ) {
            inTangent = Normalize( pPathPoints[IndiceLandingEnd] -
                                   pPathPoints[IndiceLandingEnd - 1] );
        }

        if ( i == 0 ) {
            outTangent = inTangent;
        } else if ( i == NumPathCurves - 1 ) {
            outTangent = Normalize( pPathPoints[1] - pPathPoints[0] );
        } else if ( currentID == IndiceLandingEnd &&
                    IndiceLandingEnd != kInvalidIndex &&
                    IndiceLandingEnd < NumPathPoints ) {
            outTangent = Normalize( pPathPoints[IndiceLandingEnd] -
                                    pPathPoints[IndiceLandingEnd - 1] );
        }

        const float handleLength =
            ( current - previous ).norm() * kCurveHandleFactor;

        // FUN_0040a5c0
        CubicBezier &curve = pPathCurves[i];
        curve.ControlPoints[0] = previous;
        curve.ControlPoints[1] = previous + inTangent * handleLength;
        curve.ControlPoints[2] = current - outTangent * handleLength;
        curve.ControlPoints[3] = current;

        inTangent = outTangent;
        previous = current;
    }

    TotalPathLength = 0.0f;
    for ( uint32_t i = 0; i < NumPathCurves; i++ ) {
        pPathCurvesLength[i] = pPathCurves[i].computeLength( kArcLengthStep );
        TotalPathLength += pPathCurvesLength[i];
    }

    float travelled = 0.0f;
    for ( uint32_t i = 0; i < NumPathCurves; i++ ) {
        pPathCurvesU[i] =
            TotalPathLength > 0.0f ? ( travelled / TotalPathLength ) : 0.0f;
        travelled += pPathCurvesLength[i];
    }
    pPathCurvesU[NumPathCurves] = 1.0f;

    const uint32_t numLandingPoints =
        IndiceLandingEnd != kInvalidIndex ? IndiceLandingEnd : 0;
    for ( uint32_t i = 0; i < numLandingPoints; i++ ) {
        AngleXCurve.addPoint( getCurveU( i ), 0.0f );
    }

    if ( TotalPathLength > 0.0f ) {
        AngleXCurve.addPoint( 1.0f - kPitchLevelDistance / TotalPathLength,
                              0.0f );
        AngleXCurve.addPoint( 1.0f - kPitchDiveDistance / TotalPathLength,
                              kPitchDiveAngle );
        AngleXCurve.addPoint( 1.0f - kPitchFlareDistance / TotalPathLength,
                              kPitchDiveAngle );
    }
    AngleXCurve.addPoint( 1.0f, 0.0f );
}

void AircraftPath::buildSpeedCurve( float speedOnGround, float speedOnAir ) {
    // FUN_0040c1f0
    const float wrapSpeed = ( speedOnGround + speedOnAir ) * 0.5f;

    SpeedCurve.addPoint( 0.0f, wrapSpeed );

    const uint32_t landingEnd =
        IndiceLandingEnd != kInvalidIndex ? IndiceLandingEnd : 0;
    for ( uint32_t i = 1; i < landingEnd; i++ ) {
        SpeedCurve.addPoint( getCurveU( i ), speedOnGround );
    }

    SpeedCurve.addPoint( getCurveU( IndiceLandingEnd ), speedOnAir );
    SpeedCurve.addPoint( 1.0f, wrapSpeed );
}

bool AircraftPath::evaluate( float u,
                             Eigen::Vector4f &outPosition,
                             Eigen::Vector4f &outTangent ) const {
    // FUN_00903b90
    if ( NumPathCurves == 0 ) {
        return false;
    }

    uint32_t curveID = 0;
    while ( pPathCurvesU[curveID + 1] < u ) {
        curveID++;
        if ( curveID >= NumPathCurves ) {
            return false;
        }
    }

    const float span = pPathCurvesU[curveID + 1] - pPathCurvesU[curveID];
    const float t = ( u - pPathCurvesU[curveID] ) / span;

    outPosition = pPathCurves[curveID].evaluate( t );
    outTangent = pPathCurves[curveID].evaluateTangent( t );
    return true;
}

Aircraft::Aircraft()
    : AircraftPath(),
      CurrentU( 0.0f ),
      pInstance( nullptr ),
      CurrentSpeed( 0.0f ),
      CurrentRoll( 0.0f ),
      bCineWindowLatched( 0 ),
      bCineArmed( 0 ),
      bCinePauseRequested( 0 ),
      bSoundDirty( 1 ),
      bCineDriven( 0 ),
      bCinePauseCounted( 0 ),
      pSoundInstance( nullptr ),
      pSoundInAir( nullptr ),
      pSoundOnGround( nullptr ),
      State( 0 ) {
    // FUN_0040c300
}

Aircraft::~Aircraft() { terminateSound(); }

void Aircraft::spawn( Entity *pAircraftInstance,
                      AudioDispatcherPlaybackSoundInstance *pSound,
                      const edAudioSample *pInSoundOnGround,
                      const edAudioSample *pInSoundInAir,
                      float startU ) {
    // FUN_00904ed0 (per aircraft part)
    CurrentU = startU;
    pInstance = pAircraftInstance;
    pSoundInstance = pSound;

    if ( pSoundInstance == nullptr ) {
        return;
    }

    pSoundInAir = pInSoundInAir;
    pSoundOnGround = pInSoundOnGround;

    bSoundDirty = ( bSoundDirty != 0 || State != 0 ) ? 1 : 0;
    State = 0;

    pSoundInstance->setLoopCount( -1 );
    pSoundInstance->setVolume( GetAmbianceVolumeFactor() * kSoundVolumeScale );
    pSoundInstance->setRefractionFactor( kSoundRefractionFactor );
    updateSound( false );
}

bool Aircraft::executeAction( int32_t event,
                              void *pParameters,
                              void * /*pData*/ ) {
    // FUN_00903010
    (void)pParameters;
    if ( event == CE_Ended ) {
        bCineDriven = 0;
    }
    return true;
}

void Aircraft::seedSpeed() {
    // The flight starts at the speed its own profile prescribes for the point
    // it is spawned on, so it does not have to accelerate from a standstill.
    CurrentSpeed = SpeedCurve.evaluate( CurrentU );
}

void Aircraft::terminateSound() {
    // FUN_009037e0
    if ( pSoundInstance == nullptr ) {
        return;
    }
    pSoundInstance->stop( 0.0f );
    pSoundInstance->release();
    pSoundInstance = nullptr;
}

void Aircraft::updateSound( bool bCineShifted ) {
    // FUN_00903990
    if ( pSoundInstance == nullptr || pInstance == nullptr ) {
        return;
    }

    Eigen::Vector3f position = pInstance->getModelMatrix().row( 3 ).head<3>();
    if ( bCineShifted ) {
        position += GetWorldShift();
    }

    pSoundInstance->setPosition(
        position, AudioDispatcherPlaybackInstanceValues::ADPCS_World );
    pSoundInstance->setVolume( GetAmbianceVolumeFactor() * kSoundVolumeScale );

    if ( bSoundDirty != 0 ) {
        if ( State == 1 ) {
            pSoundInstance->setSample( pSoundOnGround );
            pSoundInstance->setClipDistances( kSoundGroundClipNear,
                                              kSoundGroundClipFar );
        } else {
            pSoundInstance->setSample( pSoundInAir );
            pSoundInstance->setClipDistances( kSoundAirClipNear,
                                              kSoundAirClipFar );
        }
        bSoundDirty = 0;
    }

    if ( pSoundInstance->isStopped() ) {
        pSoundInstance->play( kSoundFadeIn );
    }
}

float Aircraft::computeTurnAngle( const Eigen::Vector4f &tangent,
                                  float lookAheadU,
                                  float divisor ) const {
    // FUN_00904350
    Eigen::Vector4f lookAheadPosition = Eigen::Vector4f::Zero();
    Eigen::Vector4f lookAheadTangent = Eigen::Vector4f::Zero();
    evaluate( lookAheadU, lookAheadPosition, lookAheadTangent );

    Eigen::Vector4f from = tangent;
    from.y() = 0.0f;
    from = Normalize( from );

    Eigen::Vector4f to = lookAheadTangent;
    to.y() = 0.0f;
    to = Normalize( to );

    const Eigen::Vector4f axis = Vector4DCross( from, to );

    float dot = from.dot( to );
    if ( dot < -1.0f ) {
        dot = -1.0f;
    } else if ( dot > 1.0f ) {
        dot = 1.0f;
    }

    const float angle = std::acos( dot );
    const float signedAngle = ( axis.y() < 0.0f ) ? -angle : angle;

    return divisor != 0.0f ? ( signedAngle / divisor ) : 0.0f;
}

void Aircraft::updateLandingState( float landingEndU ) {
    if ( CurrentU < 0.0f || landingEndU < CurrentU ) {
        bSoundDirty = ( bSoundDirty != 0 || State != 0 ) ? 1 : 0;
        State = 0;
        return;
    }

    bSoundDirty = ( bSoundDirty == 0 && State == 1 ) ? 0 : 1;
    State = 1;
    bCineArmed = 1;
}

void Aircraft::updateCinematic( AircraftPathFlights &owner,
                                const Matrix4x4f_t &aircraftMatrix,
                                float landingEndU ) {
    const float cineStartU = getCurveU( IndiceCineStart );
    if ( cineStartU < 0.0f ) {
        return;
    }

    EntityCinematic *pCinematic = owner.getCinematic();
    EntityAircraftPath *pPathEntity = owner.getPathEntity();

    if ( State != 1 || CurrentU < cineStartU ) {
        if ( landingEndU >= CurrentU ) {
            return;
        }

        bCineWindowLatched = 0;
        if ( pCinematic != nullptr && bCineArmed != 0 &&
             bCinePauseCounted != 0 &&
             ( pPathEntity == nullptr ||
               owner.getOriginalPath().getTotalLength() >
                   pPathEntity->getMinDistanceBetweenAircrafts() ) ) {
            bCineArmed = 0;
            owner.releaseCinematicPause();
        }
        return;
    }

    if ( gpMngCinematics != nullptr && gpMngCinematics->isValid() &&
         pCinematic != nullptr && !pCinematic->isPlaying() &&
         bCineDriven == 0 && bCineWindowLatched == 0 ) {
        Matrix4x4f_t transformOffset = aircraftMatrix;
        transformOffset.row( 3 ).head<3>() -= GetWorldShift();

        pCinematic->setGameModeInterface( this );
        pCinematic->setPlane( static_cast<EntityActor *>( pInstance ) );
        pCinematic->setTransformOffset( transformOffset );
        bCineDriven = 1;
        pCinematic->start( true );
        CurrentU = getCurveU( IndiceCineStop );
    }

    bCineWindowLatched = 1;
}

void Aircraft::tick( AircraftPathFlights &owner, float deltaTime ) {
    // FUN_009044e0
    const AircraftPath &originalPath = owner.getOriginalPath();
    const float originalLength = originalPath.getTotalLength();
    if ( originalLength <= 0.0f ) {
        return;
    }

    float speed = originalPath.getSpeedCurve().evaluate( CurrentU );
    const float speedStep = deltaTime * kSpeedRateOfChange;
    if ( speedStep < speed - CurrentSpeed ) {
        speed = CurrentSpeed + speedStep;
    }
    if ( speed - CurrentSpeed < -speedStep ) {
        speed = CurrentSpeed - speedStep;
    }
    CurrentSpeed = speed;

    if ( bCineDriven == 0 ) {
        CurrentU += ( speed * deltaTime ) / originalLength;
    }
    if ( CurrentU > 1.0f ) {
        CurrentU -= static_cast<float>( static_cast<int32_t>( CurrentU ) );
    }

    float lookAheadU = speed / originalLength + CurrentU;
    if ( lookAheadU > 1.0f ) {
        lookAheadU -= std::floor( lookAheadU );
    }
    if ( bCineDriven != 0 ) {
        lookAheadU = 0.0f;
    }

    Eigen::Vector4f position = Eigen::Vector4f::Zero();
    Eigen::Vector4f tangent = Eigen::Vector4f::Zero();
    evaluate( CurrentU, position, tangent );
    tangent = Normalize( tangent );

    const Eigen::Vector4f right = Vector4DCross( kWorldUpVector, tangent );
    const Eigen::Vector4f up = Vector4DCross( tangent, right );

    Matrix4x4f_t aircraftMatrix = Matrix4x4f_t::Identity();
    aircraftMatrix.row( 0 ) << right.x(), right.y(), right.z(), right.w();
    aircraftMatrix.row( 1 ) << up.x(), up.y(), up.z(), up.w();
    aircraftMatrix.row( 2 ) << tangent.x(), tangent.y(), tangent.z(),
        tangent.w();
    aircraftMatrix.row( 3 ) << position.x(), position.y(), position.z(),
        position.w();

    Eigen::Vector4f listener = Eigen::Vector4f::Zero();
    if ( gpCamera != nullptr && gpCamera->getActiveCamera() != nullptr ) {
        listener.head<3>() = gpCamera->getActiveCamera()->getWorldPosition();
    }
    const float distanceToCameraSq = ( listener - position ).squaredNorm();

    if ( State != 1 ) {
        float roll = 0.0f;
        if ( distanceToCameraSq <= kRollCullDistanceSq ) {
            roll = -computeTurnAngle( tangent, lookAheadU, 1.0f );
        }
        roll *= kRollFromTurnRate;
        roll = std::min( 1.0f, std::max( -1.0f, roll ) ) * kRollMaxAngle;

        const float invLength =
            TotalPathLength > 0.0f ? ( 1.0f / TotalPathLength ) : 0.0f;

        // Wings level for the final approach
        if ( 1.0f - invLength * kPitchFlareDistance < CurrentU ) {
            roll = 0.0f;
        }

        EntityCinematic *pCinematic = owner.getCinematic();
        EntityAircraftPath *pPathEntity = owner.getPathEntity();
        if ( CurrentU <= 1.0f - invLength * kCineArmDistance ||
             pCinematic == nullptr ) {
            bCinePauseRequested = 0;
        } else if ( speed > 0.0f && pPathEntity != nullptr &&
                    bCinePauseRequested == 0 && pCinematic->isPlaying() ) {
            const float timeToRunway =
                ( 1.0f - CurrentU ) * originalLength / speed -
                pPathEntity->getMinDistanceBetweenAircrafts() / speed;
            const float cineTimeLeft =
                pCinematic->getDuration() - pCinematic->getTime();
            if ( timeToRunway < cineTimeLeft ) {
                owner.requestCinematicPause();
                bCinePauseRequested = 1;
                bCinePauseCounted = 1;
            }
        }

        const float rollStep = deltaTime * kRollRateOfChange;
        if ( rollStep < roll - CurrentRoll ) {
            roll = CurrentRoll + rollStep;
        }
        if ( roll - CurrentRoll < -rollStep ) {
            roll = CurrentRoll - rollStep;
        }
        CurrentRoll = roll;

        aircraftMatrix = MakeRotationMatZ( CurrentRoll ) * aircraftMatrix;
    }

    const float pitch = AngleXCurve.evaluate( CurrentU );
    aircraftMatrix = MakeRotationMatX( pitch ) * aircraftMatrix;

    if ( bCineDriven == 0 && pInstance != nullptr ) {
        pInstance->setTransform( aircraftMatrix, true );
    }

    updateSound( bCineDriven != 0 );

    const float landingEndU = originalPath.getCurveU( IndiceLandingEnd );
    updateLandingState( landingEndU );
    updateCinematic( owner, aircraftMatrix, landingEndU );

    if ( State == 1 ) {
        // On the runway the aircraft is dropped onto the ground so it rolls
        // along the terrain instead of following the bezier through it.
        ProjectActorOnGround( pInstance, false, false );
    }
}

Entity *AircraftPathFlights::getAircraft( uint32_t index ) const {
    if ( index >= flights.size() ) {
        return nullptr;
    }
    return flights[index]->getInstance();
}

const Aircraft *AircraftPathFlights::getFlight( uint32_t index ) const {
    if ( index >= flights.size() ) {
        return nullptr;
    }
    return flights[index].get();
}

void AircraftPathFlights::initialize( EntityAircraftPath *pEntity ) {
    // FUN_009033f0
    pPathEntity = pEntity;
    flights.clear();

    PauseCineRefCount = 0;
    bLaunchCine = true;
    pCinematic = pEntity->getCinematic();

    const uint32_t numPoints = pEntity->getPathPointCount();

    Eigen::Vector4f start = Eigen::Vector4f::Zero();
    if ( pEntity->getPathPointVolume( 0 ) != nullptr ) {
        start = GetVolumePosition( pEntity->getPathPointVolume( 0 ) );
    }

    Eigen::Vector4f end = Eigen::Vector4f::Zero();
    uint32_t landingEndIndex = AircraftPath::kInvalidIndex;
    for ( uint32_t i = 1; i < numPoints; i++ ) {
        if ( pEntity->getPathPointType( i ) !=
             EntityAircraftPath::APPT_Waypoint ) {
            continue;
        }
        if ( pEntity->getPathPointVolume( i ) != nullptr ) {
            end = GetVolumePosition( pEntity->getPathPointVolume( i ) );
        }
        landingEndIndex = i;
        break;
    }
    originalPath.setLandingTrack( start, end, landingEndIndex );

    for ( uint32_t i = 0; i < numPoints; i++ ) {
        const Entity *pVolume = pEntity->getPathPointVolume( i );
        originalPath.addPathPoint( pVolume != nullptr
                                       ? GetVolumePosition( pVolume )
                                       : Eigen::Vector4f::Zero() );
    }

    originalPath.buildCurves();
    originalPath.buildSpeedCurve( pEntity->getSpeedOnGround(),
                                  pEntity->getSpeedOnAir() );
}

void AircraftPathFlights::buildFlightPath( Aircraft &aircraft ) {
    // FUN_00904ed0
    const uint32_t numPoints = pPathEntity->getPathPointCount();
    RandomGen &randomGen = gpScenaric->getRandomGenerator();

    Eigen::Vector4f start = Eigen::Vector4f::Zero();
    if ( pPathEntity->getPathPointVolume( 0 ) != nullptr ) {
        start = GetVolumePosition( pPathEntity->getPathPointVolume( 0 ) );
    }

    Eigen::Vector4f end = Eigen::Vector4f::Zero();
    uint32_t landingEndIndex = AircraftPath::kInvalidIndex;
    for ( uint32_t i = 1; i < numPoints; i++ ) {
        if ( pPathEntity->getPathPointType( i ) !=
             EntityAircraftPath::APPT_Waypoint ) {
            continue;
        }
        if ( pPathEntity->getPathPointVolume( i ) != nullptr ) {
            end = GetVolumePosition( pPathEntity->getPathPointVolume( i ) );
        }
        landingEndIndex = i;
        break;
    }
    aircraft.setLandingTrack( start, end, landingEndIndex );

    for ( uint32_t i = 0; i < numPoints; i++ ) {
        Entity *pVolume = pPathEntity->getPathPointVolume( i );

        switch ( pPathEntity->getPathPointType( i ) ) {
        case EntityAircraftPath::APPT_Scatter:
            if ( pVolume != nullptr ) {
                Eigen::Vector4f jitter;
                jitter.x() = RandomUnit( randomGen ) - 0.5f;
                jitter.y() = RandomUnit( randomGen ) - 0.5f;
                jitter.z() = RandomUnit( randomGen ) - 0.5f;
                jitter.w() = 1.0f;
                aircraft.addPathPoint(
                    TDUMat4x4MulVec( pVolume->getModelMatrix(), jitter ) );
            }
            break;
        case EntityAircraftPath::APPT_CineStart:
            aircraft.setCineStartIndex( i );
            if ( pVolume != nullptr ) {
                aircraft.addPathPoint( GetVolumePosition( pVolume ) );
            }
            break;
        case EntityAircraftPath::APPT_CineStop:
            aircraft.setCineStopIndex( i );
            if ( pVolume != nullptr ) {
                aircraft.addPathPoint( GetVolumePosition( pVolume ) );
            }
            break;
        case EntityAircraftPath::APPT_Waypoint:
        default:
            if ( pVolume != nullptr ) {
                aircraft.addPathPoint( GetVolumePosition( pVolume ) );
            }
            break;
        }
    }

    aircraft.buildCurves();
    aircraft.buildSpeedCurve( pPathEntity->getSpeedOnGround(),
                              pPathEntity->getSpeedOnAir() );
}

void AircraftPathFlights::spawnAircrafts( MngActor &mngActor,
                                          AudioClient *pAudioClient ) {
    // FUN_00904ed0
    if ( pPathEntity == nullptr || gpScenaric == nullptr ||
         gpAircraft == nullptr ) {
        return;
    }

    RandomGen &randomGen = gpScenaric->getRandomGenerator();

    const float minSpacing = pPathEntity->getMinDistanceBetweenAircrafts();
    const float spacingRange =
        pPathEntity->getMaxDistanceBetweenAircrafts() - minSpacing;
    const float pathLength = originalPath.getTotalLength();

    float distance = RandomUnit( randomGen ) * spacingRange + minSpacing;
    while ( flights.size() < kMaxAircraftsPerPath &&
            distance < pathLength - minSpacing ) {
        Entity *pTemplate =
            gpAircraft->pickAircraftModel( pPathEntity->getAircraftType() );
        if ( pTemplate == nullptr ) {
            return;
        }

        Entity *pInstance = mngActor.instanciateActor( pTemplate );
        if ( pInstance != nullptr ) {
            EntityAircraft *pAircraftInstance =
                static_cast<EntityAircraft *>( pInstance );
            const uint32_t numLiveries =
                static_cast<uint32_t>( pAircraftInstance->getNumMaterials() );
            pAircraftInstance->setLivery( static_cast<int32_t>(
                RandomBelow( randomGen, numLiveries + 1 ) ) );

            flights.push_back( std::make_unique<Aircraft>() );
            Aircraft &aircraft = *flights.back();

            AudioDispatcherPlaybackSoundInstance *pSound =
                pAudioClient != nullptr ? pAudioClient->acquireSoundInstance()
                                        : nullptr;
            aircraft.spawn(
                pInstance, pSound, pPathEntity->getSoundOnGroundSample(),
                pPathEntity->getSoundInAirSample(),
                pathLength > 0.0f ? ( distance / pathLength ) : 0.0f );

            buildFlightPath( aircraft );
            aircraft.seedSpeed();
        }

        distance += RandomUnit( randomGen ) * spacingRange + minSpacing;
    }
}

void AircraftPathFlights::tick( float deltaTime ) {
    // FUN_009044e0
    for ( std::unique_ptr<Aircraft> &pAircraft : flights ) {
        pAircraft->tick( *this, deltaTime );
    }

    if ( pCinematic == nullptr || !pCinematic->isPlaying() ) {
        return;
    }

    if ( PauseCineRefCount < 1 ) {
        pCinematic->setPaused( false );
    } else if ( pCinematic->getDuration() > 0.0f &&
                pCinematic->getTime() / pCinematic->getDuration() >
                    kCinePauseRatio ) {
        pCinematic->setPaused( true );
    }
}

void AircraftPathFlights::terminate() {
    // FUN_00903090 / FUN_00903ad0
    for ( std::unique_ptr<Aircraft> &pAircraft : flights ) {
        pAircraft->terminateSound();

        if ( pCinematic != nullptr &&
             pCinematic->getGameModeInterface() == pAircraft.get() ) {
            pCinematic->setGameModeInterface( nullptr );
        }
        if ( pCinematic != nullptr &&
             pCinematic->getPlane() == pAircraft->getInstance() ) {
            pCinematic->setPlane( nullptr );
        }
    }
    flights.clear();
}

GSAircraft::GSAircraft()
    : GameSystem(),
      mngActor(),
      pAudioClient( nullptr ),
      bDrawAircrafts( true ),
      bAircraftsSpawned( false ) {
    // FUN_00903150
    OTDU_ASSERT( gpAircraft == nullptr );
    gpAircraft = this;
}

GSAircraft::~GSAircraft() {
    // FUN_0040c490
    if ( gpAircraft == this ) {
        gpAircraft = nullptr;
    }
}

bool GSAircraft::initialize( TestDriveGameInstance *param_2 ) {
    // FUN_009033f0
    pGameInstance = param_2;
    bPaused = false;

    originalPaths.clear();
    bDrawAircrafts = true;

    if ( gpAudio != nullptr ) {
        pAudioClient = gpAudio->createClient( "ClientAircraft" );
    }

    mngActor.initialize( param_2 );
    mngActor.reset();

    return true;
}

static Instance *GetAircraftInstance( Entity *pAircraft ) {
    if ( pAircraft == nullptr ) {
        return nullptr;
    }
    SerializedSkinnedModel *pModel =
        static_cast<SerializedSkinnedModel *>( pAircraft->getGeometryData() );
    return ( pModel != nullptr ) ? pModel->pInstance : nullptr;
}

void AircraftPathFlights::setMapViewLOD( const bool bMapView ) {
    // FUN_00903760
    for ( const std::unique_ptr<Aircraft> &pFlight : flights ) {
        Instance *pInstance = GetAircraftInstance( pFlight->getInstance() );
        if ( pInstance == nullptr || pInstance->getNumLODs() < 2 ) {
            continue;
        }

        if ( bMapView ) {
            pInstance->getFlagsWrite() &= ~2u;
            pInstance->setActiveLODIndex( 1 );
        } else {
            pInstance->getFlagsWrite() |= 2u;
        }
    }
}

void AircraftPathFlights::setMapViewMaterials( const bool bMapView ) {
    // FUN_00903870
    const uint16_t ot = bMapView ? kAircraftMapViewOT : kAircraftDefaultOT;
    for ( const std::unique_ptr<Aircraft> &pFlight : flights ) {
        Instance *pInstance = GetAircraftInstance( pFlight->getInstance() );
        if ( pInstance == nullptr ) {
            continue;
        }

        RenderFile::Section *pBankMat = pInstance->getMaterialBank();
        const uint32_t numMaterials =
            ( pBankMat != nullptr ) ? ( pBankMat->DataSize - 0x10 ) / 0x10 : 0;
        for ( uint32_t i = 0; i < numMaterials; i++ ) {
            Material *pMaterial = pInstance->getMaterial( i );
            if ( pMaterial == nullptr || pMaterial->NumParams == 0 ) {
                continue;
            }
            if ( pMaterial->getParameterByIndex( 0 )->Hashcode ==
                 kAircraftMapViewMaterialHashcode ) {
                pMaterial->OT = ot;
            }
        }
    }
}

void GSAircraft::setMapViewLOD( const bool bMapView ) {
    // FUN_00903050
    for ( const std::unique_ptr<AircraftPathFlights> &pPath : originalPaths ) {
        pPath->setMapViewLOD( bMapView );
    }
}

void GSAircraft::setMapViewMaterials( const bool bMapView ) {
    // FUN_009030d0
    for ( const std::unique_ptr<AircraftPathFlights> &pPath : originalPaths ) {
        pPath->setMapViewMaterials( bMapView );
    }
}

void GSAircraft::spawnAircrafts() {
    static const bool bOtduOff = getenv( "OTDU_NO_AIRCRAFT" ) != nullptr;
    if ( bOtduOff ) {
        return;
    }
    // FUN_009033f0 + FUN_009033a0
    if ( bAircraftsSpawned || gpScenaric == nullptr ) {
        return;
    }
    bAircraftsSpawned = true;

    buildAircraftModelGroups();

    ScenaricBank *pPathBank = gpScenaric->getAircraftsPathsBank();
    if ( pPathBank == nullptr ) {
        return;
    }

    int32_t numPathEntities = 0;
    for ( int32_t i = 0; i < pPathBank->getEntityCount(); i++ ) {
        Entity *pEntity = pPathBank->getEntity( i );
        if ( pEntity == nullptr ||
             pEntity->getType() != eEntityType::ET_ACPA ) {
            continue;
        }
        numPathEntities++;

        EntityAircraftPath *pPathEntity =
            static_cast<EntityAircraftPath *>( pEntity );

        const uint32_t numPoints = pPathEntity->getPathPointCount();
        if ( originalPaths.size() >= kMaxOriginalPaths || numPoints <= 2 ||
             numPoints >= EntityAircraftPath::kMaxPathPoints ) {
            continue;
        }

        originalPaths.push_back( std::make_unique<AircraftPathFlights>() );
        originalPaths.back()->initialize( pPathEntity );
    }

    int32_t numAircrafts = 0;
    for ( std::unique_ptr<AircraftPathFlights> &pPath : originalPaths ) {
        pPath->spawnAircrafts( mngActor, pAudioClient );
        numAircrafts += static_cast<int32_t>( pPath->getNumAircrafts() );
    }

    OTDU_LOG_DEBUG( "Spawned %d aircraft(s) on %d of %d path(s); aircraft "
                    "models per group: %d/%d\n",
                    numAircrafts, static_cast<int32_t>( originalPaths.size() ),
                    numPathEntities,
                    static_cast<int32_t>( aircraftModelGroups[0].numModels ),
                    static_cast<int32_t>( aircraftModelGroups[1].numModels ) );
}

void GSAircraft::tick( float totalTime, float deltaTime ) {
    static const bool bOtduOff = getenv( "OTDU_NO_AIRCRAFT" ) != nullptr;
    if ( bOtduOff ) {
        return;
    }
    // FUN_00903350
    (void)totalTime;

    mngActor.tick( totalTime, deltaTime );

    for ( std::unique_ptr<AircraftPathFlights> &pPath : originalPaths ) {
        pPath->tick( deltaTime );
    }

#ifdef OTDU_USE_IMGUI
    if ( gFollowFocusedAircraft ) {
        FocusCameraOnAircraft(
            getAircraft( static_cast<uint32_t>( gFocusedAircraft ) ), false );
    }
#endif
}

void GSAircraft::draw( float deltaTime ) {
    static const bool bOtduOff = getenv( "OTDU_NO_AIRCRAFT" ) != nullptr;
    if ( bOtduOff ) {
        return;
    }
    // FUN_00903030
    (void)deltaTime;
    if ( !bDrawAircrafts ) {
        return;
    }
    mngActor.draw();
}

void GSAircraft::terminate() {
    // FUN_00903210
    for ( std::unique_ptr<AircraftPathFlights> &pPath : originalPaths ) {
        pPath->terminate();
    }
    originalPaths.clear();

    pAudioClient = nullptr;

    mngActor.terminate();
    mngActor.reset();

    for ( AircraftModelGroup &group : aircraftModelGroups ) {
        group.numModels = 0;
    }

    bAircraftsSpawned = false;
}

uint32_t GSAircraft::getNumAircrafts() const {
    uint32_t numAircrafts = 0;
    for ( const std::unique_ptr<AircraftPathFlights> &pPath : originalPaths ) {
        numAircrafts += pPath->getNumAircrafts();
    }
    return numAircrafts;
}

Entity *GSAircraft::getAircraft( uint32_t index ) const {
    const Aircraft *pFlight = getFlight( index );
    return pFlight != nullptr ? pFlight->getInstance() : nullptr;
}

const Aircraft *GSAircraft::getFlight( uint32_t index ) const {
    uint32_t pathIndex = 0;
    uint32_t firstOfPath = 0;
    while ( pathIndex < originalPaths.size() ) {
        const uint32_t nextPathFirst =
            firstOfPath + originalPaths[pathIndex]->getNumAircrafts();
        if ( index < nextPathFirst ) {
            break;
        }
        pathIndex++;
        firstOfPath = nextPathFirst;
    }

    if ( pathIndex >= originalPaths.size() ) {
        return nullptr;
    }
    return originalPaths[pathIndex]->getFlight( index - firstOfPath );
}

Entity *GSAircraft::pickAircraftModel( int32_t groupIndex ) {
    if ( gpScenaric == nullptr || groupIndex < 0 ||
         groupIndex >= static_cast<int32_t>( kNumAircraftGroups ) ) {
        return nullptr;
    }

    AircraftModelGroup &group = aircraftModelGroups[groupIndex];
    if ( group.numModels == 0 ) {
        return nullptr;
    }

    uint32_t index =
        RandomBelow( gpScenaric->getRandomGenerator(), group.numModels );
    if ( index >= group.numModels ) {
        index = group.numModels - 1;
    }
    return group.pModels[index];
}

void GSAircraft::buildAircraftModelGroups() {
    // FUN_009c66c0
    for ( AircraftModelGroup &group : aircraftModelGroups ) {
        group.numModels = 0;
    }

    ScenaricBank *pAircraftBank = gpScenaric->getAircraftBank();
    if ( pAircraftBank == nullptr ) {
        return;
    }

    for ( int32_t i = 0; i < pAircraftBank->getEntityCount(); i++ ) {
        Entity *pEntity = pAircraftBank->getEntity( i );
        if ( pEntity == nullptr ||
             pEntity->getType() != eEntityType::ET_AIRC ) {
            continue;
        }

        const int32_t aircraftType =
            static_cast<EntityAircraft *>( pEntity )->getAircraftType();
        if ( aircraftType < 0 ||
             aircraftType >= static_cast<int32_t>( kNumAircraftGroups ) ) {
            continue;
        }

        AircraftModelGroup &group = aircraftModelGroups[aircraftType];
        if ( group.numModels < kMaxAircraftModelsPerGroup ) {
            group.pModels[group.numModels++] = pEntity;
        }
    }
}

#ifdef OTDU_USE_IMGUI
void GSAircraft::drawDebug( float deltaTime ) {
    (void)deltaTime;

    ImGui::SeparatorText( "Aircraft paths" );
    ImGui::Text( "%d path(s) | %d aircraft(s) | models per group %d/%d",
                 static_cast<int32_t>( originalPaths.size() ),
                 static_cast<int32_t>( getNumAircrafts() ),
                 static_cast<int32_t>( aircraftModelGroups[0].numModels ),
                 static_cast<int32_t>( aircraftModelGroups[1].numModels ) );

    ImGui::Checkbox( "Draw aircrafts", &bDrawAircrafts );

    if ( originalPaths.empty() ) {
        ImGui::TextDisabled(
            "world_aircrafts has not been parsed yet (waiting on the world)." );
        return;
    }

    const int32_t numAircrafts = static_cast<int32_t>( getNumAircrafts() );
    if ( numAircrafts == 0 ) {
        ImGui::TextDisabled( "No aircraft spawned on any path." );
        return;
    }

    ImGui::SeparatorText( "Camera focus" );

#ifdef OTDU_DEVBUILD
    // Without the free-fly camera the game camera recomputes its transform
    // every tick and snaps straight back off the aircraft.
    if ( gpCamera != nullptr ) {
        bool bFreeCam = gpCamera->isFreeCamEnabled();
        if ( ImGui::Checkbox( "Free-fly camera", &bFreeCam ) ) {
            gpCamera->toggleFreeCam();
        }
    }
#endif

    if ( gFocusedAircraft >= numAircrafts ) {
        gFocusedAircraft = 0;
    }
    ImGui::SliderInt( "Aircraft", &gFocusedAircraft, 0, numAircrafts - 1 );
    ImGui::DragFloat( "Distance", &gFocusDistance, 0.5f, 2.0f, 500.0f );

    Entity *pFocused = getAircraft( static_cast<uint32_t>( gFocusedAircraft ) );
    if ( pFocused == nullptr ) {
        ImGui::TextDisabled( "Empty aircraft slot (instanciation failed)." );
        return;
    }

    const Matrix4x4f_t &focusedMatrix = pFocused->getModelMatrix();
    ImGui::Text(
        "Position %.1f %.1f %.1f | model group %d", focusedMatrix( 3, 0 ),
        focusedMatrix( 3, 1 ), focusedMatrix( 3, 2 ),
        static_cast<int32_t>(
            static_cast<EntityAircraft *>( pFocused )->getAircraftType() ) );

    const Aircraft *pFlight =
        getFlight( static_cast<uint32_t>( gFocusedAircraft ) );
    if ( pFlight != nullptr ) {
        ImGui::Text(
            "u %.3f of %.0fm | speed %.1f | %s", pFlight->getCurrentU(),
            pFlight->getTotalLength(), pFlight->getCurrentSpeed(),
            pFlight->isOnLandingTrack() ? "on the landing track" : "airborne" );
    }

    if ( ImGui::Button( "Focus camera" ) ) {
        FocusCameraOnAircraft( pFocused, true );
    }
    ImGui::SameLine();
    if ( ImGui::Button( "Next aircraft" ) ) {
        gFocusedAircraft = ( gFocusedAircraft + 1 ) % numAircrafts;
        FocusCameraOnAircraft(
            getAircraft( static_cast<uint32_t>( gFocusedAircraft ) ), true );
    }
    ImGui::SameLine();
    ImGui::Checkbox( "Follow", &gFollowFocusedAircraft );

    ImGui::TextDisabled( "'Focus camera' also recenters world streaming so the "
                         "ground below the aircraft loads; 'Follow' only moves "
                         "the camera, and tracks the aircraft as it flies." );

    ImGui::SeparatorText( "Aircrafts" );
    for ( int32_t i = 0; i < numAircrafts; i++ ) {
        Entity *pAircraft = getAircraft( static_cast<uint32_t>( i ) );
        if ( pAircraft == nullptr ) {
            ImGui::TextDisabled( "#%d: empty slot (instanciation failed)", i );
            continue;
        }

        const Matrix4x4f_t &modelMatrix = pAircraft->getModelMatrix();
        ImGui::Text( "#%d: %.1f %.1f %.1f", i, modelMatrix( 3, 0 ),
                     modelMatrix( 3, 1 ), modelMatrix( 3, 2 ) );
    }
}
#endif
