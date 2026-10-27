#include "shared.h"
#include "gs_boat.h"

#include "audio/gs_audio.h"
#include "audio/implementation/audio_dispatcher_playback.h"
#include "core/maths_helper.h"
#include "core/random/bcpl.h"
#include "core/cubic_bezier.h"
#include "render/camera/gs_camera.h"
#include "render/instance.h"

#include "game/scenaric/gs_scenaric.h"
#include "game/scenaric/scenaric_bank.h"
#include "game/scenaric/entity/base_entity.h"
#include "game/scenaric/entity/entity_boat.h"
#include "game/scenaric/entity/entity_boat_path.h"
#include "game/scenaric/entity/entity_factory.h"

GSBoat *gpBoat = nullptr;

static constexpr float kCurveHandleFactor = 1.0f / 3.0f; // DAT_00f41764
static constexpr float kArcLengthStep = 0.1f;
static constexpr float kHornIntervalBase = 10.0f;  // FLOAT_00f4166c
static constexpr float kHornIntervalRange = 40.0f; // DAT_00f41b70
static constexpr float kRandNorm = 3.0518509e-05f; // DAT_00f41714

static constexpr float kHornPitchBase = 0.9f;  // DAT_00f416a0
static constexpr float kHornPitchRange = 0.2f; // DAT_00f42058
static constexpr float kHornFadeIn = 0.1f;
static constexpr float kHornClipNear = 50.0f;
static constexpr float kHornClipFar = 2000.0f;
static constexpr float kHornMinVolume = 0.8f;
static constexpr float kMaxHornDistanceSq = 4000000.0f; // DAT_00f41fe4

#ifdef OTDU_DEVBUILD
#include "world/gs_world.h"
#include "world/streaming_manager.h"

static int32_t gFocusedBoat = 0;
static float gFocusDistance = 25.0f;
static bool gFollowFocusedBoat = false;
static bool gRenderEcume = false;

static void FocusCameraOnBoat( Entity *pBoat, bool bRecenterStreaming ) {
    if ( pBoat == nullptr || gpCamera == nullptr ) {
        return;
    }

    const Matrix4x4f_t &modelMatrix = pBoat->getModelMatrix();
    const Eigen::Vector4f position( modelMatrix( 3, 0 ), modelMatrix( 3, 1 ),
                                    modelMatrix( 3, 2 ), 1.0f );
    gpCamera->focusOnPoint( position, gFocusDistance );

    if ( bRecenterStreaming && gpWorld != nullptr &&
         gpWorld->getStreamingManager() != nullptr ) {
        gpWorld->getStreamingManager()->requestDebugJump( position );
    }
}
#endif

static CubicBezier MakeCurve( const BoatPathEntry &element ) {
    // FUN_00411ab0
    CubicBezier curve;
    for ( int32_t i = 0; i < 4; i++ ) {
        curve.ControlPoints[i] =
            Eigen::Vector4f( element.CurvePointsX[i], element.Height,
                             element.CurvePointsZ[i], 1.0f );
    }
    return curve;
}

static float ComputeHornInterval() {
    return static_cast<float>( rand() & 0x7fff ) * kRandNorm *
               kHornIntervalRange +
           kHornIntervalBase;
}

Eigen::Vector4f
BoatPathInstance::getMarkerPosition( uint32_t markerIndex ) const {
    const BoatPathEntry *pMarker = pPathElements[markerIndex * 2];
    if ( pMarker == nullptr ) {
        return Eigen::Vector4f( 0.0f, 0.0f, 0.0f, 1.0f );
    }
    return pMarker->getMarkerPosition();
}

int32_t BoatPathInstance::getMarkerType( uint32_t markerIndex ) const {
    const BoatPathEntry *pMarker = pPathElements[markerIndex * 2];
    return pMarker != nullptr ? pMarker->Type : BoatPathEntry::PET_Curve;
}

void BoatPathInstance::addWaypoint( const Eigen::Vector4f &position,
                                    uint32_t mode,
                                    float waitDuration ) {
    // FUN_008ea0e0
    if ( gpBoat == nullptr || NumPathElements + 2 > kMaxPathElements ) {
        return;
    }

    int32_t markerType = BoatPathEntry::PET_Waypoint;
    switch ( mode ) {
    case 0:
        markerType = BoatPathEntry::PET_Waypoint;
        break;
    case 1:
        markerType = BoatPathEntry::PET_Wait;
        break;
    case 2:
        markerType = BoatPathEntry::PET_Reverse;
        break;
    default:
        OTDU_ASSERT_ONCE( mode <= 2 );
        break;
    }

    BoatPathEntry *pMarker = gpBoat->allocatePathElement();
    BoatPathEntry *pCurve = gpBoat->allocatePathElement();
    if ( pMarker == nullptr || pCurve == nullptr ) {
        return;
    }

    pMarker->Type = markerType;
    pMarker->CurvePointsX[2] = waitDuration;
    pMarker->setMarkerPosition( position );
    pPathElements[NumPathElements++] = pMarker;

    pCurve->Type = BoatPathEntry::PET_Curve;
    pPathElements[NumPathElements++] = pCurve;
}

void BoatPathInstance::solveCurve( BoatPathEntry &curve,
                                   const Eigen::Vector4f &from,
                                   const Eigen::Vector4f &to,
                                   const Eigen::Vector4f &inTangent,
                                   const Eigen::Vector4f &outTangent ) {
    const float handleLength = ( to - from ).norm() * kCurveHandleFactor;

    curve.CurvePointsX[0] = from.x();
    curve.CurvePointsX[1] = from.x() + inTangent.x() * handleLength;
    curve.CurvePointsX[2] = to.x() - outTangent.x() * handleLength;
    curve.CurvePointsX[3] = to.x();

    curve.CurvePointsZ[0] = from.z();
    curve.CurvePointsZ[1] = from.z() + inTangent.z() * handleLength;
    curve.CurvePointsZ[2] = to.z() - outTangent.z() * handleLength;
    curve.CurvePointsZ[3] = to.z();

    curve.Height = from.y();
}

void BoatPathInstance::build( const BoatPathInstance *pSource, bool bLooping ) {
    // FUN_008ea350
    const uint32_t numElements = NumPathElements;
    const uint32_t numMarkers = numElements >> 1;

    if ( !bLooping ) {
        if ( numElements < 4 ) {
            return;
        }

        Eigen::Vector4f inTangent =
            Normalize( getMarkerPosition( 1 ) - getMarkerPosition( 0 ) );

        const uint32_t numJoints = numMarkers - 2;
        for ( uint32_t i = 0; i < numJoints; i++ ) {
            const Eigen::Vector4f from = getMarkerPosition( i );
            const Eigen::Vector4f to = getMarkerPosition( i + 1 );
            const Eigen::Vector4f next = getMarkerPosition( i + 2 );

            Eigen::Vector4f outTangent;
            bool bReverse = false;
            if ( getMarkerType( ( i + 2 ) % ( numMarkers - 1 ) ) ==
                 BoatPathEntry::PET_Reverse ) {
                outTangent = Normalize( to - from );
                bReverse = true;
            } else {
                bReverse = getMarkerType( i + 1 ) == BoatPathEntry::PET_Reverse;
                outTangent =
                    Normalize( bReverse ? ( to - from ) : ( next - from ) );
            }

            solveCurve( *pPathElements[i * 2 + 1], from, to, inTangent,
                        outTangent );
            inTangent = bReverse ? Eigen::Vector4f( -outTangent ) : outTangent;
        }

        // The closing joint has no marker beyond its destination to smooth
        // against, so it aims straight at it.
        const Eigen::Vector4f from = getMarkerPosition( numJoints );
        const Eigen::Vector4f to = getMarkerPosition( numJoints + 1 );
        solveCurve( *pPathElements[numJoints * 2 + 1], from, to, inTangent,
                    Normalize( to - from ) );

        pPathElements[numElements - 1]->Type = BoatPathEntry::PET_Waypoint;
    } else {
        if ( numElements < 6 ) {
            return;
        }

        Eigen::Vector4f inTangent = Normalize(
            getMarkerPosition( 1 ) - getMarkerPosition( numMarkers - 1 ) );

        for ( uint32_t i = 0; i < numMarkers; i++ ) {
            const Eigen::Vector4f from = getMarkerPosition( i );
            const Eigen::Vector4f to =
                getMarkerPosition( ( i + 1 ) % numMarkers );
            const Eigen::Vector4f next =
                getMarkerPosition( ( i + 2 ) % numMarkers );

            Eigen::Vector4f outTangent;
            bool bReverse = false;
            if ( getMarkerType( ( i + 2 ) % numMarkers ) ==
                 BoatPathEntry::PET_Reverse ) {
                outTangent = Normalize( to - from );
                bReverse = true;
            } else {
                bReverse = getMarkerType( ( i + 1 ) % numMarkers ) ==
                           BoatPathEntry::PET_Reverse;
                outTangent =
                    Normalize( bReverse ? ( to - from ) : ( next - from ) );
            }

            solveCurve( *pPathElements[i * 2 + 1], from, to, inTangent,
                        outTangent );
            inTangent = bReverse ? Eigen::Vector4f( -outTangent ) : outTangent;
        }
    }

    if ( pSource == nullptr ) {
        TotalPathLength = 0.0f;
        for ( uint32_t i = 0; i < NumPathElements; i++ ) {
            if ( pPathElements[i]->Type == BoatPathEntry::PET_Curve ) {
                TotalPathLength += MakeCurve( *pPathElements[i] )
                                       .computeLength( kArcLengthStep );
            }
        }

        float travelled = 0.0f;
        for ( uint32_t i = 0; i < NumPathElements; i++ ) {
            PathElemensU[i] =
                TotalPathLength > 0.0f ? ( travelled / TotalPathLength ) : 0.0f;
            if ( pPathElements[i]->Type == BoatPathEntry::PET_Curve ) {
                travelled += MakeCurve( *pPathElements[i] )
                                 .computeLength( kArcLengthStep );
            }
        }
        PathElemensU[NumPathElements] = 1.0f;
        PathElemensU[NumPathElements + 1] = 1.0f;
    } else {
        TotalPathLength = pSource->TotalPathLength;
        for ( uint32_t i = 0; i < NumPathElements + 2; i++ ) {
            PathElemensU[i] = pSource->PathElemensU[i];
        }
    }
}

float BoatPathInstance::findElementForU( float u, uint32_t &outIndex ) const {
    // FUN_008e9e10
    outIndex = 2;
    if ( NumPathElements > 2 ) {
        do {
            if ( u <= PathElemensU[outIndex] ) {
                break;
            }
            outIndex++;
        } while ( outIndex < NumPathElements );
    }
    outIndex--;

    const float span = PathElemensU[outIndex + 1] - PathElemensU[outIndex];
    if ( span <= 0.0f ) {
        return 0.0f;
    }
    return ( u - PathElemensU[outIndex] ) / span;
}

void BoatPathInstance::seek( float startU ) {
    // FUN_008eb0b0 (inlined)
    CurrentU = startU;
    CurrentPathElementID = 2;
    if ( NumPathElements > 2 ) {
        do {
            if ( startU <= PathElemensU[CurrentPathElementID] ) {
                break;
            }
            CurrentPathElementID++;
        } while ( CurrentPathElementID < NumPathElements );
    }
    CurrentPathElementID--;
}

void BoatPathInstance::update( float deltaTime ) {
    // FUN_008e9f40
    if ( NumPathElements == 0 ) {
        return;
    }

    uint32_t elementID = CurrentPathElementID;
    uint32_t nextID = ( elementID + 1 ) % NumPathElements;
    TimeInThisState += deltaTime;

    float targetSpeed = 0.0f;
    if ( pPathElements[elementID]->Type == BoatPathEntry::PET_Wait ) {
        targetSpeed = 0.0f;
    } else {
        const int32_t nextType = pPathElements[nextID]->Type;
        if ( nextType == BoatPathEntry::PET_Wait ||
             nextType == BoatPathEntry::PET_Reverse ) {
            uint32_t scratchIndex = 0;
            const float localU = findElementForU( CurrentU, scratchIndex );
            targetSpeed = ( 1.0f - localU * localU ) * Speed;
            if ( targetSpeed <= 1.0f ) {
                targetSpeed = 1.0f;
            }
        } else {
            targetSpeed = Speed;
        }
    }

    // Boats accelerate and brake at one unit per second squared.
    if ( deltaTime < targetSpeed - LastSpeed ) {
        targetSpeed = LastSpeed + deltaTime;
    }
    if ( targetSpeed - LastSpeed < -deltaTime ) {
        targetSpeed = LastSpeed - deltaTime;
    }
    LastSpeed = targetSpeed;

    if ( TotalPathLength > 0.0f ) {
        CurrentU += ( targetSpeed * deltaTime ) / TotalPathLength;
    }
    if ( CurrentU >= 1.0f ) {
        CurrentU = 0.0f;
    }

    for ( uint32_t step = 0; step < NumPathElements * 2; step++ ) {
        const BoatPathEntry *pElement = pPathElements[elementID];
        if ( pElement == nullptr ) {
            return;
        }

        switch ( pElement->Type ) {
        case BoatPathEntry::PET_Curve:
            if ( CurrentU <= PathElemensU[elementID + 1] &&
                 PathElemensU[elementID] <= CurrentU ) {
                return;
            }
            break;
        case BoatPathEntry::PET_Waypoint:
        case BoatPathEntry::PET_Reverse:
            break;
        case BoatPathEntry::PET_Wait:
            if ( TimeInThisState <= pElement->getWaitDuration() ) {
                return;
            }
            break;
        default:
            return;
        }

        const uint32_t followingID = ( nextID + 1 ) % NumPathElements;
        CurrentPathElementID = nextID;
        elementID = nextID;
        nextID = followingID;
        TimeInThisState = 0.0f;

        if ( pPathElements[elementID] != nullptr &&
             pPathElements[elementID]->Type == BoatPathEntry::PET_Wait ) {
            LastSpeed = 0.0f;
        }
    }
}

void BoatPathInstance::evaluate( Eigen::Vector4f &outPosition,
                                 Eigen::Vector4f &outTangent ) const {
    // FUN_008ea200
    if ( NumPathElements == 0 ) {
        return;
    }

    const uint32_t elementID = CurrentPathElementID;
    uint32_t nextID = ( elementID + 1 ) % NumPathElements;

    if ( pPathElements[elementID]->Type == BoatPathEntry::PET_Wait ) {
        // Idling on a marker: sit at the head of the curve leaving it.
        if ( pPathElements[nextID] == nullptr ) {
            return;
        }
        const CubicBezier curve = MakeCurve( *pPathElements[nextID] );
        outPosition = curve.evaluate( 0.0f );
        outTangent = curve.evaluateTangent( 0.0f );
        nextID = ( nextID + 1 ) % NumPathElements;
    } else {
        const float span =
            PathElemensU[elementID + 1] - PathElemensU[elementID];
        const float t =
            span > 0.0f ? ( CurrentU - PathElemensU[elementID] ) / span : 0.0f;

        const CubicBezier curve = MakeCurve( *pPathElements[elementID] );
        outPosition = curve.evaluate( t );
        outTangent = curve.evaluateTangent( t );
    }

    if ( pPathElements[nextID] != nullptr &&
         pPathElements[nextID]->Type == BoatPathEntry::PET_Reverse ) {
        outTangent = -outTangent;
    }
}

Entity *BoatPathTemplate::getBoat( uint32_t index ) const {
    if ( index >= travels.size() ) {
        return nullptr;
    }
    return travels[index]->pBoatInstance;
}

void BoatPathTemplate::initialize( EntityBoatPath *pEntity ) {
    // FUN_008eb030
    pPathEntity = pEntity;
    travels.clear();

    Eigen::Vector4f position = Eigen::Vector4f( 0.0f, 0.0f, 0.0f, 1.0f );

    const int32_t numWaypoints = pEntity->getWaypointCount();
    for ( int32_t i = 0; i < numWaypoints; i++ ) {
        pEntity->getWaypointPosition( i, position );
        path.addWaypoint( position, pEntity->getWaypointMode( i ),
                          pEntity->getWaypointParameter( i ) );
    }

    path.build( nullptr, pEntity->isLooping() );
}

void BoatPathTemplate::spawnBoats( MngActor &mngActor ) {
    // FUN_008eb0b0
    if ( pPathEntity == nullptr || gpScenaric == nullptr ||
         gpBoat == nullptr ) {
        return;
    }

    RandomGen &randomGen = gpScenaric->getRandomGenerator();

    const float minSpacing = pPathEntity->getMinBoatSpacing();
    const float spacingRange = pPathEntity->getMaxBoatSpacing() - minSpacing;
    const float pathLength = path.getTotalLength();
    const int32_t numWaypoints = pPathEntity->getWaypointCount();

    float distance = RandomUnit( randomGen ) * spacingRange + minSpacing;
    while ( travels.size() < kMaxBoatsPerPath &&
            distance < pathLength - minSpacing ) {
        Entity *pTemplate =
            gpBoat->pickBoatModel( pPathEntity->getBoatGroupIndex() );
        if ( pTemplate == nullptr ) {
            return;
        }

        travels.push_back( std::make_unique<BoatState>() );
        BoatState &travel = *travels.back();

        Entity *pBoat = mngActor.instanciateActor( pTemplate );
        if ( pBoat != nullptr ) {
            // Paths are jittered so that boats sharing a path do not line up
            for ( int32_t i = 0; i < numWaypoints; i++ ) {
                const Matrix4x4f_t *pVolume =
                    pPathEntity->getWaypointMatrix( i );
                if ( pVolume == nullptr ) {
                    continue;
                }

                Eigen::Vector4f jitter;
                jitter.x() = RandomUnit( randomGen ) - 0.5f;
                jitter.y() = RandomUnit( randomGen ) - 0.5f;
                jitter.z() = RandomUnit( randomGen ) - 0.5f;
                jitter.w() = 1.0f;

                Eigen::Vector4f position = TDUMat4x4MulVec( *pVolume, jitter );
                position.y() = ( *pVolume )( 3, 1 );

                travel.Path.addWaypoint(
                    position, pPathEntity->getWaypointMode( i ),
                    pPathEntity->getWaypointParameter( i ) );
            }

            travel.Path.build( &path, pPathEntity->isLooping() );
            travel.pBoatInstance = pBoat;
            travel.Path.setSpeed( pPathEntity->getBoatSpeed() );
            travel.Path.seek( pathLength > 0.0f ? ( distance / pathLength )
                                                : 0.0f );
        }

        distance += RandomUnit( randomGen ) * spacingRange + minSpacing;
    }
}

void BoatPathTemplate::tick( float deltaTime ) {
    // FUN_008ead20
    for ( std::unique_ptr<BoatState> &pTravel : travels ) {
        BoatState &travel = *pTravel;
        travel.Path.update( deltaTime );

        if ( travel.pBoatInstance == nullptr ) {
            continue;
        }

        Eigen::Vector4f position = Eigen::Vector4f( 0.0f, 0.0f, 0.0f, 1.0f );
        Eigen::Vector4f tangent = Eigen::Vector4f::Zero();
        travel.Path.evaluate( position, tangent );

        // Build the boat frame from the path tangent and the world up axis.
        const float tangentLength = tangent.head<3>().norm();
        if ( tangentLength <= 0.0f ) {
            continue;
        }
        const Eigen::Vector3f forward = tangent.head<3>() / tangentLength;

        Eigen::Vector3f right = kWorldUpVector.head<3>().cross( forward );
        const float rightLength = right.norm();
        if ( rightLength <= 0.0f ) {
            continue;
        }
        right /= rightLength;

        Eigen::Vector3f up = forward.cross( right );
        const float upLength = up.norm();
        if ( upLength <= 0.0f ) {
            continue;
        }
        up /= upLength;

        Matrix4x4f_t boatMatrix = Matrix4x4f_t::Identity();
        boatMatrix.row( 0 ) << right.x(), right.y(), right.z(), 0.0f;
        boatMatrix.row( 1 ) << up.x(), up.y(), up.z(), 0.0f;
        boatMatrix.row( 2 ) << forward.x(), forward.y(), forward.z(), 0.0f;
        boatMatrix.row( 3 ) << position.x(), position.y(), position.z(), 1.0f;

        travel.pBoatInstance->setTransform( boatMatrix, true );
    }
}

void BoatPathTemplate::refreshHullRenderMode( bool bMapView ) {
    // FUN_008e9ea0
    for ( std::unique_ptr<BoatState> &pTravel : travels ) {
        Entity *pBoat = pTravel->pBoatInstance;
        if ( pBoat == nullptr ) {
            continue;
        }

        Instance *pHull =
            static_cast<EntityBoat *>( pBoat )->getRenderInstance();
        if ( pHull == nullptr || pHull->getNumLODs() <= 1 ) {
            continue;
        }

        if ( bMapView ) {
            // The map camera sits far above the island: stop the per-frame LOD
            // selection and pin the hull to its second LOD.
            pHull->getFlagsWrite() &= ~kInstanceAutoLODFlag;
            pHull->setActiveLODIndex( 1 );
        } else {
            pHull->getFlagsWrite() |= kInstanceAutoLODFlag;
        }
    }
}

GSBoat::GSBoat()
    : GameSystem(),
      mngActor(),
      pathElementPool( nullptr ),
      numUsedPathElements( 0 ),
      pAudioClient( nullptr ),
      pHornSoundInstance( nullptr ),
      pHornBoat( nullptr ),
      timeSinceLastHorn( 0.0f ),
      hornInterval( 0.0f ),
      bBoatsSpawned( false ) {
    // FUN_008e8fb0
    OTDU_ASSERT( gpBoat == nullptr );
    gpBoat = this;
}

GSBoat::~GSBoat() {
    // FUN_008e8f80
}

bool GSBoat::initialize( TestDriveGameInstance *param_2 ) {
    // FUN_008e9be0
    pGameInstance = param_2;
    bPaused = false;

    originalPaths.clear();
    timeSinceLastHorn = 0.0f;
    hornInterval = ComputeHornInterval();

    pHornBoat = nullptr;
    if ( gpAudio != nullptr ) {
        pAudioClient = gpAudio->createClient( "ClientBoat" );
    }
    if ( pAudioClient != nullptr ) {
        pHornSoundInstance = pAudioClient->acquireSoundInstance();
    }
    if ( pHornSoundInstance != nullptr ) {
        pHornSoundInstance->setLoopCount( 0 );
        pHornSoundInstance->setVolume( 1.0f );
        pHornSoundInstance->setClipDistances( kHornClipNear, kHornClipFar );
        pHornSoundInstance->getNextValues().MinVolume = kHornMinVolume;
    }

    mngActor.initialize( param_2 );
    mngActor.reset();

    pathElementPool = std::make_unique<BoatPathEntry[]>( kPathElementPoolSize );
    numUsedPathElements = 0;

    return true;
}

void GSBoat::spawnBoats() {
    static const bool bOtduOff = getenv( "OTDU_NO_BOATS" ) != nullptr;
    if ( bOtduOff ) {
        return;
    }
    // FUN_008e9be0
    if ( bBoatsSpawned || gpScenaric == nullptr ) {
        return;
    }
    bBoatsSpawned = true;

    if ( pathElementPool == nullptr ) {
        pathElementPool =
            std::make_unique<BoatPathEntry[]>( kPathElementPoolSize );
        numUsedPathElements = 0;
    }

    buildBoatModelGroups();

    ScenaricBank *pPathBank = gpScenaric->getBoatsPathsBank();
    if ( pPathBank == nullptr ) {
        return;
    }

    int32_t numPathEntities = 0;
    for ( int32_t i = 0; i < pPathBank->getEntityCount(); i++ ) {
        Entity *pEntity = pPathBank->getEntity( i );
        if ( pEntity == nullptr ||
             pEntity->getType() != eEntityType::ET_BOPA ) {
            continue;
        }
        numPathEntities++;

        EntityBoatPath *pPathEntity = static_cast<EntityBoatPath *>( pEntity );
        if ( pPathEntity->getWaypointCount() <= 2 ||
             originalPaths.size() >= kMaxOriginalPaths ) {
            continue;
        }

        originalPaths.push_back( std::make_unique<BoatPathTemplate>() );
        originalPaths.back()->initialize( pPathEntity );
    }

    int32_t numBoats = 0;
    for ( std::unique_ptr<BoatPathTemplate> &pPath : originalPaths ) {
        pPath->spawnBoats( mngActor );
        numBoats += static_cast<int32_t>( pPath->getNumBoats() );
    }

    OTDU_LOG_DEBUG(
        "Spawned %d boat(s) on %d of %d path(s); boat models per group: "
        "%d/%d/%d; %d path element(s) used\n",
        numBoats, static_cast<int32_t>( originalPaths.size() ), numPathEntities,
        static_cast<int32_t>( boatModelGroups[0].numModels ),
        static_cast<int32_t>( boatModelGroups[1].numModels ),
        static_cast<int32_t>( boatModelGroups[2].numModels ),
        static_cast<int32_t>( numUsedPathElements ) );
}

void GSBoat::tick( float totalTime, float deltaTime ) {
    static const bool bOtduOff = getenv( "OTDU_NO_BOATS" ) != nullptr;
    if ( bOtduOff ) {
        return;
    }
    // FUN_008e98f0
    (void)totalTime;

    for ( std::unique_ptr<BoatPathTemplate> &pPath : originalPaths ) {
        pPath->tick( deltaTime );
    }

#ifdef OTDU_USE_IMGUI
    if ( gFollowFocusedBoat ) {
        FocusCameraOnBoat( getBoat( static_cast<uint32_t>( gFocusedBoat ) ),
                           false );
    }
#endif

    timeSinceLastHorn += deltaTime;
    if ( hornInterval <= timeSinceLastHorn ) {
        const Eigen::Vector3f &listenerPosition =
            gEdAudio.getListener().Position;
        const Eigen::Vector4f listener( listenerPosition.x(), 0.0f,
                                        listenerPosition.z(), 1.0f );

        pHornBoat = nullptr;
        const edAudioSample *pHornSample = nullptr;
        float bestDistanceSq = kMaxHornDistanceSq;

        const uint32_t numBoats = getNumBoats();
        for ( uint32_t i = 0; i < numBoats; i++ ) {
            Entity *pBoat = getBoat( i );
            if ( pBoat == nullptr ) {
                continue;
            }

            const edAudioSample *pSample =
                static_cast<EntityBoat *>( pBoat )->getHornSample();
            if ( pSample == nullptr ) {
                continue;
            }

            const Matrix4x4f_t &modelMatrix = pBoat->getModelMatrix();
            const Eigen::Vector4f boatPosition( modelMatrix( 3, 0 ), 0.0f,
                                                modelMatrix( 3, 2 ),
                                                modelMatrix( 3, 3 ) );
            const float distanceSq = ( boatPosition - listener ).squaredNorm();
            if ( distanceSq < bestDistanceSq ) {
                bestDistanceSq = distanceSq;
                pHornBoat = pBoat;
                pHornSample = pSample;
            }
        }

        if ( pHornBoat != nullptr ) {
            if ( pHornSoundInstance != nullptr ) {
                const Matrix4x4f_t &modelMatrix = pHornBoat->getModelMatrix();
                pHornSoundInstance->setSample( pHornSample );
                pHornSoundInstance->setPosition(
                    modelMatrix.row( 3 ).head<3>(),
                    AudioDispatcherPlaybackInstanceValues::ADPCS_World );
                pHornSoundInstance->setVolume( 1.0f );
                pHornSoundInstance->setPitch(
                    static_cast<float>( rand() & 0x7fff ) * kRandNorm *
                        kHornPitchRange +
                    kHornPitchBase );
                pHornSoundInstance->play( kHornFadeIn );
            }

            timeSinceLastHorn = 0.0f;
            hornInterval = ComputeHornInterval();
        }
    }

    // Keep the instance following the boat that is sounding.
    if ( pHornBoat != nullptr && pHornSoundInstance != nullptr ) {
        const Matrix4x4f_t &modelMatrix = pHornBoat->getModelMatrix();
        pHornSoundInstance->setPosition(
            modelMatrix.row( 3 ).head<3>(),
            AudioDispatcherPlaybackInstanceValues::ADPCS_World );
        pHornSoundInstance->setVolume( 1.0f );
    }
}

void GSBoat::setEcumeEnabled( bool bEnabled ) {
    // FUN_008e8ea0
    for ( std::unique_ptr<BoatPathTemplate> &pPath : originalPaths ) {
        pPath->refreshHullRenderMode( bEnabled );
    }

    const uint32_t numBoats = getNumBoats();
    for ( uint32_t i = 0; i < numBoats; i++ ) {
        Entity *pBoat = getBoat( i );
        if ( pBoat != nullptr ) {
            static_cast<EntityBoat *>( pBoat )->setRenderEcume( bEnabled );
        }
    }
}

void GSBoat::draw( float deltaTime ) {
    static const bool bOtduOff = getenv( "OTDU_NO_BOATS" ) != nullptr;
    if ( bOtduOff ) {
        return;
    }
    // FUN_008e9790
    (void)deltaTime;
    mngActor.draw();

    OTDU_UNIMPLEMENTED_ONCE; // TODO: DEBUG (support BOAT TRACE)
}

void GSBoat::terminate() {
    // FUN_008e97b0
    originalPaths.clear();
    pHornBoat = nullptr;

    if ( pHornSoundInstance != nullptr ) {
        pHornSoundInstance->stop( 0.0f );
        pHornSoundInstance->release();
        pHornSoundInstance = nullptr;
    }
    pAudioClient = nullptr;

    pathElementPool.reset();
    numUsedPathElements = 0;

    mngActor.terminate();
    mngActor.reset();

    for ( BoatModelGroup &group : boatModelGroups ) {
        group.numModels = 0;
    }

    bBoatsSpawned = false;
}

BoatPathEntry *GSBoat::allocatePathElement() {
    if ( pathElementPool == nullptr ||
         numUsedPathElements >= kPathElementPoolSize ) {
        OTDU_ASSERT_ONCE( numUsedPathElements < kPathElementPoolSize );
        return nullptr;
    }

    BoatPathEntry *pElement = &pathElementPool[numUsedPathElements++];
    *pElement = BoatPathEntry();
    return pElement;
}

uint32_t GSBoat::getNumBoats() const {
    uint32_t numBoats = 0;
    for ( const std::unique_ptr<BoatPathTemplate> &pPath : originalPaths ) {
        numBoats += pPath->getNumBoats();
    }
    return numBoats;
}

Entity *GSBoat::getBoat( uint32_t index ) const {
    // FUN_008e8e00
    uint32_t pathIndex = 0;
    uint32_t firstBoatOfPath = 0;
    while ( pathIndex < originalPaths.size() ) {
        const uint32_t nextPathFirstBoat =
            firstBoatOfPath + originalPaths[pathIndex]->getNumBoats();
        if ( index < nextPathFirstBoat ) {
            break;
        }
        pathIndex++;
        firstBoatOfPath = nextPathFirstBoat;
    }

    if ( pathIndex >= originalPaths.size() ) {
        return nullptr;
    }
    return originalPaths[pathIndex]->getBoat( index - firstBoatOfPath );
}

Entity *GSBoat::pickBoatModel( int32_t groupIndex ) {
    if ( gpScenaric == nullptr || groupIndex < 0 ||
         groupIndex >= static_cast<int32_t>( kNumBoatGroups ) ) {
        return nullptr;
    }

    BoatModelGroup &group = boatModelGroups[groupIndex];
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

void GSBoat::buildBoatModelGroups() {
    // FUN_009c65e0
    for ( BoatModelGroup &group : boatModelGroups ) {
        group.numModels = 0;
    }

    ScenaricBank *pBoatBank = gpScenaric->getBoatBank();
    if ( pBoatBank == nullptr ) {
        return;
    }

    for ( int32_t i = 0; i < pBoatBank->getEntityCount(); i++ ) {
        Entity *pEntity = pBoatBank->getEntity( i );
        if ( pEntity == nullptr ||
             pEntity->getType() != eEntityType::ET_BOAT ) {
            continue;
        }

        const int32_t boatType =
            static_cast<EntityBoat *>( pEntity )->getBoatType();
        if ( boatType < 0 ||
             boatType >= static_cast<int32_t>( kNumBoatGroups ) ) {
            continue;
        }

        BoatModelGroup &group = boatModelGroups[boatType];
        if ( group.numModels < kMaxBoatModelsPerGroup ) {
            group.pModels[group.numModels++] = pEntity;
        }
    }
}

#ifdef OTDU_USE_IMGUI
void GSBoat::drawDebug( float deltaTime ) {
    (void)deltaTime;

    ImGui::SeparatorText( "Boat paths" );
    ImGui::Text( "%d path(s) | %d boat(s) | %d/%d path elements | models per "
                 "group %d/%d/%d",
                 static_cast<int32_t>( originalPaths.size() ),
                 static_cast<int32_t>( getNumBoats() ),
                 static_cast<int32_t>( numUsedPathElements ),
                 static_cast<int32_t>( kPathElementPoolSize ),
                 static_cast<int32_t>( boatModelGroups[0].numModels ),
                 static_cast<int32_t>( boatModelGroups[1].numModels ),
                 static_cast<int32_t>( boatModelGroups[2].numModels ) );

    if ( originalPaths.empty() ) {
        ImGui::TextDisabled(
            "world_boats has not been parsed yet (waiting on the world)." );
        return;
    }

    const int32_t numBoats = static_cast<int32_t>( getNumBoats() );
    if ( numBoats == 0 ) {
        ImGui::TextDisabled( "No boat spawned on any path." );
        return;
    }

    ImGui::SeparatorText( "Camera focus" );

#ifdef OTDU_DEVBUILD
    // Without the free-fly camera the game camera recomputes its transform
    // every tick and snaps straight back off the boat.
    if ( gpCamera != nullptr ) {
        bool bFreeCam = gpCamera->isFreeCamEnabled();
        if ( ImGui::Checkbox( "Free-fly camera", &bFreeCam ) ) {
            gpCamera->toggleFreeCam();
        }
    }
#endif

    if ( gFocusedBoat >= numBoats ) {
        gFocusedBoat = 0;
    }
    ImGui::SliderInt( "Boat", &gFocusedBoat, 0, numBoats - 1 );
    ImGui::DragFloat( "Distance", &gFocusDistance, 0.5f, 2.0f, 500.0f );

    Entity *pBoat = getBoat( static_cast<uint32_t>( gFocusedBoat ) );
    if ( pBoat == nullptr ) {
        ImGui::TextDisabled( "Empty boat slot (instanciation failed)." );
        return;
    }

    EntityBoat *pBoatEntity = static_cast<EntityBoat *>( pBoat );
    const Matrix4x4f_t &modelMatrix = pBoat->getModelMatrix();
    ImGui::Text( "Position %.1f %.1f %.1f | model group %d | hull instance: %s",
                 modelMatrix( 3, 0 ), modelMatrix( 3, 1 ), modelMatrix( 3, 2 ),
                 pBoatEntity->getBoatType(),
                 pBoatEntity->hasRenderInstance() ? "bound" : "MISSING" );

    if ( ImGui::Button( "Focus camera" ) ) {
        FocusCameraOnBoat( pBoat, true );
    }
    ImGui::SameLine();
    if ( ImGui::Button( "Next boat" ) ) {
        gFocusedBoat = ( gFocusedBoat + 1 ) % numBoats;
        FocusCameraOnBoat( getBoat( static_cast<uint32_t>( gFocusedBoat ) ),
                           true );
    }
    ImGui::SameLine();
    ImGui::Checkbox( "Follow", &gFollowFocusedBoat );

    ImGui::TextDisabled( "'Focus camera' also recenters world streaming so the "
                         "sea around the boat loads; 'Follow' only moves the "
                         "camera, and tracks the boat as it sails." );

    ImGui::SeparatorText( "Map-screen mode" );
    if ( ImGui::Checkbox( "Wake foam + pinned hull LOD (GMMap look)",
                          &gRenderEcume ) ) {
        setEcumeEnabled( gRenderEcume );
    }

    ImGui::SeparatorText( "Horn" );
    ImGui::Text( "Audio client: %s | sound instance: %s | next blast in %.1fs",
                 pAudioClient != nullptr ? "ok" : "MISSING",
                 pHornSoundInstance != nullptr ? "ok" : "MISSING",
                 hornInterval - timeSinceLastHorn );
    ImGui::Text( "Horn sample on this boat: %s",
                 pBoatEntity->getHornSample() != nullptr ? "loaded" : "none" );
    if ( ImGui::Button( "Sound horn now" ) ) {
        timeSinceLastHorn = hornInterval;
    }
}
#endif
