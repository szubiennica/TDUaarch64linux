#include "shared.h"
#include "gs_particles.h"

#include "config/gs_config.h"

#include "render/camera/camera_base.h"
#include "render/frustum.h"
#include "render/gs_render.h"
#include "render/material.h"
#include "render/render_context.h"
#include "render/render_scene.h"

#include "world/gs_world_streaming.h"
#include "world/weather/gs_weather.h"
#include "world/weather/weather_config.h"

#include <cstring>

GSParticles *gpParticles = nullptr; // DAT_010e7c24

struct PrtDescriptor {
    const char *pName;
    eParticleType id;
    int32_t instanceCount;
    uint8_t alphaReject;
    uint8_t coverageBudget;
};

// DAT_00f8aea8
static constexpr const PrtDescriptor kPrtDescriptors[] = {
    {"PRT_BURN",        eParticleType::PT_Burn,      10, 32, 35},
    {"PRT_SMOKE",       eParticleType::PT_Smoke,     10, 32, 40},
    {"PRT_START",       eParticleType::PT_StartLine, 10, 0,  0 },
    {"PRT_DUST",        eParticleType::PT_Dust,      10, 16, 50},
    {"PRT_GRASS",       eParticleType::PT_Grass,     10, 0,  0 },
    {"PRT_GRAVEL",      eParticleType::PT_Gravel,    10, 0,  0 },
    {"PRT_WOOD",        eParticleType::PT_Wood,      4,  0,  0 },
    {"PRT_FEUILLE",     eParticleType::PT_Leaf,      4,  0,  0 },
    {"PRT_FLYING_LEAF", eParticleType::PT_Leaf2,     4,  0,  0 },
    {"PRT_BUMPER",      eParticleType::PT_CarBody,   4,  0,  0 },
    {"PRT_GLASS",       eParticleType::PT_Glass,     4,  0,  0 },
    {"PRT_PLASTIC",     eParticleType::PT_Plastic,   4,  0,  0 },
    {"PRT_STONE",       eParticleType::PT_Stone,     4,  0,  0 },
    {"PRT_METAL",       eParticleType::PT_Metal,     4,  0,  0 },
    {"PRT_FACTORY",     eParticleType::PT_Factory,   8,  32, 40},
    {"PRT_OLD",         eParticleType::PT_Old,       4,  0,  0 },
};

static uint8_t OtduCoverage( uint8_t v ) {
    static const bool bLow = getenv( "OTDU_PARTICLE_QUALITY" ) != nullptr;
    if ( !bLow || v == 0 ) {
        return v;
    }
    const int32_t scaled = v / 2;
    return (uint8_t)( scaled < 5 ? 5 : scaled );
}

static uint8_t OtduAlphaReject( uint8_t v ) {
    static const bool bLow = getenv( "OTDU_PARTICLE_QUALITY" ) != nullptr;
    if ( !bLow || v == 0 ) {
        return v;
    }
    const int32_t scaled = v + v / 2;
    return (uint8_t)( scaled > 255 ? 255 : scaled );
}

const PrtDescriptor *FindDescriptor( const char *pName ) {
    // FUN_0080c8d0
    std::string upper( pName );
    for ( char &c : upper ) {
        c = static_cast<char>( std::toupper( (unsigned char)c ) );
    }
    for ( const PrtDescriptor &descriptor : kPrtDescriptors ) {
        if ( std::strcmp( descriptor.pName, upper.c_str() ) == 0 ) {
            return &descriptor;
        }
    }
    return nullptr;
}

// A bank entry's basename: drop the path and extension, cap at 31 chars.
std::string ParticleFileToName( const std::string &filename ) {
    size_t start = filename.find_last_of( "/\\:" );
    start = ( start == std::string::npos ) ? 0 : start + 1;
    const size_t dot = filename.find( '.', start );
    const size_t length =
        ( dot == std::string::npos ) ? std::string::npos : dot - start;
    std::string name = filename.substr( start, length );
    if ( name.size() > 31 ) {
        name.resize( 31 );
    }
    return name;
}

GSParticles::GSParticles() : GameSystem() {
    // FUN_0080d620
    OTDU_ASSERT( gpParticles == nullptr );
    gpParticles = this;
}

GSParticles::~GSParticles() { gpParticles = nullptr; }

bool GSParticles::initialize( TestDriveGameInstance * ) {
    // FUN_0080dcc0
    buildRegistry();
    return true;
}

void GSParticles::tick( float, float deltaTime ) {
    // FUN_0080d0e0
    for ( Particle *pParticle : particles ) {
        if ( pParticle->SimuType == eParticleSimulationType::PST_Sparkle ) {
            SimulateSparkles( pParticle, deltaTime );
        } else {
            simulateParticle( pParticle, deltaTime );
        }
    }
}

void GSParticles::draw( float ) {
    // FUN_0080d0b0
    for ( Particle *pParticle : particles ) {
        submitParticle( pParticle );
    }
}

void GSParticles::terminate() {
    // FUN_0080d6e0
    stopAllParticleSystems();
    for ( Particle *pParticle : particles ) {
        for ( ParticleInstance *pInstance : pParticle->Instances ) {
            delete pInstance;
        }
        delete pParticle;
    }
    particles.clear();
    memoryUsed = 0;

    for ( int32_t i = 0; i < num2DM; i++ ) {
        collection2D.unregister2DM( &render2DMs[i] );
        render2DMs[i].destroy();
    }
    for ( int32_t i = 0; i < num2DB; i++ ) {
        collection2D.unregister2DB( &render2DBs[i] );
        render2DBs[i].destroy();
    }
    num2DM = 0;
    num2DB = 0;

    if ( bank.isLoaded() ) {
        bank.unload();
    }
}

ParticleInstance *GSParticles::activateParticles( eParticleType id,
                                                  bool bReset ) {
    // FUN_00413c50
    Particle *pParticle = findByID( id );
    return ( pParticle != nullptr ) ? pParticle->activateParticles( bReset )
                                    : nullptr;
}

ParticleInstance *GSParticles::activateParticles( uint64_t hashcode,
                                                  bool bReset ) {
    // FUN_004147e0
    Particle *pParticle = findByHashcode( hashcode );
    return ( pParticle != nullptr ) ? pParticle->activateParticles( bReset )
                                    : nullptr;
}

void GSParticles::stopAllParticleSystems() {
    // FUN_0080d330
    for ( Particle *pParticle : particles ) {
        for ( ParticleInstance *pInstance : pParticle->Instances ) {
            pInstance->State = eParticleState::PS_Dead;
        }
    }
}

void GSParticles::resetInstancesInState( eParticleState state ) {
    // FUN_0080d2c0
    for ( Particle *pParticle : particles ) {
        for ( ParticleInstance *pInstance : pParticle->Instances ) {
            if ( pInstance->State == state ) {
                pInstance->State = eParticleState::PS_Dead;
            }
        }
    }
}

Particle *GSParticles::findByID( eParticleType id ) {
    for ( Particle *pParticle : particles ) {
        if ( pParticle->Type == id ) {
            return pParticle;
        }
    }
    return nullptr;
}

Particle *GSParticles::findByHashcode( uint64_t hashcode ) {
    // FUN_00414810
    for ( Particle *pParticle : particles ) {
        if ( pParticle->Hashcode == hashcode ) {
            return pParticle;
        }
    }
    return nullptr;
}

void GSParticles::buildRegistry() {
    // FUN_0080d750
    std::string path = gpConfig->getResRootPath();
    path += "FX/particles.bnk";
    bank.loadBootBank( path.c_str(), nullptr );
    if ( !bank.isLoaded() ) {
        return;
    }

    loadMaterials();

    const int32_t numEntries = bank.getNumElements();
    for ( int32_t i = 0; i < numEntries && particles.size() < 32; i++ ) {
        ParsedBankEntry entry;
        bank.getEntry( i, entry );
        if ( entry.pData == nullptr || entry.Size <= 0 ) {
            continue;
        }
        const uint8_t *pData = static_cast<const uint8_t *>( entry.pData );
        if ( !ParticleFile::IsParticleFile(
                 pData, static_cast<uint32_t>( entry.Size ) ) ) {
            continue;
        }
        particles.push_back(
            createParticleType( ParticleFileToName( entry.Filename ), pData,
                                static_cast<uint32_t>( entry.Size ) ) );
    }

    if ( particles.size() < 32 ) {
        Particle *pSparkles = new Particle();
        pSparkles->Name = "SPARKLES";
        pSparkles->Hashcode = GetIdentifier64bit( "SPARKLES" );
        pSparkles->Type = eParticleType::PT_Sparkle;
        pSparkles->SimuType = eParticleSimulationType::PST_Sparkle;
        particles.push_back( pSparkles );
        InitializeSparkles( pSparkles, memoryUsed, kParticleMemorySize );
    }
}

void GSParticles::loadMaterials() {
    if ( !bank.isLoaded() ) {
        return;
    }

    uint32_t resSize = 0;
    void *pEntry = bank.getFirstEntry( AG_FX, AT_2DB, &resSize );
    while ( num2DB < kMaxParticle2D && pEntry != nullptr ) {
        if ( render2DBs[num2DB].initialize( pEntry ) ) {
            collection2D.register2DB( &render2DBs[num2DB] );
            num2DB++;
        }
        pEntry = bank.getNextEntry( AG_FX, AT_2DB, &resSize );
    }

    pEntry = bank.getFirstEntry( AG_FX, AT_2DM, &resSize );
    while ( num2DM < kMaxParticle2D && pEntry != nullptr ) {
        if ( render2DMs[num2DM].initialize( pEntry ) ) {
            collection2D.register2DM( &render2DMs[num2DM] );
            num2DM++;
        }
        pEntry = bank.getNextEntry( AG_FX, AT_2DM, &resSize );
    }

    for ( int32_t i = 0; i < num2DM; i++ ) {
        collection2D.bindBitmapReferences( &render2DMs[i] );
    }

    if ( gpRenderContext != nullptr ) {
        for ( int32_t i = 0; i < num2DM; i++ ) {
            for ( uint32_t m = 0;; m++ ) {
                Material *pMaterial = render2DMs[i].getMaterialAtIndex( m );
                if ( pMaterial == nullptr ) {
                    break;
                }
                if ( pMaterial->NumParams == 0 ) {
                    continue;
                }
                MaterialParameter *pParam = pMaterial->getParameterByIndex( 0 );

                constexpr uint64_t kParticleParamGUID = 0x73078865ca37336dull;
                if ( pParam->Hashcode == kParticleParamGUID ) {
                    pMaterial->StencilEnable = 0;
                    pMaterial->AlphaTest = eComparisonFunction::CF_Never;
                    pMaterial->AlphaRef = 4;
                    pMaterial->DepthWrite = 2;
                    pMaterial->DepthTest = 1;
                }

                pParam->Flags = MaterialParameterFlags{};
                pParam->Flags.NumUVMaps = 1;
                pParam->Flags.NumTextures = 1;
                pParam->Flags.bUsingNormals = 0;
                pParam->Flags.NumPrelight = 1;
                gpRenderContext->retrieveShadersForMaterial( pMaterial, 0 );
            }
        }
    }
}

Material *GSParticles::resolveParticleMaterial( uint64_t hashcode ) {
    for ( int32_t i = 0; i < num2DM; i++ ) {
        RenderFile::Section *pSection = render2DMs[i].getMaterial( hashcode );
        if ( pSection != nullptr ) {
            return reinterpret_cast<Material *>( pSection + 1 );
        }
    }
    return nullptr;
}

Particle *GSParticles::createParticleType( const std::string &name,
                                           const uint8_t *pData,
                                           uint32_t size ) {
    // FUN_0080ca40
    const PrtDescriptor *pDescriptor = FindDescriptor( name.c_str() );
    int32_t instanceCount =
        pDescriptor != nullptr ? pDescriptor->instanceCount : 4;
    if ( getenv( "OTDU_PARTICLE_QUALITY" ) != nullptr && instanceCount > 4 ) {
        instanceCount = 4;
    }

    Particle *pParticle = new Particle();
    pParticle->Name = name;
    pParticle->Hashcode = GetIdentifier64bit( name.c_str() );
    pParticle->Type =
        pDescriptor != nullptr ? pDescriptor->id : eParticleType::PT_Invalid;
    pParticle->SimuType = eParticleSimulationType::PST_Default;

    for ( int32_t i = 0; i < instanceCount; i++ ) {
        if ( i < instanceCount - 1 ) {
            if ( kParticleMemorySize - memoryUsed < size ) {
                break;
            }
            memoryUsed += size;
        }

        ParticleInstance *pInstance = new ParticleInstance();
        pInstance->pParticleInstanciated = pParticle;
        pInstance->State = eParticleState::PS_Dead;
        pInstance->Matrix = Matrix4x4f_t::Identity();
        pInstance->Color = Eigen::Vector4f::Zero();
        pInstance->Scale = 0.0f;
        pInstance->Density = 0.0f;
        pInstance->Speed = 0.0f;
        pInstance->StopTimer = 0.0f;
        pInstance->Alpha = 1.0f;
        pInstance->pPrimaryEmitter = nullptr;

        // Each instance parses its own private copy of the definition blob.
        pInstance->blob.assign( pData, pData + size );
        if ( !pInstance->file.parse(
                 pInstance->blob.data(),
                 static_cast<uint32_t>( pInstance->blob.size() ) ) ) {
            delete pInstance;
            break;
        }

        pInstance->file.reseed( static_cast<uint32_t>( rand() & 0x7fff ) );
        pInstance->pPrimaryEmitter = pInstance->file.findPrimaryDefinition();
        pInstance->file.applyMaterialDescriptor(
            pDescriptor != nullptr,
            pDescriptor != nullptr ? OtduAlphaReject( pDescriptor->alphaReject ) : 0,
            pDescriptor != nullptr ? OtduCoverage( pDescriptor->coverageBudget ) : 0 );
        pParticle->Instances.push_back( pInstance );
    }
    return pParticle;
}

void GSParticles::simulateParticle( Particle *pParticle, float deltaTime ) {
    // FUN_0080cd50
    const float timeScale = 1.0f; // TODO: Read time scale for replay!

    if ( pParticle->drawList.isInitialized() ) {
        pParticle->drawList.reset();
    }

    Eigen::Vector4f wind = Eigen::Vector4f::Zero();
    if ( gpWeather != nullptr ) {
        const WeatherConfig &config =
            gpWeather->getActiveRandomizedWeatherConfig();
        const Eigen::Vector4f direction = config.getWindDirection();
        const float speed = config.getWindSpeed();
        wind = Eigen::Vector4f( direction.x() * speed, 0.2f * speed,
                                direction.z() * speed, 0.0f );
    }

    const Eigen::Vector4f shiftPosition =
        ( gpWorldStreaming != nullptr && gpWorldStreaming->isEnabled() )
            ? gpWorldStreaming->getPosition()
            : Eigen::Vector4f::Zero();

    RenderScene *pScene =
        gpRender->getRenderPass<eRenderPass::RP_Near>().pScene;
    if ( pScene == nullptr || pScene->getCamera() == nullptr ) {
        return;
    }
    const Matrix4x4f_t &worldToCam = pScene->getCamera()->getWorldToCam();
    const Matrix4x4f_t &projection =
        pScene->getFrustumWrite().getProjectionMatrix();

    for ( ParticleInstance *pInstance : pParticle->Instances ) {
        if ( pInstance->State == eParticleState::PS_Dead ) {
            continue;
        }

        pInstance->file.updateInstance(
            wind, pInstance->pPrimaryEmitter, pInstance->Color,
            pInstance->Alpha, pInstance->Scale, pInstance->Density,
            pInstance->Speed, pInstance->Matrix, shiftPosition );

        pInstance->file.simulate( timeScale * deltaTime );

        if ( pInstance->State == eParticleState::PS_Killed ) {
            pInstance->StopTimer += deltaTime;
            if ( !pInstance->file.hasLiveParticles() ||
                 pInstance->StopTimer >= 8.0f ) { // DAT_00f41ac4
                pInstance->State = eParticleState::PS_Dead;
            }
        }

        pInstance->file.buildGeometry( worldToCam, projection,
                                       pParticle->drawList );
    }
}

void GSParticles::submitParticle( Particle *pParticle ) {
    // FUN_0080c9f0
    if ( !pParticle->drawList.isInitialized() ||
         pParticle->drawList.getNumPrimitives() == 0 ) {
        return;
    }
    gpRender->getRenderPass<eRenderPass::RP_Particles>()
        .pScene->enqueueDynamicDrawList( &pParticle->drawList );
}

#ifdef OTDU_USE_IMGUI
#include "imgui.h"

void GSParticles::drawDebug( float ) {
    ImGui::Text( "Bank loaded: %s", bank.isLoaded() ? "yes" : "no" );
    ImGui::Text( "Particle types: %d   2DM: %d   2DB: %d",
                 static_cast<int32_t>( particles.size() ), num2DM, num2DB );

    if ( particles.empty() ) {
        ImGui::TextDisabled(
            "No particle types loaded (is FX/particles.bnk present?)." );
        return;
    }

    static int32_t selectedType = 0;
    if ( selectedType >= static_cast<int32_t>( particles.size() ) ) {
        selectedType = 0;
    }

    ImGui::SeparatorText( "Spawn a test particle" );
    if ( ImGui::BeginCombo( "Type", particles[selectedType]->Name.c_str() ) ) {
        for ( int32_t i = 0; i < static_cast<int32_t>( particles.size() );
              i++ ) {
            const bool bSelected = ( i == selectedType );
            if ( ImGui::Selectable( particles[i]->Name.c_str(), bSelected ) ) {
                selectedType = i;
            }
            if ( bSelected ) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    static float color[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
    static float scale = 1.0f;
    static float speed = 1.0f;
    static float density = 1.0f;
    static float position[3] = { 0.0f, 0.0f, 0.0f };
    static bool bAtCamera = true;

    ImGui::ColorEdit4( "Colour", color );
    ImGui::DragFloat( "Scale", &scale, 0.1f, 0.0f, 100.0f );
    ImGui::DragFloat( "Speed", &speed, 0.1f, 0.0f, 100.0f );
    ImGui::DragFloat( "Density", &density, 0.1f, 0.0f, 100.0f );
    ImGui::Checkbox( "Spawn 5m in front of the camera", &bAtCamera );
    if ( !bAtCamera ) {
        ImGui::DragFloat3( "Position", position, 0.5f );
    }

    if ( ImGui::Button( "Spawn" ) ) {
        ParticleInstance *pInstance =
            activateParticles( particles[selectedType]->Hashcode, true );
        if ( pInstance != nullptr ) {
            Eigen::Vector3f worldPos( position[0], position[1], position[2] );
            if ( bAtCamera && gpRender != nullptr ) {
                RenderScene *pScene =
                    gpRender->getRenderPass<eRenderPass::RP_Near>().pScene;
                if ( pScene != nullptr && pScene->getCamera() != nullptr ) {
                    const Matrix4x4f_t &c2w =
                        pScene->getCamera()->getCamToWorld();
                    const Eigen::Vector3f camPos( c2w( 3, 0 ), c2w( 3, 1 ),
                                                  c2w( 3, 2 ) );
                    const Eigen::Vector3f forward( -c2w( 2, 0 ), -c2w( 2, 1 ),
                                                   -c2w( 2, 2 ) );
                    worldPos = camPos + forward * 5.0f;
                }
            }
            pInstance->Matrix = Matrix4x4f_t::Identity();
            pInstance->Matrix( 3, 0 ) = worldPos.x();
            pInstance->Matrix( 3, 1 ) = worldPos.y();
            pInstance->Matrix( 3, 2 ) = worldPos.z();
            pInstance->Color =
                Eigen::Vector4f( color[0], color[1], color[2], color[3] );
            pInstance->Scale = scale;
            pInstance->Speed = speed;
            pInstance->Density = density;
            pInstance->Alpha = color[3];
        }
    }
    ImGui::SameLine();
    if ( ImGui::Button( "Stop all" ) ) {
        stopAllParticleSystems();
    }

    int32_t activeInstances = 0;
    for ( Particle *pParticle : particles ) {
        for ( ParticleInstance *pInstance : pParticle->Instances ) {
            if ( pInstance->State != eParticleState::PS_Dead ) {
                activeInstances++;
            }
        }
    }
    ImGui::Text( "Active instances: %d", activeInstances );
}
#endif
