#include "shared.h"
#include "gs_render.h"

#include "config/gs_config.h"
#include "player_data/gs_playerdata.h"
#include "render_target.h"
#include "system/ed_sys.h"
#include "world/weather/gs_weather.h"
#include "world/weather/weather_config.h"

#ifdef OTDU_USE_IMGUI
#include "debug/gs_imgui.h"
#include "render/mng_shadow.h"
#include "render/camera/gs_camera.h"
#include "render/mng_env_map.h"
#include "world/gs_mng_resources.h"
#endif

#include "postfx/effects/postfx_blit.h"
#include "postfx/effects/postfx_downscale.h"
#include "postfx/effects/postfx_texture_sample.h"
#include "postfx/postfx_renderer.h"
#include "postfx/postfx_stack.h"
#include "shaders/shader_binary_cache.h"

#include "gs_timer.h"
#include "render/frame_graph.h"

#include "3dg.h"
#include "draw_list.h"
#include "ed_render.h"
#include "geometry_buffer.h"
#include "heightmap_renderer.h"
#include "instance.h"
#include "render_buckets.h"
#include "render_context.h"
#include "scene_renderer.h"
#include "state_cache.h"
#include "text_renderer.h"

#include "world/mng_background.h"
#include "world/mng_clouds_plane.h"
#include "world/mng_ocean.h"

#include "tdu_instance.h"

GSRender gRender = {};
GSRender *gpRender = nullptr;

eViewFormat gDepthStencilFormat = eViewFormat::VF_D24S8F; // DAT_00fac8e4
GPUTexture *gpMainDepthBuffer = nullptr;                  // DAT_00f47920
static float gUVATime = 0.0f;                             // DAT_016a2c14

static float FLOAT_00faa71c = 2.5f; // DAT_00faa71c
static float FLOAT_00faa720 = 1.0f; // DAT_00faa720
static float DebugLODFactor = 1.0f; // DAT_00f71ef0

// Used to calculate weather stuff (but appears to never be modified at
// runtime; could be some debug leftover)
static Eigen::Vector4f DAT_00fac360 = { 0.0f, 0.0f, 0.0f, 0.0f };

static constexpr BucketConfiguration kTDUBucketConfig = { // DAT_00f721c8
    0x2d, 0x1e, 0x3e8, 0x3a98, 0xe10, gRenderBuckets, Material::GetBucket };

static void FUN_00994850() {
    // FUN_00994850
    gpRender->notifyDeviceResetListeners();
}

GSRender::GSRender()
    : GameSystem(),
      activeAA( eAntiAliasingMethod::AAM_Disabled ),
      vsyncMode( eVSyncMode::VSM_Immediate ),
      activeLODQuality( 2u ),
      defaultViewport(),
      clearColorViewport(),
      clearViewport(),
      clearColorDepthViewport(),
      clearColorStencilViewport(),
      clearDepthViewport(),
      viewportBack(),
      viewportHUDMap(),
      viewportNoise(),
      viewportShadowClear(),
      viewportShadow(),
      viewportEnvmapBack(),
      viewportEnvmapNear(),
      viewportNear(),
      viewportParticles(),
      viewportSun(),
      viewportInterior(),
      viewportAvatar(),
      viewportHelmet(),
      viewport2D(),
      viewportLRFront(),
      viewportLRBack(),
      viewportCockpitAvatar(),
      layerBits( 0 ),
      renderWidth( 0 ),
      renderHeight( 0 ),
      frameIndex( 0 ),
      activeRenderLayers( 0 ),
      aspectRatio( 0.0f ),
      invAspectRatio( 0.0f ),
      bHDREnabled( false ),
      bWasHDREnabled( false ),
      bFovVsLodOn( true ),
      bInitialized( false ),
      mainRTwM(),
      scnDown4(),
      mainRT(),
      noiseComposite(),
      noiseAssembleF(),
      noiseAssembleS(),
      oceanNMap(),
      pMainRTwM( nullptr ),
      pMainRT( nullptr ),
      pScnDown4RT( nullptr ),
      pUnknownRT( nullptr ),
      pUnknownRT2( nullptr ),
      sun2DBs{ RuntimeRender2DB(), RuntimeRender2DB(), RuntimeRender2DB(),
               RuntimeRender2DB(), RuntimeRender2DB(), RuntimeRender2DB(),
               RuntimeRender2DB(), RuntimeRender2DB() },
      pSunRTs{ nullptr, nullptr, nullptr, nullptr,
               nullptr, nullptr, nullptr, nullptr },
      pNoiseCompositeRT( nullptr ),
      pNoiseAssembleFRT( nullptr ),
      pNoiseAssembleSRT( nullptr ),
      pOceanNMapRT( nullptr ),
      pPostFXMainHDR( nullptr ),
      pPostFXMainHDRDownscale( nullptr ),
      pSunAvgLum( nullptr ),
      pSunAvgLumCopy( nullptr ),
      pSunDazzleLUT( nullptr ),
      pSunDazzleLUTCopy( nullptr ),
      pSunBillboard( nullptr ),
      pSunDownscaleChain{ nullptr, nullptr, nullptr },
      camera2D(),
      cameraMain(),
      cameraLRFront(),
      cameraLRBack(),
      cameraShadow(),
      cameraShadowCockpit(),
      cameraEnvMapBack(),
      cameraEnvMap(),
      cameraReflection(),
      cameraHelmet(),
      shaderUniforms{ Eigen::Vector4f::Zero(), Eigen::Vector4f::Zero(),
                      Eigen::Vector4f::Zero(), Eigen::Vector4f::Zero(),
                      Eigen::Vector4f::Zero(), Eigen::Vector4f::Zero(),
                      Eigen::Vector4f::Zero(), Eigen::Vector4f::Zero() },
      defaultAmbientLight(),
      defaultDirectionalLight(),
      lightsDefault(),
      sceneNear(),
      scenePreNear(),
      sceneNearOpaque(),
      sceneCarPlayerPrepass(),
      sceneNearPrePassVegetation(),
      sceneParticles(),
      sceneCarPlayer(),
      sceneNearOpaqueCarAlpha(),
      sceneCarPlayerCarAlpha(),
      sceneNearAfterCarAlpha(),
      sceneCarPlayerPPCarAlpha(),
      sceneNoiseComposeSlow(),
      sceneNearNoFog(),
      sceneOcclusionDOFPrepass(),
      sceneOcclusionDOF(),
      sceneNearCarAlpha(),
      sceneNoiseAssembleSlow(),
      sceneNoiseComposeFast(),
      sceneNoiseAssembleFast(),
      sceneOceanNmap(),
      sceneAVTLRFront(),
      sceneAVTLRBack(),
      sceneHUDMap(),
      sceneShadowClear(),
      sceneShadow(),
      sceneShadowCockpitClear(),
      sceneShadowCockpit(),
      sceneEnvMapBack(),
      sceneEnvMap(),
      sceneBack(),
      sceneEnvMapMini(),
      sceneSunComputeEyeLum(),
      sceneSunComputeDazzleCoeff(),
      sceneOcclusion(),
      sceneBack2D(),
      sceneClearDepth(),
      sceneReflectPrepass(),
      sceneReflect(),
      sceneNearCompositing(),
      sceneSunCore(),
      sceneCarPlayerCompositing(),
      sceneCockpit(),
      sceneOcclusionCockpit(),
      sceneCockpitAvatarPrepassCarAlpha(),
      sceneCockpitAvatarPrepass(),
      sceneCockpitCarAlpha(),
      scenePostCockpit(),
      sceneCockpitAvatar(),
      sceneCockpitAvatarCarAlpha(),
      sceneSunHalo(),
      sceneSunMarker(),
      sceneHelmet(),
      sceneHelmetCarAlpha(),
      sceneSunExtractMarker(),
      sceneCockpitCompositing(),
      sceneSunDazzle(),
      sceneSpotMap2D(),
      sceneBefore2D(),
      scene2D(),
      sceneAfter2D(),
      ambientLightBack(),
      directionalLightBack(),
      lightsBack(),
      default2DAmbientLight(),
      default2DDirectionalLight(),
      lights2DDefault(),
      lightLRAmbient( 0.0f, 0.0f, 0.0f, 1.0f ),
      lightLRDiffuse( 0.0f, 0.0f, 0.0f, 1.0f ),
      lightLRDirectionFront( 0.0f, 0.0f, 0.0f ),
      lightLRDirectionBack( 0.0f, 0.0f, 0.0f ),
      lightLRFront(),
      lightLRBack(),
      lightsLRFront(),
      lightsLRBack(),
      lightsEnvmap(),
      globalLODFactor( 1.0f ),
      fovVsLodFactor( 1.0f ),
      fovVsLodFactorStart( 0.4f ),
      fovVsLodFactorEnd( 0.1f ),
      lineMipBias( 0.1f ),
      vegetationUVAFactor( 0.1f ),
      vegetationFactor( 0.015f ),
      vegetationPeriod( 0.001f ),
      treeWindSpeedLimit( 200.5f ),
      vegetationWindSpeedLimit( 100.0f ),
      hmapBeginFadeDist( 100.0f ),
      hmapFadeLength( 3000.0f ),
      nearPlane( 0.8f ),
      farPlane( 40000.0f ),
      lodFarPlane( 40000.0f ),
      interiorNearPlane( 0.1f ),
      interiorFarPlane( 40.0f ),
      sunDirection( 0.0f, -0.4f, -1.0f ),
      pActiveScene( nullptr ),
      pActiveCamera( nullptr ),
      pListeners( nullptr ),
      pLODListeners( nullptr ),
      deviceResetReason( 0 ),
      pDeviceResetListeners( nullptr ) {
    // FUN_00996f80
    sunDirection.normalize();

    shaderUniforms[4] = { 0.1456f, -0.49f, -0.49f, -1.5f };
    shaderUniforms[5] = { 1.0f, 1.0f, 1.0f, 1.0f };

    gpRender = this;
}

GSRender::~GSRender() { gpRender = nullptr; }

bool GSRender::initialize( TestDriveGameInstance *param_2 ) {
    // FUN_00996850
    this->pGameInstance = param_2;
    this->bPaused = false;

    gEdRender.Config.MaxNumHiearchyNode = gpConfig->NbMaxHierNode;
    gEdRender.Config.MaxNumInstance = gpConfig->NbMaxInstance;
    gEdRender.Config.pWindowHandle = gEdSys.getWindow();
    gEdRender.Config.Buckets = kTDUBucketConfig;
    gEdRender.Config.MaxNum3DG = 0x100;
    gEdRender.Config.MaxNumUVA = 0x100;
    gEdRender.Config.MaxNum2DB = 0x800;
    gEdRender.Config.bUseTestDriveHash = true;
    gEdRender.Config.FrameGraph.MaxNumHiearchyNode = 0x17c;
    gEdRender.Config.FrameGraph.MaxNumInstance = 10000;
    gEdRender.Config.FrameGraph.MaxNumDrawList = 500;
    gEdRender.Config.FrameGraph.MaxNumObject = 0x9a;
    gEdRender.Config.FrameGraph.MaxNumSceneData = 0x5a;
    gEdRender.Config.FrameGraph.MaxNumInstanceWithCustomMaterial = 100;
    gEdRender.Config.MaxNum2DM = 1000;
    gEdRender.Config.MaxNum3DD = 0x180;
    gEdRender.Config.CoreID = 2;
    gEdRender.Config.field1_0x18 = 0x96000;
    gEdRender.Config.field2_0x1c = 0x12c000;
    gEdRender.Config.MemoryPoolCapacity[0] = 8000;
    gEdRender.Config.MemoryPoolCapacity[1] = 0x800;
    gEdRender.Config.MemoryPoolCapacity[2] = 0x200;
    gEdRender.Config.bAllowGPUCapture = false;
    gEdRender.Config.MaxNumSetupNode = 0x1000;
    gEdRender.Config.MaxNumLightNode = 0x1000;
    gEdRender.Config.MaxNumPostFX = 0x40;
    gEdRender.Config.MemoryPoolSize = gEdRender.getRequiredMem() + 0x2800;
    gEdRender.Config.pMemoryPool =
        TestDrive::Alloc( gEdRender.Config.MemoryPoolSize );

    int32_t edRenderInit = gEdRender.initialize();
    if ( edRenderInit == 0 ) {
        if ( ( gpConfig->TimerPriority & eTimerPriority::SyncToVBL ) != 0 ) {
            vsyncMode = eVSyncMode( Clamp( gpConfig->NbVBL, 1, 4 ) );
            gEdRender.pRenderDevice->setVSyncEnabled( true );
        }

        if ( gpConfig->bWindowed ||
             gpConfig->FullscreenMode == eFullscreenMode::FM_Desktop ) {
            renderWidth = gpConfig->WindowWidth;
            renderHeight = gpConfig->WindowHeight;
        } else {
            renderWidth = gpConfig->Width;
            renderHeight = gpConfig->Height;
        }

        // Note: The original game didn't work this way (but it's probably more
        // convenient than making fullscreen exclusive only...)
        gEdSys.resizeGameWindow( renderWidth, renderHeight,
                                 gpConfig->FullscreenMode,
                                 gpConfig->bWindowed );

        gBootDisplayMode.Width = renderWidth;
        gBootDisplayMode.Height = renderHeight;

        gEdRender.Config.bAllocateZBufferForMainWindow = true;
        gEdRender.Config.WindowWidth = renderWidth;
        gEdRender.Config.WindowHeight = renderHeight;
        if ( renderWidth < 900 ) {
            gEdRender.Config.WindowWidth = 900;
        }

        if ( renderHeight < 900 ) {
            gEdRender.Config.WindowHeight = 900;
        }

        bool bVar1 = gEdRender.selectAdapterAndCreateDevice();
        if ( bVar1 ) {
            // Update aspect ratio flag based on the actual render resolution
            // (this will effectively override the configuration parsed from
            // disk).
            aspectRatio = (float)renderWidth / (float)renderHeight;
            invAspectRatio = 1.0f / aspectRatio;

            gpConfig->setScreenRatio( abs( aspectRatio - 1.777778f ) <=
                                      abs( aspectRatio - 1.333333f ) );

            activeAA = gpPlayerData->getAAMethod();
            bHDREnabled = gpPlayerData->isHDREnabled();
            bWasHDREnabled = bHDREnabled;

            {
                fprintf( stderr, "OTDU: profile AA=%d HDR=%d LOD=%u\n",
                         (int)activeAA, (int)bHDREnabled,
                         (unsigned)activeLODQuality );
                const char *pQuality = getenv( "OTDU_QUALITY" );
                if ( pQuality != nullptr && strcmp( pQuality, "low" ) == 0 ) {
                    activeAA = eAntiAliasingMethod::AAM_Disabled;
                    bHDREnabled = false;
                    bWasHDREnabled = false;
                    setLODQuality( 0 );
                    fprintf( stderr, "OTDU: quality override LOW applied\n" );
                }
            }

            gPickedRefreshRate = gEdSys.getMonitorRefreshRate();
            gBootDisplayMode.RefreshRate = uint32_t( gPickedRefreshRate );

            setupRenderPasses();
            allocateDeviceResources();

            gpWeather->registerListener( this );

            gEdRender.pUnknownCallback = FUN_00994850;
            bInitialized = true;
            return true;
        }
    }

    return false;
}

void GSRender::tick( float totalTime, float deltaTime ) {
    // FUN_00998710
    gEdRender.Time = totalTime;
    gEdRender.pRenderDevice->setTime( totalTime );

    if ( gpWeather->isConfigDirty() ) {
        gpWeather->updateActiveConfig();
    }

    float fVar2 = gpWeather->getActiveRandomizedWeatherConfig().getWindSpeed();
    if ( vegetationWindSpeedLimit <=
         gpWeather->getActiveRandomizedWeatherConfig().getWindSpeed() ) {
        fVar2 = vegetationWindSpeedLimit;
    }
    gUVATime = vegetationUVAFactor * fVar2 * deltaTime + gUVATime;
    gRenderBuckets[eRenderBucket::RB_Vegetation].Time = gUVATime;
    gRenderBuckets[eRenderBucket::RB_VegetationAlphaTest].Time = gUVATime;
    gRenderBuckets[eRenderBucket::RB_VegetationAlpha].Time = gUVATime;
    gRenderBuckets[eRenderBucket::RB_VegetationFullAlpha].Time = gUVATime;
    updateWind();

    pActiveScene = nullptr;
    pActiveCamera = nullptr;
}

void GSRender::draw( float deltaTime ) {
    // FUN_00994fc0
    shaderUniforms[5].y() = -1.0f / hmapFadeLength;
    shaderUniforms[5].z() =
        ( hmapBeginFadeDist + hmapFadeLength ) * ( 1.0f / hmapFadeLength );

    for ( uint32_t i = 0; i < 8; i++ ) {
        gEdRender.Uniforms[i] = shaderUniforms[i];
    }
}

void GSRender::terminate() {
    // FUN_00994bc0
    if ( gpWeather != nullptr ) {
        gpWeather->unregisterListener( this );
    }
    destroyDeviceResources();

    void *_Memory = gEdRender.Config.pMemoryPool;
    gEdRender.Config.pMemoryPool = nullptr;
    gEdRender.Config.MemoryPoolSize = 0;
    gEdRender.destroy();

    if ( gEdRender.Config.pMemoryPool != nullptr ) {
        TestDrive::FreeAligned( gEdRender.Config.pMemoryPool );
    }
}

void GSRender::setLODQuality( const uint32_t qualityIndex ) {
    // FUN_00993b50
    if ( qualityIndex < 0 || 2 < qualityIndex ) {
        return;
    }

    bool bDirtyLODQuality = activeLODQuality != qualityIndex;
    activeLODQuality = qualityIndex;

    switch ( activeLODQuality ) {
    case 0:
        globalLODFactor = 0.6f;
        lodFarPlane = farPlane * 0.25f;
        break;
    case 1:
        globalLODFactor = 0.8f;
        lodFarPlane = farPlane * 0.50f;
        break;
    case 2:
    default:
        globalLODFactor = 1.0f;
        lodFarPlane = farPlane;
        break;
    };

    gpPlayerData->setLODQuality( activeLODQuality );
    if ( bDirtyLODQuality ) {
        notifyConfigListeners( pLODListeners );
    }
}

void GSRender::setGammaRamp( const uint32_t gammaLevel ) {
    // FUN_00509e70
    const float exponent =
        1.0f / ( float( Min( gammaLevel, 10u ) ) * 0.1f * 1.4f + 0.6f );

    GammaRamp ramp;
    for ( uint32_t i = 0; i < GammaRamp::kNumControlPoints; i++ ) {
        const uint16_t value = uint16_t(
            std::pow( float( i ) * ( 1.0f / 255.0f ), exponent ) * 65535.0f );
        ramp.Red[i] = value;
        ramp.Green[i] = value;
        ramp.Blue[i] = value;
    }

    gEdRender.pRenderDevice->setGammaRamp( ramp );
}

void GSRender::onWeatherConfigChange( WeatherConfig *param_1 ) {
    // FUN_009982e0
    defaultAmbientLight.Color = param_1->getGlobalAmbientColor();

    param_1->fillDirectionalLight( defaultDirectionalLight );

    param_1->fillFogDesc( sceneNear.getFogDescWrite() );
    param_1->fillFogDesc( scenePreNear.getFogDescWrite() );
    param_1->fillFogDesc( sceneNearOpaque.getFogDescWrite() );
    param_1->fillFogDesc( sceneCarPlayerPrepass.getFogDescWrite() );
    param_1->fillFogDesc( sceneNearPrePassVegetation.getFogDescWrite() );
    param_1->fillFogDesc( sceneParticles.getFogDescWrite() );
    param_1->fillFogDesc( sceneCarPlayer.getFogDescWrite() );
    param_1->fillFogDesc( sceneNearOpaqueCarAlpha.getFogDescWrite() );
    param_1->fillFogDesc( sceneCarPlayerCarAlpha.getFogDescWrite() );
    param_1->fillFogDesc( sceneNearAfterCarAlpha.getFogDescWrite() );
    param_1->fillFogDesc( sceneCarPlayerPPCarAlpha.getFogDescWrite() );

    updateWind();

    shaderUniforms[1] = param_1->getTerrainUniformParams();

    // OTDU_ADDITION
    gEdRender.NightParams = param_1->getNightParams();
    gEdRender.LampParams = param_1->getLampParams();
    const Eigen::Vector3f sunDirection = param_1->getSunDirection();
    gEdRender.MoonDirection = Eigen::Vector4f(
        -sunDirection.x(), -sunDirection.y(), -sunDirection.z(), 0.0f );
}

uint32_t GSRender::getFreeStencilMask() {
    // FUN_00993940
    uint8_t iVar1 = 0;
    do {
        if ( ( layerBits & 1 << ( iVar1 & 0x1f ) ) == 0 ) {
            layerBits = 1 << ( iVar1 & 0x1f ) | layerBits;
            return iVar1;
        }
        iVar1 = iVar1 + 1;
    } while ( iVar1 < 8 );

    return 0xffffffff;
}

void GSRender::releaseStencilBits( uint32_t mask ) {
    // FUN_006b955e (inlined)
    layerBits &= ~( 1 << ( (uint8_t)mask & 0x1f ) ) &
                 ~( 1 << ( (uint8_t)mask + 1U & 0x1f ) ) &
                 ~( 1 << ( (uint8_t)mask + 2U & 0x1f ) ) &
                 ~( 1 << ( (uint8_t)mask + 3U & 0x1f ) ) &
                 ~( 1 << ( (uint8_t)mask + 4U & 0x1f ) );
}

void GSRender::AppendListener( RenderConfigurationChangeListener *&pHead,
                               RenderConfigurationChangeListener *callback ) {
    callback->pNextListener = nullptr;
    if ( pHead == nullptr ) {
        pHead = callback;
        return;
    }

    RenderConfigurationChangeListener *pListener = pHead;
    while ( pListener->pNextListener != nullptr ) {
        pListener = pListener->pNextListener;
    }
    pListener->pNextListener = callback;
}

void GSRender::RemoveListener( RenderConfigurationChangeListener *&pHead,
                               RenderConfigurationChangeListener *callback ) {
    RenderConfigurationChangeListener *pPrev = nullptr;

    for ( RenderConfigurationChangeListener *pIt = pHead; pIt != nullptr;
          pIt = pIt->pNextListener ) {
        if ( pIt == callback ) {
            if ( pPrev == nullptr ) {
                pHead = pIt->pNextListener;
            } else {
                pPrev->pNextListener = pIt->pNextListener;
            }
            return;
        }
        pPrev = pIt;
    }
}

void GSRender::registerConfigListener(
    RenderConfigurationChangeListener *callback ) {
    AppendListener( pListeners, callback );
}

void GSRender::unregisterConfigListener(
    RenderConfigurationChangeListener *callback ) {
    RemoveListener( pListeners, callback );
}

void GSRender::registerLODListener(
    RenderConfigurationChangeListener *callback ) {
    AppendListener( pLODListeners, callback );
}

void GSRender::unregisterLODListener(
    RenderConfigurationChangeListener *callback ) {
    RemoveListener( pLODListeners, callback );
}

void GSRender::registerDeviceResetListener(
    RenderDeviceResetListener *callback ) {
    if ( pDeviceResetListeners == nullptr ) {
        pDeviceResetListeners = callback;
        return;
    }

    RenderDeviceResetListener *pIt = pDeviceResetListeners;
    while ( pIt->pNextResetListener != nullptr ) {
        pIt = pIt->pNextResetListener;
    }

    pIt->pNextResetListener = callback;
}

void GSRender::unregisterDeviceResetListener(
    RenderDeviceResetListener *callback ) {
    if ( callback == nullptr ) {
        return;
    }

    RenderDeviceResetListener *pPrev = nullptr;
    for ( RenderDeviceResetListener *pIt = pDeviceResetListeners;
          pIt != nullptr; pIt = pIt->pNextResetListener ) {
        if ( pIt == callback ) {
            if ( pPrev == nullptr ) {
                pDeviceResetListeners = pIt->pNextResetListener;
            } else {
                pPrev->pNextResetListener = pIt->pNextResetListener;
            }
            return;
        }
        pPrev = pIt;
    }
}

void GSRender::notifyDeviceResetListeners() {
    // FUN_00994850
    for ( RenderDeviceResetListener *pIt = pDeviceResetListeners;
          pIt != nullptr; pIt = pIt->pNextResetListener ) {
        if ( !pIt->onRenderDeviceReset( deviceResetReason ) ) {
            return;
        }
    }
}

bool GSRender::resize( uint32_t width,
                       uint32_t height,
                       uint32_t refreshRate,
                       eAntiAliasingMethod antiAliasing,
                       bool bEnableHDR ) {
    // FUN_00995140
    bool cVar1;
    bool bVar1;

    gEdRender.Config.WindowWidth = 900;
    if ( 899 < width ) {
        gEdRender.Config.WindowWidth = width;
    }
    gEdRender.Config.WindowHeight = 900;
    if ( 899 < height ) {
        gEdRender.Config.WindowHeight = height;
    }
    activeAA = antiAliasing;
    bWasHDREnabled = bHDREnabled;
    bHDREnabled = bEnableHDR;

    static constexpr uint32_t kMinRenderDim = 256u;
    renderWidth = ( width < kMinRenderDim ) ? kMinRenderDim : width;
    renderHeight = ( height < kMinRenderDim ) ? kMinRenderDim : height;
    aspectRatio = (float)renderWidth / (float)renderHeight;
    invAspectRatio = 1.0f / aspectRatio;

    const bool bDisplayModeChanged =
        ( gWindowWidth != width ) || ( gWindowHeight != height ) ||
        ( uint32_t( gPickedRefreshRate ) != refreshRate ) ||
        ( gpConfig != nullptr &&
          !gEdSys.isWindowStateApplied( gpConfig->FullscreenMode,
                                        bool( gpConfig->bWindowed ) ) );

    gWindowWidth = width;
    gWindowHeight = height;
    gPickedRefreshRate = refreshRate;

    if ( bDisplayModeChanged && gpConfig != nullptr ) {
        gEdSys.resizeGameWindow( width, height, gpConfig->FullscreenMode,
                                 gpConfig->bWindowed );
    }

    cVar1 = gEdRender.applyVideoSettingsChange( gWindowWidth, gWindowHeight,
                                                vsyncMode, activeAA );
    if ( cVar1 ) {
        gFrameGraph.dropClearCommands();

        FUN_00994330();

        bVar1 = allocateRenderTargets();
        if ( bVar1 ) {
            bVar1 = linkPostFXNodes();
            if ( bVar1 ) {
                bVar1 = notifyConfigListeners( pListeners );
            }
        }

        gEdRender.invalidateRenderTargetBindings();

        return bVar1;
    }

    return true;
}

bool GSRender::resizeToWindow( uint32_t width, uint32_t height ) {
    if ( !bInitialized ) {
        return true;
    }

    if ( width == 0 || height == 0 ) {
        return true;
    }

    if ( width == renderWidth && height == renderHeight ) {
        return true;
    }

    gEdRender.pRenderDevice->waitForIdle();

    return resize( width, height, gPickedRefreshRate, activeAA, bHDREnabled );
}

#ifdef OTDU_USE_IMGUI
void GSRender::drawDebug( float deltaTime ) {
    ImGui::Text( "Render Time: %.3f (internal frame index %u)", gEdRender.Time,
                 gEdRender.pRenderDevice->getInternalFrameIndex() );
    ImGui::Checkbox( "Enable Gamma Ramp",
                     &gEdRender.pRenderDevice->getGammaRampPass().bEnabled );

    ImGui::SeparatorText( "Geometry" );
    ImGui::SliderFloat( "LOD factor", &DebugLODFactor, 0.01f, 1.0f, "%.2f" );
    if ( ImGui::SliderInt( "LOD Quality", (int32_t *)&activeLODQuality, 0,
                           2 ) ) {
        setLODQuality( activeLODQuality );
    }
    bool bFOVvsLOD = bFovVsLodOn;
    if ( ImGui::Checkbox( "bFovVsLodOn", &bFOVvsLOD ) ) {
        bFovVsLodOn = bFOVvsLOD;
    }

    ImGui::SeparatorText( "Heightmap" );
    ImGui::Checkbox( "Wireframe", &gbDebugWireframeHmap );
    ImGui::Checkbox( "Disable footprint cull",
                     &gbDebugHmapDisableFootprintCull );
    ImGui::Checkbox( "Disable stitch mask", &gbDebugHmapDisableStitchMask );
    ImGui::Checkbox( "Force finest LOD", &gbDebugHmapForceFinestLOD );
    ImGui::Checkbox( "Disable seams", &gbDebugHmapDisableSeams );

    ImGui::SeparatorText( "Background" );
    ImGui::Checkbox( "Draw background clouds", &gbBackgroundCloudsEnabled );
    ImGui::Checkbox( "Draw sky dome", &gbBackgroundDomeEnabled );
    ImGui::Checkbox( "Sky dome wireframe", &gbBackgroundDomeWireframe );

    ImGui::SeparatorText( "Ocean" );
    ImGui::Checkbox( "Reflect opaque heightmap (slot 0)",
                     &gbDebugOceanReflectOpaqueHeightmap );

    ImGui::SeparatorText( "Env Map" );
    ImGui::Checkbox( "Dense reflection", &gbDebugEnvMapDenseReflection );
    if ( gbDebugEnvMapDenseReflection ) {
        ImGui::SliderFloat( "Far plane scale", &gDebugEnvMapFarPlaneScale, 1.0f,
                            32.0f, "%.1fx" );
        ImGui::SliderFloat( "Off-axis reach", &gDebugEnvMapFarPlaneCoeff, 0.3f,
                            1.0f, "%.2f" );
        ImGui::SliderFloat( "Object cull scale", &gDebugEnvMapObjectCullScale,
                            1.0f, 64.0f, "%.1fx" );
    }

    ImGui::Checkbox( "Enable debug overlay", &gbDebugEnvMapOverlay );

    MngEnvMap::DrawEnvMapDebugPanel( 192.0f );

    ImGui::SeparatorText( "Shadow map" );
    if ( gpMngResources != nullptr && gpImgui != nullptr ) {
        ShadowResources &shadowResources = gpMngResources->getShadowResources();

        ImGui::BeginGroup();
        ImGui::Text( "Exterior (casters)" );
        Material *pShadowMaterial = gpImgui->getDebugViewMaterial(
            &shadowResources.getShadowMap2DB().Render2DB );
        if ( pShadowMaterial != nullptr ) {
            ImGui::Image( (ImTextureID)pShadowMaterial,
                          ImVec2( 256.0f, 256.0f ) );
        }
        ImGui::EndGroup();

        ImGui::SameLine();

        ImGui::BeginGroup();
        ImGui::Text( "Cockpit" );
        Material *pCockpitMaterial = gpImgui->getDebugViewMaterial(
            &shadowResources.getShadowMapCockpit2DB().Render2DB );
        if ( pCockpitMaterial != nullptr ) {
            ImGui::Image( (ImTextureID)pCockpitMaterial,
                          ImVec2( 256.0f, 256.0f ) );
        }
        ImGui::EndGroup();

#if OTDU_DEVBUILD
        if ( ImGui::Button( "Dump exterior map" ) ) {
            RenderTarget *pRenderTarget =
                shadowResources.getShadowMap2DB().pRenderTarget;
            if ( pRenderTarget != nullptr ) {
                gEdRender.pRenderDevice->requestRenderTargetDump(
                    pRenderTarget->getTextureColor(), "shadowmap_exterior" );
            }
        }

        ImGui::SameLine();

        if ( ImGui::Button( "Dump cockpit map" ) ) {
            RenderTarget *pRenderTarget =
                shadowResources.getShadowMapCockpit2DB().pRenderTarget;
            if ( pRenderTarget != nullptr ) {
                gEdRender.pRenderDevice->requestRenderTargetDump(
                    pRenderTarget->getTextureColor(), "shadowmap_cockpit" );
            }
        }
#endif

        if ( gpMngShadow != nullptr ) {
            ImGui::Text( "Exterior: %u casters, %u receivers",
                         gpMngShadow->getLastNumCaster(),
                         gpMngShadow->getLastNumReceiver() );
            ImGui::Text( "Cockpit : %u casters, %u receivers",
                         gpMngShadow->getLastNumCasterCockpit(),
                         gpMngShadow->getLastNumReceiverCockpit() );

            const Eigen::Vector4f &sunDir = gpMngShadow->getSunDirection();
            const float sunElevation =
                -asinf( std::min( 1.0f, std::max( -1.0f, sunDir.y() ) ) ) *
                57.29578f;
            ImGui::Text( "Sun dir  = %.5f %.5f %.5f (elevation %.2f deg)",
                         sunDir.x(), sunDir.y(), sunDir.z(), sunElevation );

            const Eigen::Vector4f &bbMin = gpMngShadow->getCasterBBMin();
            const Eigen::Vector4f &bbMax = gpMngShadow->getCasterBBMax();
            ImGui::Text( "CasterBB min = %.3f %.3f %.3f", bbMin.x(), bbMin.y(),
                         bbMin.z() );
            ImGui::Text( "CasterBB max = %.3f %.3f %.3f", bbMax.x(), bbMax.y(),
                         bbMax.z() );
            ImGui::Text( "CasterBB size= %.3f %.3f %.3f", bbMax.x() - bbMin.x(),
                         bbMax.y() - bbMin.y(), bbMax.z() - bbMin.z() );
            ImGui::Text( "Ortho    = %.3f x %.3f m (%.2f mm/texel)",
                         gpMngShadow->getLastOrthoWidth(),
                         gpMngShadow->getLastOrthoHeight(),
                         gpMngShadow->getLastOrthoWidth() * 1000.0f / 900.0f );
            ImGui::Text( "Caster   = dist %.3f m -> G %.6f",
                         gpMngShadow->getLastCasterDistance(),
                         gpMngShadow->getLastCasterIntensity() );

            const Eigen::Vector4f &cockpitBBMin =
                gpMngShadow->getCasterCockpitBBMin();
            const Eigen::Vector4f &cockpitBBMax =
                gpMngShadow->getCasterCockpitBBMax();
            ImGui::Text( "CockpitBB ctr = %.3f %.3f %.3f",
                         ( cockpitBBMax.x() + cockpitBBMin.x() ) * 0.5f,
                         ( cockpitBBMax.y() + cockpitBBMin.y() ) * 0.5f,
                         ( cockpitBBMax.z() + cockpitBBMin.z() ) * 0.5f );
            ImGui::Text( "CockpitBB size= %.3f %.3f %.3f",
                         cockpitBBMax.x() - cockpitBBMin.x(),
                         cockpitBBMax.y() - cockpitBBMin.y(),
                         cockpitBBMax.z() - cockpitBBMin.z() );
            ImGui::Text( "Cockpit ortho = %.3f x %.3f m (%.2f mm/texel)",
                         gpMngShadow->getLastOrthoWidthCockpit(),
                         gpMngShadow->getLastOrthoHeightCockpit(),
                         gpMngShadow->getLastOrthoWidthCockpit() * 1000.0f /
                             900.0f );

            ImGui::Text( "Ground   = %d/%d cells (%d gathered vtx)%s",
                         gpMngShadow->getLastNumGroundCells(),
                         gpMngShadow->getMaxGroundCells(),
                         gpMngShadow->getLastNumGroundCells() * 6,
                         gpMngShadow->getLastNumGroundCells() >=
                                 gpMngShadow->getMaxGroundCells()
                             ? "  <-- CAPPED"
                             : "" );
            ImGui::Text(
                "Road     = %d/%d vtx from %d bodies (%d sun-culled)%s",
                gpMngShadow->getLastNumRoadVerts(),
                gpMngShadow->getMaxRoadVerts(),
                gpMngShadow->getLastNumRoadBodies(),
                gpMngShadow->getLastNumRoadCulled(),
                gpMngShadow->getLastNumRoadVerts() >=
                        gpMngShadow->getMaxRoadVerts()
                    ? "  <-- CAPPED"
                    : "" );

            ImGui::Checkbox( "Disable ground strip receiver",
                             &gbDebugShadowDisableGroundStrip );
            ImGui::Checkbox( "Disable road receiver",
                             &gbDebugShadowDisableRoadReceiver );
        }

        if ( gpCamera != nullptr && gpCamera->getActiveCamera() != nullptr ) {
            const uint32_t camFlags = gpCamera->getActiveCamera()->getFlags();
            ImGui::Text( "Camera flags=0x%x bit0(draw car)=%u "
                         "bit18(cockpit)=%u",
                         camFlags, camFlags & 1u, ( camFlags >> 0x12 ) & 1u );
        }
    } else {
        ImGui::Text( "Shadow resources are not available yet." );
    }
}
#endif

bool GSRender::allocateDeviceResources() {
    // FUN_00995520
    bool bVar13 = allocateRenderTargets();
    if ( !bVar13 ) {
        OTDU_LOG_ERROR( "Failed to allocate global render targets!\n" );
        return false;
    }

    bVar13 = allocateAtmosphereResources();
    if ( !bVar13 ) {
        OTDU_LOG_ERROR( "Failed to allocate sun/ocean render targets!\n" );
        return false;
    }

    bVar13 = setupSunPostFXStack();
    if ( !bVar13 ) {
        OTDU_LOG_ERROR( "Failed to setup sun PostFX stack!\n" );
        return false;
    }

    linkPostFXNodes();

    float width = (float)renderWidth;
    if ( width < 0 ) {
        width = width + 4.2949673e+09f;
    }
    float height = (float)renderHeight;
    if ( height < 0 ) {
        height = height + 4.2949673e+09f;
    }

    // 2D Camera
    camera2D.setAspectRatio( width, height );
    camera2D.setDefaultMatrix2D();

    // Main Camera
    Eigen::Vector3f defaultPosition( 0.0f, 0.0f, 200.0f );
    Eigen::Vector3f defaultLookAt( 0.0f, 0.0f, 0.0f );
    Eigen::Vector3f defaultUp( 0.0f, 1.0f, 0.0f );

    cameraMain.setAspectRatio( width, height );
    cameraMain.setFOV( 1.5707964f );
    cameraMain.updateMatrix( defaultPosition, defaultUp, defaultLookAt );

    // LRFront Camera
    defaultPosition = Eigen::Vector3f( 0.0f, 0.85f, 1.0f );
    defaultLookAt = Eigen::Vector3f( 0.0f, 0.85f, 0.0f );

    cameraLRFront.setAspectRatio( 1.0f, 1.0f );
    cameraLRFront.updateMatrix( defaultPosition, defaultUp, defaultLookAt );

    // LRBack Camera
    defaultPosition = Eigen::Vector3f( 0.0f, 0.85f, -1.0f );

    cameraLRBack.setAspectRatio( 1.0f, 1.0f );
    cameraLRBack.updateMatrix( defaultPosition, defaultUp, defaultLookAt );

    defaultViewport.Flags = defaultViewport.Flags & 0xfffffff8;

    uint32_t uVar19 = clearColorViewport.Flags;
    clearColorViewport.Flags = uVar19 | 1;
    clearColorViewport.Flags = ( uVar19 & 0xfffffff9 ) | 1;
    clearColorViewport.ClearColor.R = 0;
    clearColorViewport.ClearColor.G = 0;
    clearColorViewport.ClearColor.B = 0;
    clearColorViewport.ClearColor.A = 0xff;

    uint32_t *puVar1 = &clearViewport.Flags;
    *puVar1 = *puVar1 | 7;

    puVar1 = &clearColorDepthViewport.Flags;
    *puVar1 = *puVar1 | 3;
    *puVar1 = *puVar1 & 0xfffffffb;
    clearColorDepthViewport.ClearColor.R = 0;
    clearColorDepthViewport.ClearColor.G = 0;
    clearColorDepthViewport.ClearColor.B = 0;
    clearColorDepthViewport.ClearColor.A = 0xff;

    puVar1 = &clearColorStencilViewport.Flags;
    *puVar1 = *puVar1 | 6;
    *puVar1 = *puVar1 & 0xfffffffe;
    clearColorStencilViewport.ClearColor.R = 0;
    clearColorStencilViewport.ClearColor.G = 0;
    clearColorStencilViewport.ClearColor.B = 0;
    clearColorStencilViewport.ClearColor.A = 0xff;

    puVar1 = &clearDepthViewport.Flags;
    *puVar1 = *puVar1 | 2;
    *puVar1 = *puVar1 & 0xfffffffa;
    clearDepthViewport.ClearColor.R = 0;
    clearDepthViewport.ClearColor.G = 0;
    clearDepthViewport.ClearColor.B = 0;
    clearDepthViewport.ClearColor.A = 0xff;

    puVar1 = &viewportBack.Flags;
    *puVar1 = *puVar1 | 7;
    viewportBack.ClearColor.R = 0;
    viewportBack.ClearColor.G = 0;
    viewportBack.ClearColor.B = 0;
    viewportBack.ClearColor.A = 0xff;

    viewportHUDMap.ClearColor.R = 0x1c;
    viewportHUDMap.ClearColor.G = 0x3c;
    viewportHUDMap.ClearColor.B = 0x5e;
    viewportHUDMap.ClearColor.A = 0xff;

    puVar1 = &viewportNoise.Flags;
    *puVar1 = *puVar1 & 0xfffffff8;

    viewportShadowClear.ClearColor.R = 0xff;
    viewportShadowClear.ClearColor.G = 0xff;
    viewportShadowClear.ClearColor.B = 0xff;
    viewportShadowClear.ClearColor.A = 0xff;

    puVar1 = &viewportShadowClear.Flags;
    *puVar1 = *puVar1 | 7;

    viewportShadow.ClearColor.R = 0xff;
    viewportShadow.ClearColor.G = 0xff;
    viewportShadow.ClearColor.B = 0xff;
    viewportShadow.ClearColor.A = 0xff;

    puVar1 = &viewportShadow.Flags;
    *puVar1 = *puVar1 & 0xfffffff8;

    viewportEnvmapBack.ClearColor.R = 0x2b;
    viewportEnvmapBack.ClearColor.G = 0x51;
    viewportEnvmapBack.ClearColor.B = 0x2b;
    viewportEnvmapBack.ClearColor.A = 0xff;

    puVar1 = &viewportEnvmapBack.Flags;
    *puVar1 = *puVar1 | 7;

    puVar1 = &viewportEnvmapNear.Flags;
    *puVar1 = *puVar1 & 0xfffffff8;

    viewportNear.ClearColor.R = 0xff;
    viewportNear.ClearColor.G = 0x0;
    viewportNear.ClearColor.B = 0x0;
    viewportNear.ClearColor.A = 0xff;

    puVar1 = &viewportNear.Flags;
    *puVar1 = *puVar1 & 0xfffffff8;

    puVar1 = &defaultViewport.Flags;
    *puVar1 = *puVar1 & 0xfffffff8;

    puVar1 = &viewportParticles.Flags;
    *puVar1 = *puVar1 & 0xfffffff8;

    viewportParticles.ClearColor.R = 0x0;
    viewportParticles.ClearColor.G = 0x0;
    viewportParticles.ClearColor.B = 0xff;
    viewportParticles.ClearColor.A = 0x0;

    uVar19 = viewportSun.Flags;
    viewportSun.Flags = uVar19 | 1;
    viewportSun.Flags = ( uVar19 & 0xfffffff9 ) | 1;
    viewportSun.ClearColor.A = 0xff;
    viewportSun.ClearColor.R = 0;
    viewportSun.ClearColor.G = 0;
    viewportSun.ClearColor.B = 0;

    uVar19 = viewportInterior.Flags & 0xfffffffa;
    viewportInterior.Flags = uVar19;
    viewportInterior.Flags = uVar19 | 2;

    puVar1 = &viewportAvatar.Flags;
    *puVar1 = *puVar1 & 0xfffffff8;

    puVar1 = &viewportCockpitAvatar.Flags;
    *puVar1 = *puVar1 & 0xfffffff8;

    puVar1 = &viewportHelmet.Flags;
    *puVar1 = *puVar1 & 0xfffffff8;

    puVar1 = &viewport2D.Flags;
    *puVar1 = ( *puVar1 & 0xfffffffe ) | 2;

    viewportLRFront.ClearColor.Color = 0xff7f7f7f;
    viewportLRBack.ClearColor.Color = 0xff7f7f7f;

    viewportAfter2D = viewport2D;

    ambientLightBack.Color = Eigen::Vector4f::Ones();
    directionalLightBack.Ambient = Eigen::Vector4f::Ones();
    directionalLightBack.Diffuse = Eigen::Vector4f::Ones();
    directionalLightBack.Specular = Eigen::Vector4f::Ones();
    directionalLightBack.Direction = sunDirection;

    lightsBack.addLight( &directionalLightBack );
    lightsBack.addLight( &ambientLightBack );

    lightsDefault.addLight( &defaultDirectionalLight );
    lightsDefault.addLight( &defaultAmbientLight );

    default2DAmbientLight.Color = Eigen::Vector4f::Ones();

    default2DDirectionalLight.Ambient.x() = 0.4f;
    default2DDirectionalLight.Ambient.y() = 0.4f;
    default2DDirectionalLight.Ambient.z() = 0.4f;
    default2DDirectionalLight.Ambient.w() = 1.0f;
    default2DDirectionalLight.Diffuse.x() = 0.8f;
    default2DDirectionalLight.Diffuse.y() = 0.8f;
    default2DDirectionalLight.Diffuse.z() = 0.8f;
    default2DDirectionalLight.Diffuse.w() = 1.0f;
    default2DDirectionalLight.Specular = Eigen::Vector4f::Ones();
    default2DDirectionalLight.Direction = sunDirection;

    lights2DDefault.addLight( &default2DDirectionalLight );
    lights2DDefault.addLight( &default2DAmbientLight );

    lightLRAmbient =
        Eigen::Vector4f( 0.484375f, 0.4921875f, 0.51953125f, 1.0f );
    lightLRDiffuse = Eigen::Vector4f( 0.5f, 0.5f, 0.5f, 1.0f );
    lightLRDirectionFront = Eigen::Vector3f( 0.0f, 0.0f, -1.0f );
    lightLRDirectionBack = Eigen::Vector3f( 0.0f, 0.0f, 1.0f );

    lightLRFront.Ambient = lightLRAmbient;
    lightLRFront.Diffuse = lightLRDiffuse;
    lightLRFront.Direction = lightLRDirectionFront;

    lightLRBack.Ambient = lightLRAmbient;
    lightLRBack.Diffuse = lightLRAmbient;
    lightLRBack.Direction = lightLRDirectionBack;

    lightsLRFront.addLight( &lightLRFront );
    lightsLRBack.addLight( &lightLRBack );

    for ( RenderPass &ppeVar21 : renderPasses ) {
        ppeVar21.pScene->create( ppeVar21.pCamera, ppeVar21.pViewport,
                                 ppeVar21.pFramebufferAttachments,
                                 ppeVar21.Flags, ppeVar21.Name.c_str(), true, 0,
                                 nullptr );

        ppeVar21.pScene->setSceneType( !ppeVar21.bIs3DScene );

        if ( ppeVar21.pLightSetupNode != nullptr ) {
            ppeVar21.pScene->addLightSetup( ppeVar21.pLightSetupNode );
        }

        ppeVar21.pScene->setShaderIndexes( ppeVar21.ShaderIndex,
                                           ppeVar21.ShaderIndex );
    }

    for ( RenderPass &ppeVar21 : renderPasses ) {
        ppeVar21.pScene->setEnabled( ppeVar21.bEnabled );
    }

    for ( RenderPass &ppeVar21 : renderPasses ) {
        if ( ppeVar21.pSceneCopy != nullptr ) {
            ppeVar21.pScene->copyDrawCommands( ppeVar21.pSceneCopy );
        }
    }

    sceneOcclusion.setBucketsMask( BucketMask(
        RB_Car, RB_Avatar, RB_Building, RB_Vegetation, RB_Opaque, RB_Road,
        RB_Heightmap, RB_WaterSea, RB_ShadowPlane, RB_Video ) );
    sceneOcclusionCockpit.setBucketsMask( BucketMask(
        RB_Car, RB_Avatar, RB_Building, RB_Vegetation, RB_Opaque, RB_Road,
        RB_Heightmap, RB_WaterSea, RB_ShadowPlane, RB_Video ) );
    sceneNear.setBucketsMask( BucketMask(
        RB_CarAlphaTest, RB_AvatarAlphaTest, RB_BuildingAlphaTest,
        RB_VegetationAlphaTest, RB_AlphaTest, RB_HeightmapAlphaTest,
        RB_HeightmapAlpha, RB_Water, RB_WaterFullAlpha, RB_Cloud,
        RB_CloudFullAlpha, RB_RoadPatch, RB_Shadow, RB_HardInst,
        RB_FullAlphaFar, RB_Impostor, RB_ImpostorFar, RB_TallGrass,
        RB_Electrical, RB_AvatarAlpha, RB_BuildingAlpha, RB_BuildingFullAlpha,
        RB_AvatarFullAlpha0, RB_AvatarFullAlpha1, RB_AvatarFullAlpha2,
        RB_AvatarFullAlpha3, RB_CarAlpha, RB_VegetationAlpha,
        RB_VegetationFullAlpha, RB_Alpha, RB_ShadowInt, RB_FullAlpha, RB_Sun,
        RB_FrontEnd, RB_Profile ) );
    sceneNearOpaque.setBucketsMask( BucketMask(
        RB_Car, RB_Avatar, RB_Building, RB_Vegetation, RB_Opaque, RB_Road,
        RB_Heightmap, RB_WaterSea, RB_ShadowPlane, RB_Video ) );
    sceneNearOpaqueCarAlpha.setBucketsMask( BucketMask(
        RB_Car, RB_Avatar, RB_Building, RB_Vegetation, RB_Opaque, RB_Road,
        RB_Heightmap, RB_WaterSea, RB_ShadowPlane, RB_Video ) );
    sceneNearCarAlpha.setBucketsMask( BucketMask(
        RB_CarAlphaTest, RB_AvatarAlphaTest, RB_BuildingAlphaTest,
        RB_VegetationAlphaTest, RB_AlphaTest, RB_HeightmapAlphaTest,
        RB_HeightmapAlpha, RB_Water, RB_WaterFullAlpha, RB_Cloud,
        RB_CloudFullAlpha, RB_RoadPatch, RB_Shadow, RB_HardInst,
        RB_FullAlphaFar, RB_Impostor, RB_ImpostorFar, RB_TallGrass,
        RB_Electrical, RB_AvatarAlpha, RB_BuildingAlpha, RB_BuildingFullAlpha,
        RB_AvatarFullAlpha0, RB_AvatarFullAlpha1, RB_AvatarFullAlpha2,
        RB_AvatarFullAlpha3, RB_CarAlpha, RB_VegetationAlpha,
        RB_VegetationFullAlpha, RB_Alpha, RB_ShadowInt, RB_FullAlpha, RB_Sun,
        RB_FrontEnd, RB_Profile ) );
    sceneCockpit.setBucketsMask( ~BucketMask(
        RB_Avatar, RB_AvatarAlpha, RB_AvatarFullAlpha0, RB_AvatarFullAlpha1,
        RB_AvatarFullAlpha2, RB_AvatarFullAlpha3 ) );
    sceneReflectPrepass.setBucketsMask(
        BucketMask( RB_AvatarFullAlpha0, RB_AvatarFullAlpha1,
                    RB_AvatarFullAlpha2, RB_AvatarFullAlpha3 ) );
    sceneOcclusionDOFPrepass.setBucketsMask(
        BucketMask( RB_AvatarFullAlpha0, RB_AvatarFullAlpha1,
                    RB_AvatarFullAlpha2, RB_AvatarFullAlpha3 ) );
    sceneCarPlayerPrepass.setBucketsMask(
        BucketMask( RB_AvatarFullAlpha0, RB_AvatarFullAlpha1,
                    RB_AvatarFullAlpha2, RB_AvatarFullAlpha3 ) );
    sceneCarPlayerPPCarAlpha.setBucketsMask(
        BucketMask( RB_AvatarFullAlpha0, RB_AvatarFullAlpha1,
                    RB_AvatarFullAlpha2, RB_AvatarFullAlpha3 ) );
    sceneNearPrePassVegetation.setBucketsMask(
        BucketMask( RB_Impostor, RB_AvatarFullAlpha0, RB_AvatarFullAlpha1,
                    RB_AvatarFullAlpha2, RB_AvatarFullAlpha3,
                    RB_VegetationAlpha, RB_VegetationFullAlpha ) );
    scenePreNear.setBucketsMask( BucketMask( RB_WaterSea ) );
    scenePostCockpit.setBucketsMask( ~BucketMask(
        RB_Avatar, RB_AvatarAlpha, RB_AvatarFullAlpha0, RB_AvatarFullAlpha1,
        RB_AvatarFullAlpha2, RB_AvatarFullAlpha3 ) );
    sceneCockpitAvatarPrepassCarAlpha.setBucketsMask(
        BucketMask( RB_AvatarFullAlpha0, RB_AvatarFullAlpha1,
                    RB_AvatarFullAlpha2, RB_AvatarFullAlpha3 ) );
    sceneCockpitAvatarPrepass.setBucketsMask(
        BucketMask( RB_AvatarFullAlpha0, RB_AvatarFullAlpha1,
                    RB_AvatarFullAlpha2, RB_AvatarFullAlpha3 ) );
    sceneCockpitCarAlpha.setBucketsMask( ~BucketMask(
        RB_Avatar, RB_AvatarAlpha, RB_AvatarFullAlpha0, RB_AvatarFullAlpha1,
        RB_AvatarFullAlpha2, RB_AvatarFullAlpha3 ) );
    sceneCockpitAvatar.setBucketsMask( BucketMask(
        RB_Avatar, RB_AvatarAlpha, RB_AvatarFullAlpha0, RB_AvatarFullAlpha1,
        RB_AvatarFullAlpha2, RB_AvatarFullAlpha3 ) );
    sceneCockpitAvatarCarAlpha.setBucketsMask( BucketMask(
        RB_Avatar, RB_AvatarAlpha, RB_AvatarFullAlpha0, RB_AvatarFullAlpha1,
        RB_AvatarFullAlpha2, RB_AvatarFullAlpha3 ) );

    // LAB_00996050
    sceneSunCore.getFrustumWrite().setPlanes( 1.0f, 100.0f );
    sceneSunHalo.getFrustumWrite().setPlanes( 1.0f, 100.0f );
    sceneSunMarker.getFrustumWrite().setPlanes( 1.0f, 100.0f );

    // LAB_009961a0
    sceneOcclusionCockpit.getFrustumWrite().setPlanes( 0.1f, 3.0f );
    sceneCockpit.getFrustumWrite().setPlanes( 0.1f, 3.0f );
    sceneCockpitAvatar.getFrustumWrite().setPlanes( 0.1f, 3.0f );
    sceneCockpitAvatarPrepass.getFrustumWrite().setPlanes( 0.1f, 3.0f );
    scenePostCockpit.getFrustumWrite().setPlanes( 0.1f, 3.0f );
    sceneHelmet.getFrustumWrite().setPlanes( 0.1f, 3.0f );
    sceneCockpitCarAlpha.getFrustumWrite().setPlanes( 0.1f, 3.0f );
    sceneCockpitAvatarCarAlpha.getFrustumWrite().setPlanes( 0.1f, 3.0f );
    sceneCockpitAvatarPrepassCarAlpha.getFrustumWrite().setPlanes( 0.1f, 3.0f );
    sceneHelmetCarAlpha.getFrustumWrite().setPlanes( 0.1f, 3.0f );

    // LAB_009962a0
    for ( uint32_t i = 0; i < kNumAvtScenes; i++ ) {
        sceneAVTLRFront[i].getFrustumWrite().setOrthoDimensions( 0.6f, 1.8f );
        sceneAVTLRBack[i].getFrustumWrite().setOrthoDimensions( 0.6f, 1.8f );
    }

    RenderScene *local_a0[0xc] = { &scenePreNear,
                                   &sceneNearOpaque,
                                   &sceneNear,
                                   &sceneNearPrePassVegetation,
                                   &sceneParticles,
                                   &sceneCarPlayerPrepass,
                                   &sceneCarPlayer,
                                   &sceneNearOpaqueCarAlpha,
                                   &sceneNearCarAlpha,
                                   &sceneNearAfterCarAlpha,
                                   &sceneCarPlayerPPCarAlpha,
                                   &sceneCarPlayerCarAlpha };

    for ( uint32_t uVar18 = 0; uVar18 < 0xc; uVar18++ ) {
        FogDesc &peVar5 = local_a0[uVar18]->getFogDescWrite();

        peVar5.DistanceStart = 0.0f;
        peVar5.Type = eFogType::FT_Linear;
        peVar5.Color.x() = 0.39215687f;
        peVar5.Color.y() = 0.5764706f;
        peVar5.Color.z() = 0.7882353f;
        peVar5.DistanceEnd = 12500.0f;
        peVar5.Intensity = 1.0f;
    }

    pSunDownscaleChain[2]->setName( "SunDownSample" ); // DAT_00f080d0
    pSunAvgLumCopy->setName( "SunEyeLumBk" );
    pSunDazzleLUTCopy->setName( "SunDazzleBk" );

    gEdRender.pFrameGraph->insertAfterObject(
        pSunDownscaleChain[2],
        renderPasses[eRenderPass::RP_EnvMapMini].pScene );
    gEdRender.pFrameGraph->insertAfterObject( pSunAvgLumCopy,
                                              pSunDownscaleChain[2] );
    gEdRender.pFrameGraph->insertAfterObject( pSunDazzleLUTCopy,
                                              pSunAvgLumCopy );
    gEdRender.pFrameGraph->insertAfterObject(
        pPostFXMainHDRDownscale,
        renderPasses[eRenderPass::RP_CarCompo].pScene );

    pPostFXMainHDRDownscale->setName( "DownSample4X" );

    setupScenes( nearPlane, farPlane );

    return true;
}

bool GSRender::setupSunPostFXStack() {
    // FUN_00993c00
    pPostFXMainHDR = new ( TestDrive::Alloc( sizeof( PostFXNodeTexSample ) ) )
        PostFXNodeTexSample();
    pPostFXMainHDRDownscale =
        new ( TestDrive::Alloc( sizeof( PostFXNodeDownscale ) ) )
            PostFXNodeDownscale();

    pSunAvgLum = new ( TestDrive::Alloc( sizeof( PostFXNodeTexSample ) ) )
        PostFXNodeTexSample();
    pSunAvgLumCopy =
        new ( TestDrive::Alloc( sizeof( PostFXNodeBlit ) ) ) PostFXNodeBlit();

    pSunDazzleLUT = new ( TestDrive::Alloc( sizeof( PostFXNodeTexSample ) ) )
        PostFXNodeTexSample();
    pSunDazzleLUTCopy =
        new ( TestDrive::Alloc( sizeof( PostFXNodeBlit ) ) ) PostFXNodeBlit();

    pSunBillboard = new ( TestDrive::Alloc( sizeof( PostFXNodeTexSample ) ) )
        PostFXNodeTexSample();

    for ( uint32_t i = 0; i < kNumSunDownscaleRT; i++ ) {
        pSunDownscaleChain[i] =
            new ( TestDrive::Alloc( sizeof( PostFXNodeDownscale ) ) )
                PostFXNodeDownscale();
    }

    pSunAvgLumCopy->connect( pSunAvgLum, 0, 0 );
    pSunDazzleLUTCopy->connect( pSunDazzleLUT, 0, 0 );

    pSunDownscaleChain[0]->connect( pSunBillboard, 0, 0 );
    pSunDownscaleChain[1]->connect( pSunDownscaleChain[0], 0, 0 );
    pSunDownscaleChain[2]->connect( pSunDownscaleChain[1], 0, 0 );

    return true;
}

void GSRender::setupScenes( float nearZ, float farZ ) {
    // FUN_00995370
    scenePreNear.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneNearOpaque.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneNear.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneNearNoFog.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneOcclusionDOFPrepass.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneOcclusionDOF.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneNearPrePassVegetation.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneParticles.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneCarPlayerPrepass.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneCarPlayer.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneNearOpaqueCarAlpha.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneNearCarAlpha.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneNearAfterCarAlpha.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneCarPlayerPPCarAlpha.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneCarPlayerCarAlpha.getFrustumWrite().setPlanes( nearZ, farZ );
}

void GSRender::setupInteriorScenes( float nearZ, float farZ ) {
    // FUN_00995230
    sceneCockpit.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneCockpitAvatarPrepass.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneCockpitAvatar.getFrustumWrite().setPlanes( nearZ, farZ );
    scenePostCockpit.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneCockpitCarAlpha.getFrustumWrite().setPlanes( nearZ, farZ );
    sceneCockpitAvatarPrepassCarAlpha.getFrustumWrite().setPlanes( nearZ,
                                                                   farZ );
    sceneCockpitAvatarCarAlpha.getFrustumWrite().setPlanes( nearZ, farZ );
}

void GSRender::updateFovVsLOD( float param_2 ) {
    // FUN_00994ae0
    gEdRender.LODFactor = DebugLODFactor;
    if ( bFovVsLodOn != false ) {
        if ( fovVsLodFactorStart <= param_2 &&
             param_2 != fovVsLodFactorStart ) {
            param_2 = fovVsLodFactorStart;
        }
        if ( param_2 < fovVsLodFactorEnd ) {
            param_2 = fovVsLodFactorEnd;
        }
        float fVar1 = ( ( fovVsLodFactorStart - param_2 ) /
                        ( fovVsLodFactorStart - fovVsLodFactorEnd ) ) *
                          ( FLOAT_00faa71c - FLOAT_00faa720 ) +
                      FLOAT_00faa720;
        if ( fovVsLodFactor <= 1.0f ) {
            if ( fovVsLodFactor <= 1e-05f && fovVsLodFactor != 1e-05f ) {
                fovVsLodFactor = 1e-05f;
            }
        } else {
            fovVsLodFactor = 1.0f;
        }
        gEdRender.LODFactor = fVar1 / ( globalLODFactor * fovVsLodFactor );
        if ( gEdRender.LODFactor < 1.0f ) {
            gEdRender.LODFactor = 1.0f;
            return;
        }
    }
}

bool GSRender::allocateRenderTargets() {
    // FUN_00994490
    uint32_t height = renderHeight;
    uint32_t width = renderWidth;

    bool bSDR = !bHDREnabled;
    uint32_t uVar6 = height >> 0x1f;
    uint32_t uVar3 = width >> 0x1f;

    eViewFormat viewFormat = ( bSDR ) ? VF_X8R8G8B8 : VF_A16B16G16R16F;
    uint32_t uVar7 = ( bSDR ) ? 0 : 6;
    bool bVar2 = false;
    bVar2 = mainRTwM.allocateAndCreate( width, height, 1, 1, viewFormat, 0x200,
                                        "MainRTwM" );
    if ( !bVar2 ) {
        return false;
    }
    uVar6 = (uint32_t)( height + ( uVar6 & 3 ) ) >> 2;
    uVar3 = (uint32_t)( width + ( uVar3 & 3 ) ) >> 2;
    bVar2 = scnDown4.allocateAndCreate( uVar3, uVar6, 1, 1, viewFormat, 0x200,
                                        "ScnDown4" );

    if ( bVar2 ) {
        bVar2 = mainRT.allocateAndCreate( width, height, 1, 1, viewFormat,
                                          0x200, "MainRT" );
        if ( bVar2 ) {
            pMainRTwM = CreateRenderTarget( width, height, viewFormat, activeAA,
                                            uVar7 );
            if ( pMainRTwM != nullptr ) {
                pMainRT = CopyRenderTarget( pMainRTwM, 8 );
                if ( pMainRT != nullptr ) {
                    pUnknownRT = CopyRenderTarget( pMainRTwM, 8 );
                    if ( pUnknownRT != nullptr ) {
                        pUnknownRT2 = CopyRenderTarget( pMainRTwM, 8 );
                        if ( pUnknownRT2 != nullptr ) {
                            eViewFormat eVar6 = pMainRT->getFormat();
                            pScnDown4RT = FUN_0050aff0( pMainRT, (float)uVar3,
                                                        (float)uVar6,
                                                        viewFormat, 0x200, 1 );
                            if ( pScnDown4RT != nullptr ) {
                                pMainRTwM->bind2DB( &mainRTwM );
                                pMainRT->bind2DB( &mainRT );
                                pUnknownRT->bind2DB( &mainRTwM );
                                pUnknownRT2->bind2DB( &mainRT );

                                RenderTarget *peVar3 =
                                    RenderTarget::GetBackBuffer();
                                peVar3->bind2DB( &mainRT );
                                pScnDown4RT->bind2DB( &scnDown4 );

                                mainRTwMFramebuffer.pAttachments[0] = pMainRTwM;
                                mainRTFramebuffer.pAttachments[0] = pMainRT;

                                backbufferFramebuffer.pAttachments[0] =
                                    RenderTarget::GetBackBuffer();

                                carFramebuffer.pAttachments[0] = pUnknownRT2;
                                carPlayerFramebuffer.pAttachments[0] =
                                    pUnknownRT;

                                sunFramebuffer.pAttachments[0] = pUnknownRT;

                                return true;
                            }
                        }
                    }
                }
            }
        }
    }

    return false;
}

bool GSRender::allocateAtmosphereResources() {
    // FUN_00993fc0
    static constexpr const char *kResourcesName[kNumSunRT] = {
        "SUN_DS_64x64",     "SUN_DS_16x16",       "SUN_DS_4x4",
        "SUN_REAL_LUM",     "SUN_EYE_LUM",        "SUN_EYE_LUM_BK",
        "SUN_DAZZLE_COEFF", "SUN_DAZZLE_COEFF_BK" };

    bool bVar1 = sun2DBs[0].allocateAndCreate( 0x40, 0x40, 1, 1, VF_X8R8G8B8,
                                               0x200, "SUN_DS_64x64" );
    if ( !bVar1 ) {
        return false;
    }

    uint32_t uVar7 = 0;
    eAntiAliasingMethod activeAA = pMainRT->getAntiAliasingMethod();
    pSunRTs[0] = CreateRenderTarget( 0x40, 0x40, VF_X8R8G8B8, activeAA, uVar7 );
    pSunRTs[0]->bind2DB( &sun2DBs[0] );

    uint32_t dimension = 16;
    for ( int32_t i = 1; i < kNumSunRT; i++ ) {
        bVar1 = sun2DBs[i].allocateAndCreate(
            dimension, dimension, 1, 1, VF_X8R8G8B8, 0x200, kResourcesName[i] );
        if ( !bVar1 ) {
            return false;
        }
        pSunRTs[i] = CreateRenderTargetFrom2DB( &sun2DBs[i], 2 );
        OTDU_ASSERT( pSunRTs[i] );

        dimension >>= 2;
        if ( dimension < 2 ) {
            dimension = 1;
        }
    }

    sunRealLumFramebuffer.pAttachments[0] = pSunRTs[0];
    sunEyeLumFramebuffer.pAttachments[0] = pSunRTs[4];
    sunDazzleCoeffFramebuffer.pAttachments[0] = pSunRTs[6];

    bool bVar2 = noiseComposite.allocateAndCreate(
        0x20, 0x20, 1, 1, VF_X8R8G8B8, 0x200, "NOISE_COMPOSITE" );
    if ( bVar2 ) {
        pNoiseCompositeRT = CreateRenderTargetFrom2DB( &noiseComposite, 2 );

        if ( pNoiseCompositeRT != nullptr ) {
            noiseCompositeFramebuffer.pAttachments[0] = pNoiseCompositeRT;

            bVar2 = noiseAssembleF.allocateAndCreate(
                0x100, 0x100, 1, 1, VF_X8R8G8B8, 0x200, "NOISE_ASSEMBLE_F" );
            if ( bVar2 ) {
                bVar2 = noiseAssembleS.allocateAndCreate( 0x100, 0x100, 1, 1,
                                                          VF_X8R8G8B8, 0x200,
                                                          "NOISE_ASSEMBLE_S" );
                if ( bVar2 ) {
                    pNoiseAssembleFRT =
                        CreateRenderTargetFrom2DB( &noiseAssembleF, 2 );
                    pNoiseAssembleSRT =
                        CreateRenderTargetFrom2DB( &noiseAssembleS, 2 );

                    if ( pNoiseAssembleFRT != nullptr &&
                         pNoiseAssembleSRT != nullptr ) {
                        noiseAssembleFFramebuffer.pAttachments[0] =
                            pNoiseAssembleFRT;
                        noiseAssembleSFramebuffer.pAttachments[0] =
                            pNoiseAssembleSRT;

                        bVar2 = oceanNMap.allocateAndCreate(
                            0x80, 0x80, 1, 1, VF_X8R8G8B8, 0xc200,
                            "OceanNMap" );
                        if ( bVar2 ) {
                            pOceanNMapRT =
                                CreateRenderTargetFrom2DB( &oceanNMap, 2 );

                            if ( pOceanNMapRT != nullptr ) {
                                oceanNMapFramebuffer.pAttachments[0] =
                                    pOceanNMapRT;
                                return true;
                            }
                        }
                    }
                }
            }
        }
    }

    return false;
}

void GSRender::setupRenderPasses() {
    // FUN_00990370
    renderPasses = {
        /*          pScene | pCamera | pViewport | pFramebufferAttachments |
           pLights | Flags | ShaderID | Name | pCopy | bIs3DScene | bEnabled */
        RenderPass{&sceneNoiseComposeSlow,             &camera2D,            &viewportNoise,
                   &noiseCompositeFramebuffer,                                                                                      nullptr,          0,    0, "NOISE_COMP_S",
                   nullptr,                                                                                                                                                                                      false, true }, // DAT_00f08500
        RenderPass{&sceneNoiseAssembleSlow,            &camera2D,            &viewportNoise,
                   &noiseAssembleSFramebuffer,                                                                                      nullptr,          0,    0, "NOISE_ASS_S",
                   nullptr,                                                                                                                                                                                      false, true }, // DAT_00f084f4
        RenderPass{&sceneNoiseComposeFast,             &camera2D,            &viewportNoise,
                   &noiseCompositeFramebuffer,                                                                                      nullptr,          0,    0, "NOISE_COMP_F",
                   nullptr,                                                                                                                                                                                      false, true }, // DAT_00f084e4
        RenderPass{&sceneNoiseAssembleFast,            &camera2D,            &viewportNoise,
                   &noiseAssembleFFramebuffer,                                                                                      nullptr,          0,    0, "NOISE_ASS_F",
                   nullptr,                                                                                                                                                                                      false, true }, // DAT_00f084d8
        RenderPass{&sceneOceanNmap,                    &camera2D,            &viewportNoise,
                   &oceanNMapFramebuffer,                                                                                           nullptr,          0,    0, "OCEAN_NMAP",            nullptr,
                   false,                                                                                                                                                                                               true }, // DAT_00f084cc
        RenderPass{&sceneAVTLRFront[0],                &cameraLRFront,       &viewportLRFront,
                   &rtArrayAVTLRFront[0],                                                                                           &lightsLRFront,   0,    0,
                   "AVT LR FRONT 0",                                                                                                                                                    nullptr,                 false, false}, // DAT_00f084bc
        RenderPass{&sceneAVTLRFront[1],                &cameraLRFront,       &viewportLRFront,
                   &rtArrayAVTLRFront[1],                                                                                           &lightsLRFront,   0,    0,
                   "AVT LR FRONT 1",                                                                                                                                                    nullptr,                 false, false},
        RenderPass{&sceneAVTLRFront[2],                &cameraLRFront,       &viewportLRFront,
                   &rtArrayAVTLRFront[2],                                                                                           &lightsLRFront,   0,    0,
                   "AVT LR FRONT 2",                                                                                                                                                    nullptr,                 false, false},
        RenderPass{&sceneAVTLRFront[3],                &cameraLRFront,       &viewportLRFront,
                   &rtArrayAVTLRFront[3],                                                                                           &lightsLRFront,   0,    0,
                   "AVT LR FRONT 3",                                                                                                                                                    nullptr,                 false, false},
        RenderPass{&sceneAVTLRFront[4],                &cameraLRFront,       &viewportLRFront,
                   &rtArrayAVTLRFront[4],                                                                                           &lightsLRFront,   0,    0,
                   "AVT LR FRONT 4",                                                                                                                                                    nullptr,                 false, false},
        RenderPass{&sceneAVTLRFront[5],                &cameraLRFront,       &viewportLRFront,
                   &rtArrayAVTLRFront[5],                                                                                           &lightsLRFront,   0,    0,
                   "AVT LR FRONT 5",                                                                                                                                                    nullptr,                 false, false},
        RenderPass{&sceneAVTLRFront[6],                &cameraLRFront,       &viewportLRFront,
                   &rtArrayAVTLRFront[6],                                                                                           &lightsLRFront,   0,    0,
                   "AVT LR FRONT 6",                                                                                                                                                    nullptr,                 false, false},
        RenderPass{&sceneAVTLRFront[7],                &cameraLRFront,       &viewportLRFront,
                   &rtArrayAVTLRFront[7],                                                                                           &lightsLRFront,   0,    0,
                   "AVT LR FRONT 7",                                                                                                                                                    nullptr,                 false, false},
        RenderPass{&sceneAVTLRFront[8],                &cameraLRFront,       &viewportLRFront,
                   &rtArrayAVTLRFront[8],                                                                                           &lightsLRFront,   0,    0,
                   "AVT LR FRONT 8",                                                                                                                                                    nullptr,                 false, false},
        RenderPass{&sceneAVTLRBack[0],                 &cameraLRBack,        &viewportLRBack,
                   &rtArrayAVTLRBack[0],                                                                                            &lightsLRBack,    0,    0, "AVT LR BACK 0",
                   nullptr,                                                                                                                                                                                      false, false}, // DAT_00f0842c
        RenderPass{&sceneAVTLRBack[1],                 &cameraLRBack,        &viewportLRBack,
                   &rtArrayAVTLRBack[1],                                                                                            &lightsLRBack,    0,    0, "AVT LR BACK 1",
                   nullptr,                                                                                                                                                                                      false, false},
        RenderPass{&sceneAVTLRBack[2],                 &cameraLRBack,        &viewportLRBack,
                   &rtArrayAVTLRBack[2],                                                                                            &lightsLRBack,    0,    0, "AVT LR BACK 2",
                   nullptr,                                                                                                                                                                                      false, false},
        RenderPass{&sceneAVTLRBack[3],                 &cameraLRBack,        &viewportLRBack,
                   &rtArrayAVTLRBack[3],                                                                                            &lightsLRBack,    0,    0, "AVT LR BACK 3",
                   nullptr,                                                                                                                                                                                      false, false},
        RenderPass{&sceneAVTLRBack[4],                 &cameraLRBack,        &viewportLRBack,
                   &rtArrayAVTLRBack[4],                                                                                            &lightsLRBack,    0,    0, "AVT LR BACK 4",
                   nullptr,                                                                                                                                                                                      false, false},
        RenderPass{&sceneAVTLRBack[5],                 &cameraLRBack,        &viewportLRBack,
                   &rtArrayAVTLRBack[5],                                                                                            &lightsLRBack,    0,    0, "AVT LR BACK 5",
                   nullptr,                                                                                                                                                                                      false, false},
        RenderPass{&sceneAVTLRBack[6],                 &cameraLRBack,        &viewportLRBack,
                   &rtArrayAVTLRBack[6],                                                                                            &lightsLRBack,    0,    0, "AVT LR BACK 6",
                   nullptr,                                                                                                                                                                                      false, false},
        RenderPass{&sceneAVTLRBack[7],                 &cameraLRBack,        &viewportLRBack,
                   &rtArrayAVTLRBack[7],                                                                                            &lightsLRBack,    0,    0, "AVT LR BACK 7",
                   nullptr,                                                                                                                                                                                      false, false},
        RenderPass{&sceneAVTLRBack[8],                 &cameraLRBack,        &viewportLRBack,
                   &rtArrayAVTLRBack[8],                                                                                            &lightsLRBack,    0,    0, "AVT LR BACK 8",
                   nullptr,                                                                                                                                                                                      false, false},
        RenderPass{&sceneHUDMap,                       &camera2D,            &viewportHUDMap,
                   &hudMapFramebuffer,                                                                                              nullptr,          0,    0, "RT HUD MAP",            nullptr,
                   false,                                                                                                                                                                                               false},
        RenderPass{&sceneShadowClear,                  &cameraShadow,        &viewportShadowClear,
                   &shadowFramebuffer,                                                                                              nullptr,          0x20, 0, "RT SHADOW CLR",
                   nullptr,                                                                                                                                                                                      false, false},
        RenderPass{&sceneShadow,                       &cameraShadow,        &viewportShadow,
                   &shadowFramebuffer,                                                                                              nullptr,          0x0,  0, "RT SHADOW",             nullptr,
                   false,                                                                                                                                                                                               false},
        RenderPass{&sceneShadowCockpitClear,           &cameraShadowCockpit,
                   &viewportShadowClear,                                                                 &shadowCockpitFramebuffer, nullptr,
                   0x20,                                                                                                                                    0, "RT SHADOW COCKPIT CLR", nullptr,                 false, false},
        RenderPass{&sceneShadowCockpit,                &cameraShadowCockpit, &viewportShadow,
                   &shadowCockpitFramebuffer,                                                                                       nullptr,          0x0,  0,
                   "RT SHADOW COCKPIT",                                                                                                                                                 nullptr,                 false, false},
        RenderPass{&sceneEnvMapBack,                   &cameraEnvMapBack,    &viewportEnvmapBack,
                   &envmapBackFramebuffer,                                                                                          &lightsBack,      0x20, 0, "ENVMAP BACK",
                   &sceneBack,                                                                                                                                                                                   true,  false},
        RenderPass{&sceneEnvMap,                       &cameraEnvMap,        &viewportEnvmapNear,
                   &envmapBackFramebuffer,                                                                                          &lightsEnvmap,    0x0,  0, "ENVMAP",
                   nullptr,                                                                                                                                                                                      true,  false},
        RenderPass{&sceneEnvMapMini,                   &camera2D,            &viewportEnvmapNear,
                   &envmapMiniFramebuffer,                                                                                          nullptr,          0x0,  0, "ENVMAP MINI",
                   nullptr,                                                                                                                                                                                      false, false},
        RenderPass{&sceneSunComputeEyeLum,             &camera2D,            &viewportSun,
                   &sunEyeLumFramebuffer,                                                                                           nullptr,          0x0,  0,
                   "SUN_COMPUTE_EYE_LUM",                                                                                                                                               nullptr,                 false, true },
        RenderPass{&sceneSunComputeDazzleCoeff,        &camera2D,            &viewportSun,
                   &sunDazzleCoeffFramebuffer,                                                                                      nullptr,          0x0,  0,
                   "SUN_COMPUTE_DAZZLE_COEFF",                                                                                                                                          nullptr,                 false, true },
        RenderPass{&sceneOcclusion,                    &cameraMain,          &viewportBack,
                   &mainRTwMFramebuffer,                                                                                            nullptr,          0x60, 0, "OCCLUSION",
                   nullptr,                                                                                                                                                                                      true,  true },
        RenderPass{&sceneBack,                         &cameraMain,          &viewportBack,
                   &mainRTwMFramebuffer,                                                                                            &lightsBack,      0x20, 0, "BACK",                  nullptr,
                   true,                                                                                                                                                                                                true },
        RenderPass{&sceneBack2D,                       &camera2D,            &viewportNear,
                   &mainRTwMFramebuffer,                                                                                            &lightsBack,      0x20, 0, "BACK2D",
                   nullptr,                                                                                                                                                                                      false, true },
        RenderPass{&scenePreNear,                      &cameraMain,          &clearColorStencilViewport,
                   &mainRTwMFramebuffer,                                                                                            &lightsDefault,   0x20, 0, "PRE_NEAR",
                   &sceneNear,                                                                                                                                                                                   true,  false},
        RenderPass{&sceneClearDepth,                   &cameraMain,          &clearDepthViewport,
                   &mainRTwMFramebuffer,                                                                                            nullptr,          0x20, 0, "CLEAR_ZSTENCIL",
                   nullptr,                                                                                                                                                                                      true,  true },
        RenderPass{&sceneReflectPrepass,               &cameraReflection,    &viewportNear,
                   &mainRTwMFramebuffer,                                                                                            &lightsDefault,   0x29, 0,
                   "REFLECT PRE PASS",                                                                                                                                                  &sceneReflect,           true,  false},
        RenderPass{&sceneReflect,                      &cameraReflection,    &viewportNear,
                   &mainRTwMFramebuffer,                                                                                            &lightsDefault,   0x21, 0, "REFLECT",
                   nullptr,                                                                                                                                                                                      true,  false},
        RenderPass{&sceneNearOpaque,                   &cameraMain,          &viewportNear,
                   &mainRTwMFramebuffer,                                                                                            &lightsDefault,   0x20, 0,
                   "NEAR_OPAQUE",                                                                                                                                                       &sceneNear,              true,  true },
        RenderPass{&sceneNearNoFog,                    &cameraMain,          &viewportNear,
                   &mainRTwMFramebuffer,                                                                                            &lightsDefault,   0x20, 0,
                   "NEAR_NO_FOG",                                                                                                                                                       nullptr,                 true,  true },
        RenderPass{&sceneNearPrePassVegetation,        &cameraMain,          &viewportNear,
                   &mainRTwMFramebuffer,                                                                                            &lightsDefault,   0x28, 0,
                   "VEGET PRE PASS",                                                                                                                                                    &sceneNear,              true,  true },
        RenderPass{&sceneNear,                         &cameraMain,          &viewportNear,
                   &mainRTwMFramebuffer,                                                                                            &lightsDefault,   0x20, 0, "NEAR",
                   nullptr,                                                                                                                                                                                      true,  true },
        RenderPass{&sceneNearOpaqueCarAlpha,           &cameraMain,          &clearColorViewport,
                   &carFramebuffer,                                                                                                 &lightsDefault,   0x20, 0, "NEAR_OPAQ_CA",
                   &sceneNearCarAlpha,                                                                                                                                                                           true,  false},
        RenderPass{&sceneNearCarAlpha,                 &cameraMain,          &defaultViewport,
                   &carFramebuffer,                                                                                                 &lightsDefault,   0,    0, "NEAR_CA",               nullptr,
                   true,                                                                                                                                                                                                false},
        RenderPass{&sceneNearCompositing,              &camera2D,            &defaultViewport,
                   &mainRTwMFramebuffer,                                                                                            nullptr,          0x30, 0, "NEAR_COMPO",
                   nullptr,                                                                                                                                                                                      false, false},
        RenderPass{&sceneNearAfterCarAlpha,            &cameraMain,          &defaultViewport,
                   &mainRTwMFramebuffer,                                                                                            &lightsDefault,   0x20, 0,
                   "NEAR_AFTER_CA",                                                                                                                                                     nullptr,                 true,  true },
        RenderPass{&sceneSunCore,                      &camera2D,            &defaultViewport,
                   &mainRTwMFramebuffer,                                                                                            nullptr,          0x20, 0, "SUN",                   nullptr,
                   false,                                                                                                                                                                                               true },
        RenderPass{&sceneOcclusionDOFPrepass,          &cameraMain,          &defaultViewport,
                   &mainRTFramebuffer,                                                                                              &lightsDefault,   0x28, 0, "FAKEDOF PP",
                   &sceneOcclusionDOF,                                                                                                                                                                           true,  false},
        RenderPass{&sceneOcclusionDOF,                 &cameraMain,          &defaultViewport,
                   &mainRTFramebuffer,                                                                                              &lightsDefault,   0x20, 0, "FAKEDOF",
                   nullptr,                                                                                                                                                                                      true,  false},
        RenderPass{&sceneCarPlayerPrepass,             &cameraMain,          &defaultViewport,
                   &mainRTFramebuffer,                                                                                              &lightsDefault,   0x28, 0, "CARPLAYER PP",
                   &sceneCarPlayer,                                                                                                                                                                              true,  true },
        RenderPass{&sceneCarPlayer,                    &cameraMain,          &defaultViewport,
                   &mainRTFramebuffer,                                                                                              &lightsDefault,   0,    0, "CARPLAYER",
                   nullptr,                                                                                                                                                                                      true,  true },
        RenderPass{&sceneCarPlayerPPCarAlpha,          &cameraMain,          &clearColorViewport,
                   &carPlayerFramebuffer,                                                                                           &lightsDefault,   0x28, 0,
                   "CARPLAYER PP_CA",                                                                                                                                                   &sceneCarPlayerCarAlpha, true,  false},
        RenderPass{&sceneCarPlayerCarAlpha,            &cameraMain,          &defaultViewport,
                   &carPlayerFramebuffer,                                                                                           &lightsDefault,   0,    0, "CARPLAYER_CA",
                   nullptr,                                                                                                                                                                                      true,  false},
        RenderPass{&sceneCarPlayerCompositing,         &camera2D,            &defaultViewport,
                   &mainRTFramebuffer,                                                                                              nullptr,          0x10, 0, "CARCOMPO",              nullptr,
                   false,                                                                                                                                                                                               false},
        RenderPass{&sceneOcclusionCockpit,             &cameraMain,          &defaultViewport,
                   &sunFramebuffer,                                                                                                 nullptr,          0x60, 0, "OCCLUSION_COCKPIT",
                   &sceneCockpit,                                                                                                                                                                                true,  true },
        RenderPass{&sceneSunMarker,                    &camera2D,            &clearColorViewport,
                   &sunFramebuffer,                                                                                                 nullptr,          0,    0, "SUNMARKER",             nullptr,                 false,
                   true                                                                                                                                                                                                      },
        RenderPass{&sceneSunExtractMarker,             &camera2D,            &viewportSun,
                   &sunRealLumFramebuffer,                                                                                          nullptr,          0,    0,
                   "SUN_COMPUTE_REAL_LUM",                                                                                                                                              nullptr,                 false, true },
        RenderPass{&sceneParticles,                    &cameraMain,          &defaultViewport,
                   &mainRTFramebuffer,                                                                                              &lightsDefault,   0x30, 0, "PARTICLES",
                   nullptr,                                                                                                                                                                                      true,  true },
        RenderPass{&sceneCockpit,                      &cameraMain,          &viewportInterior,
                   &mainRTFramebuffer,                                                                                              &lightsDefault,   0x20, 0, "COCKPIT",
                   nullptr,                                                                                                                                                                                      true,  true },
        RenderPass{&sceneCockpitAvatarPrepass,         &cameraMain,
                   &viewportCockpitAvatar,                                                               &mainRTFramebuffer,        &lightsDefault,
                   0x28,                                                                                                                                    0, "COCKPIT AVT PP",        &sceneCockpit,           true,  true },
        RenderPass{&sceneCockpitAvatar,                &cameraMain,          &viewportCockpitAvatar,
                   &mainRTFramebuffer,                                                                                              &lightsDefault,   0x20, 0,
                   "COCKPIT AVATAR",                                                                                                                                                    &sceneCockpit,           true,  true },
        RenderPass{&scenePostCockpit,                  &cameraMain,          &viewportCockpitAvatar,
                   &mainRTFramebuffer,                                                                                              &lightsDefault,   0x20, 0, "POST COCKPIT",
                   nullptr,                                                                                                                                                                                      true,  false},
        RenderPass{&sceneHelmet,                       &cameraHelmet,        &viewportHelmet,
                   &mainRTFramebuffer,                                                                                              &lightsDefault,   0,    0, "HELMET",                nullptr,
                   true,                                                                                                                                                                                                true },
        RenderPass{&sceneCockpitCarAlpha,              &cameraMain,
                   &clearColorDepthViewport,                                                             &carPlayerFramebuffer,
                   &lightsDefault,                                                                                                                    0x20, 0, "COCKPIT CA",            nullptr,                 true,
                   false                                                                                                                                                                                                     },
        RenderPass{&sceneCockpitAvatarPrepassCarAlpha, &cameraMain,
                   &defaultViewport,                                                                     &carPlayerFramebuffer,     &lightsDefault,
                   0x28,                                                                                                                                    0, "COCKPIT AVT PP CA",     &sceneCockpitCarAlpha,   true,
                   false                                                                                                                                                                                                     },
        RenderPass{&sceneCockpitAvatarCarAlpha,        &cameraMain,          &defaultViewport,
                   &carPlayerFramebuffer,                                                                                           &lightsDefault,   0x20, 0,
                   "COCKPIT AVATAR CA",                                                                                                                                                 &sceneCockpitCarAlpha,   true,  false},
        RenderPass{&sceneHelmetCarAlpha,               &cameraHelmet,        &defaultViewport,
                   &carPlayerFramebuffer,                                                                                           &lightsDefault,   0,    0, "HELMET CA",
                   nullptr,                                                                                                                                                                                      true,  false},
        RenderPass{&sceneCockpitCompositing,           &camera2D,            &defaultViewport,
                   &mainRTFramebuffer,                                                                                              nullptr,          0x10, 0, "COCKPIT COMPO ",
                   nullptr,                                                                                                                                                                                      false, false},
        RenderPass{&sceneSunHalo,                      &camera2D,            &defaultViewport,
                   &backbufferFramebuffer,                                                                                          nullptr,          0x20, 0, "SUN_HALO",
                   nullptr,                                                                                                                                                                                      false, true },
        RenderPass{&sceneSunDazzle,                    &camera2D,            &defaultViewport,
                   &backbufferFramebuffer,                                                                                          nullptr,          0x20, 0, "SUN_DAZZLE",
                   nullptr,                                                                                                                                                                                      false, true },
        RenderPass{&sceneSpotMap2D,                    &camera2D,            &viewport2D,
                   &backbufferFramebuffer,                                                                                          &lights2DDefault, 0x20, 0,
                   "SpotMap2D",                                                                                                                                                         nullptr,                 false, true },
        RenderPass{&sceneBefore2D,                     &camera2D,            &viewport2D,
                   &backbufferFramebuffer,                                                                                          &lights2DDefault, 0x20, 0,
                   "Before2D",                                                                                                                                                          nullptr,                 false, true },
        RenderPass{&scene2D,                           &camera2D,            &viewport2D,                &backbufferFramebuffer,
                   &lights2DDefault,                                                                                                                  0x20, 0, "2D",                    nullptr,                 false, true },
        RenderPass{&sceneAfter2D,                      &camera2D,            &viewportAfter2D,
                   &backbufferFramebuffer,                                                                                          &lights2DDefault, 0x20, 0,
                   "After2D",                                                                                                                                                           nullptr,                 false, true }
    };

    // OTDU_ADDITION: the night time parameters only reach the world scenes
    // (main and reflection cameras, environment maps); 2D, HUD (helmet) and
    // shadow scenes keep the neutral values.
    for ( RenderPass &pass : renderPasses ) {
        if ( pass.pScene == nullptr ) {
            continue;
        }
        const bool bWorldCamera =
            pass.pCamera == &cameraMain || pass.pCamera == &cameraReflection ||
            pass.pCamera == &cameraEnvMap || pass.pCamera == &cameraEnvMapBack;
        pass.pScene->setNightScene( bWorldCamera );
    }
}

void GSRender::updateWind() {
    // FUN_009981d0
    if ( gpWeather->isConfigDirty() ) {
        gpWeather->updateActiveConfig();
    }

    float fVar2 = gpWeather->getActiveWeatherConfig().getTreeWindSpeed(
        vegetationPeriod, DAT_00fac360 );
    float fVar3 = ( treeWindSpeedLimit <= fVar2 ) ? treeWindSpeedLimit : fVar2;

    Eigen::Vector3f local_30 =
        gpWeather->getActiveWeatherConfig().getRandomizedWindDirection(
            vegetationPeriod, DAT_00fac360 );
    local_30.x() = local_30.x() * fVar3 * vegetationFactor;

    Eigen::Vector3f local_20 =
        gpWeather->getActiveWeatherConfig().getRandomizedWindDirection(
            vegetationPeriod, DAT_00fac360 );
    local_30.y() = local_20.z() * fVar3 * vegetationFactor;
    local_30.z() = gpTimer->GameTotalTime;

    fVar2 = gpWeather->getActiveWeatherConfig().getRandomizedLeafWindSpeed(
        DAT_00fac360 );

    shaderUniforms[0].x() = local_30.x();
    shaderUniforms[0].y() = local_30.y();
    shaderUniforms[0].z() = local_30.z();
    shaderUniforms[0].w() = fVar2;
}

#define OTDU_SAFE_RELEASE( x )                                                 \
    if ( x != nullptr ) {                                                      \
        delete x;                                                              \
        x = nullptr;                                                           \
    }

void GSRender::destroyDeviceResources() {
    // FUN_009949e0
    FUN_00990290();
    FUN_00993df0();
    FUN_00994330();

    if ( !lightsLRBack.isEmpty() ) {
        lightsLRBack.removeLight( &lightLRBack );
    }

    if ( !lightsLRFront.isEmpty() ) {
        lightsLRFront.removeLight( &lightLRFront );
    }

    if ( !lights2DDefault.isEmpty() ) {
        lights2DDefault.removeLight( &default2DAmbientLight );
        lights2DDefault.removeLight( &default2DDirectionalLight );
    }

    if ( !lightsBack.isEmpty() ) {
        lightsBack.removeLight( &ambientLightBack );
        lightsBack.removeLight( &directionalLightBack );
    }

    if ( !lightsDefault.isEmpty() ) {
        lightsDefault.removeLight( &defaultAmbientLight );
        lightsDefault.removeLight( &defaultDirectionalLight );
    }

    for ( RenderPass &pgVar1 : renderPasses ) {
        if ( pgVar1.pScene != nullptr ) {
            pgVar1.pScene->destroy();
        }
    }
}

void GSRender::FUN_00990290() {
    // FUN_00990290
    OTDU_SAFE_RELEASE( pPostFXMainHDR );
    OTDU_SAFE_RELEASE( pPostFXMainHDRDownscale );

    OTDU_SAFE_RELEASE( pSunAvgLum );
    OTDU_SAFE_RELEASE( pSunAvgLumCopy );
    OTDU_SAFE_RELEASE( pSunDazzleLUT );
    OTDU_SAFE_RELEASE( pSunDazzleLUTCopy );
    OTDU_SAFE_RELEASE( pSunBillboard );

    for ( uint32_t i = 0; i < kNumSunDownscaleRT; i++ ) {
        OTDU_SAFE_RELEASE( pSunDownscaleChain[i] );
    }
}

void GSRender::FUN_00993df0() {
    // FUN_00993df0
    ReleasePooledRenderTarget( pOceanNMapRT );
    oceanNMap.destroy();
    ReleasePooledRenderTarget( pNoiseCompositeRT );
    ReleasePooledRenderTarget( pNoiseAssembleFRT );
    ReleasePooledRenderTarget( pNoiseAssembleSRT );
    noiseComposite.destroy();
    noiseAssembleF.destroy();
    noiseAssembleS.destroy();
    for ( uint32_t i = 0; i < kNumSunRT; i++ ) {
        sun2DBs[i].destroy();
        ReleasePooledRenderTarget( pSunRTs[i] );
    }
}

void GSRender::setSceneViewports( const float x,
                                  const float y,
                                  const float width,
                                  const float height ) {
    // FUN_009948e0
    for ( uint32_t i = eRenderPass::RP_Occlusion;
          i <= eRenderPass::RP_SunDazzle; i++ ) {
        Viewport *pViewport = renderPasses[i].pViewport;
        pViewport->X = x;
        pViewport->Y = y;
        pViewport->Width = width;
        pViewport->Height = height;
    }
}

void GSRender::FUN_00994330() {
    // FUN_00994330
    mainRTwM.destroy();
    scnDown4.destroy();
    mainRT.destroy();
    ReleasePooledRenderTarget( pMainRTwM );
    ReleasePooledRenderTarget( pMainRT );
    ReleasePooledRenderTarget( pUnknownRT );
    ReleasePooledRenderTarget( pUnknownRT2 );
    ReleasePooledRenderTarget( pScnDown4RT );
}

bool GSRender::linkPostFXNodes() {
    // FUN_00990180
    pPostFXMainHDR->setTextureToSample( pMainRT );
    pPostFXMainHDRDownscale->setEnabled( true );

    pPostFXMainHDRDownscale->connect( pPostFXMainHDR, 0, 0 );
    pPostFXMainHDRDownscale->setOutput( pScnDown4RT, 0 );

    pSunAvgLum->setTextureToSample( pSunRTs[4] );
    pSunDazzleLUT->setTextureToSample( pSunRTs[6] );

    pSunAvgLumCopy->setOutput( pSunRTs[5], 0 );
    pSunDazzleLUTCopy->setOutput( pSunRTs[7], 0 );

    pSunAvgLumCopy->setBlendOP( eBlendOP::BO_Disabled );
    pSunDazzleLUTCopy->setBlendOP( eBlendOP::BO_Disabled );

    pSunBillboard->setTextureToSample( pSunRTs[0] );

    pSunDownscaleChain[0]->setOutput( pSunRTs[1], 0 );
    pSunDownscaleChain[1]->setOutput( pSunRTs[2], 0 );
    pSunDownscaleChain[2]->setOutput( pSunRTs[3], 0 );

    return true;
}

bool GSRender::notifyConfigListeners(
    RenderConfigurationChangeListener *param_1 ) {
    // FUN_00993b30
    RenderConfigurationChangeListener *puVar1 = param_1;
    bool cVar1 = true;
    do {
        if ( puVar1 == nullptr ) {
            return cVar1;
        }
        cVar1 = puVar1->onConfigurationChanged();
        puVar1 = puVar1->pNextListener;
    } while ( cVar1 != false );

    return false;
}
