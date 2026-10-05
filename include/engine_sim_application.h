#ifndef ATG_ENGINE_SIM_ENGINE_SIM_APPLICATION_H
#define ATG_ENGINE_SIM_ENGINE_SIM_APPLICATION_H

#include "geometry_generator.h"
#include "simulator.h"
#include "engine.h"
#include "simulation_object.h"
#include "ui_manager.h"
#include "dynamometer.h"
#include "oscilloscope.h"
#include "audio_buffer.h"
#include "convolution_filter.h"
#include "shaders.h"
#include "engine_view.h"
#include "right_gauge_cluster.h"
#include "cylinder_temperature_gauge.h"
#include "synthesizer.h"
#include "oscilloscope_cluster.h"
#include "performance_cluster.h"
#include "load_simulation_cluster.h"
#include "mixer_cluster.h"
#include "info_cluster.h"
#include "control_bar.h"
#include "hero_cluster.h"
#include "application_settings.h"
#include "transmission.h"
#include "wav_writer.h"
#include "drive_controller.h"

#include "delta.h"
#include "dtv.h"

#include <vector>

class EngineSimApplication {
    private:
        static std::string s_buildVersion;

    public:
        EngineSimApplication();
        virtual ~EngineSimApplication();

        static std::string getBuildVersion() { return s_buildVersion; }

        void initialize(void *instance, ysContextObject::DeviceAPI api);
        void run();
        void destroy();

        void loadEngine(Engine *engine, Vehicle *vehicle, Transmission *transmission);
        void drawGenerated(
                const GeometryGenerator::GeometryIndices &indices,
                int layer = 0);
        void drawGeneratedUi(
                const GeometryGenerator::GeometryIndices &indices,
                int layer = 0);
        void drawGenerated(
                const GeometryGenerator::GeometryIndices &indices,
                int layer,
                dbasic::StageEnableFlags flags);
        void configure(const ApplicationSettings &settings);
        GeometryGenerator *getGeometryGenerator() { return &m_geometryGenerator; }

        Shaders *getShaders() { return &m_shaders; }
        dbasic::TextRenderer *getTextRenderer() { return &m_textRenderer; }

        void createObjects(Engine *engine);
        void destroyObjects();
        dbasic::DeltaEngine *getEngine() { return &m_engine; }

        float pixelsToUnits(float pixels) const;
        float unitsToPixels(float units) const;

        ysVector getBackgroundColor() const { return m_background; }
        ysVector getForegroundColor() const { return m_foreground; }
        ysVector getHightlight1Color() const { return m_highlight1; }
        ysVector getPink() const { return m_pink; }
        ysVector getGreen() const { return m_green; }
        ysVector getYellow() const { return m_yellow; }
        ysVector getRed() const { return m_red; }
        ysVector getOrange() const { return m_orange; }
        ysVector getBlue() const { return m_blue; }

        const SimulationObject::ViewParameters &getViewParameters() const;
        void setViewLayer(int view) { m_viewParameters.Layer0 = view; }

        dbasic::AssetManager *getAssetManager() { return &m_assetManager; }

        int getScreenWidth() const { return m_screenWidth; }
        int getScreenHeight() const { return m_screenHeight; }

        Simulator *getSimulator() { return m_simulator; }
        InfoCluster *getInfoCluster() { return m_infoCluster; }
        ApplicationSettings* getAppSettings() { return &m_applicationSettings; }

        // VRUM: actions shared by the keyboard handler and the on-screen
        // control bar, so a button and its shortcut cannot drift apart.
        void toggleIgnition();
        void toggleDyno();
        void toggleBackfire();
        void toggleRevMatch();
        void setStarterEnabled(bool enabled);
        void shiftGear(int delta);

        bool isIgnitionEnabled() const;
        bool isDynoEnabled() const;
        bool isStarterEnabled() const;
        bool isBackfireEnabled() const;
        bool isRevMatchEnabled() const { return m_drive.revMatch; }

        // Launch control and Shift Assist. The logic lives in the shared core
        // (DriveController) so the Android build behaves identically.
        void toggleLaunchControl();
        void toggleAssist();
        void toggleAssistOption(bool DriveController::Assist::*option, const char *name);
        void toggleTcu();
        DriveController &getDrive() { return m_drive; }

        // Listening perspective. Works on any engine -- it is a property of
        // where you are standing, not of the engine.
        //
        // Three modes rather than one "reverb amount", because open ground and
        // concrete are not the same room scaled: outdoors is short and heavily
        // damped (grass and air eat the top end, nothing is close enough to
        // slap back), a tunnel is long and barely damped with strong early
        // reflections off walls a few metres away. A single knob between them
        // passes through settings that are neither.
        enum class Perspective {
            Close,
            Outdoor,
            Tunnel,
            Count
        };

        // The live acoustics values. Presets write these; the ACOUSTICS sliders
        // edit them directly, which is why they are plain doubles owned here
        // rather than being derived from the preset on the fly.
        struct Acoustics {
            double mix = 0.0;        // wet/dry blend
            double roomSize = 0.4;   // reflection decay length
            double damping = 0.5;    // how fast the tail loses top end
            double early = 0.0;      // level of the discrete slap-backs
            double airCutoff = 0.0;  // Hz; 0 = off. Distance/air absorption.
        };

        void togglePerspective();
        void applyPreset(Perspective p);
        Perspective getPerspective() const { return m_perspective; }
        // Returns the preset name, or "CUSTOM" once a slider has moved it away.
        const char *getPerspectiveName() const;
        bool isOutdoorPerspective() const { return m_acoustics.mix > 0.0; }

        double *getAcousticsMix() { return &m_acoustics.mix; }
        double *getAcousticsRoom() { return &m_acoustics.roomSize; }
        double *getAcousticsDamping() { return &m_acoustics.damping; }
        double *getAcousticsEarly() { return &m_acoustics.early; }
        double *getAcousticsAir() { return &m_acoustics.airCutoff; }

        // Wheel brake, held rather than toggled. Set by the control bar's
        // button; the B key ORs into the same thing.
        void setBrakeInput(bool held) { m_brakeInput = held; }
        bool isBraking() const { return m_brakeInput; }

        // Handed to the control bar's sliders, which edit them in place. The
        // keyboard bindings write the same variables, so the two stay in sync
        // for free rather than needing to be pushed at each other.
        double *getThrottleRampTime() { return &m_drive.throttleRampTime; }
        double *getBrakeRampTime() { return &m_drive.brakeRampTime; }
        double *getPedalReleaseTime() { return &m_drive.pedalReleaseTime; }

        // Audio capture to WAV for offline spectral analysis
        // (scripts/analyze_sound.py). Independent of the video capture.
        void toggleAudioCapture();
        bool isCapturingAudio() const { return m_audioCapture.isOpen(); }

        void cycleEngine(int delta);

        // Defers cycleEngine() to the end of the frame. Anything reached from
        // inside the UI update walk MUST use this: cycleEngine() rebuilds the
        // UI tree, destroying the very element that called it.
        void requestEngineCycle(int delta) { m_pendingEngineCycle += delta; }

    protected:
        bool loadScript();
        void processEngineInput();
        void renderScene();

        void refreshUserInterface();
        void applyPerspective();

    protected:
        double m_targetSpeedSetting = 0.0;
        bool m_brakeInput = false;
        Perspective m_perspective = Perspective::Close;
        Acoustics m_acoustics;
        // Last values pushed to the synthesizer, so the sliders can be polled
        // cheaply each frame and only push on a real change.
        Acoustics m_appliedAcoustics;

        DriveController m_drive;
        double m_manualClutchLevel = 1.0;
        void drainDriveEvents();

        int m_lastMouseWheel = 0;

    protected:
        virtual void initialize();
        virtual void process(float dt);
        virtual void render();

        float m_displayAngle;
        float m_displayHeight;
        int m_gameWindowHeight;
        int m_screenWidth;
        int m_screenHeight;
        
        ApplicationSettings m_applicationSettings;
        dbasic::ShaderSet m_shaderSet;
        Shaders m_shaders;

        dbasic::DeltaEngine m_engine;
        dbasic::AssetManager m_assetManager;

        std::string m_assetPath;
        // Where delta-studio's shaders and fonts live; needed after startup
        // because the UI font is loaded from it.
        std::string m_enginePath;

        ysRenderTarget *m_mainRenderTarget;
        ysGPUBuffer *m_geometryVertexBuffer;
        ysGPUBuffer *m_geometryIndexBuffer;

        GeometryGenerator m_geometryGenerator;
        dbasic::TextRenderer m_textRenderer;

        std::vector<SimulationObject *> m_objects;
        Engine *m_iceEngine;
        Vehicle *m_vehicle;
        Transmission *m_transmission;
        Simulator *m_simulator;
        double m_torque;

        UiManager m_uiManager;
        EngineView *m_engineView;
        RightGaugeCluster *m_rightGaugeCluster;
        OscilloscopeCluster *m_oscCluster;
        CylinderTemperatureGauge *m_temperatureGauge;
        PerformanceCluster *m_performanceCluster;
        LoadSimulationCluster *m_loadSimulationCluster;
        MixerCluster *m_mixerCluster;
        InfoCluster *m_infoCluster;
        ControlBar *m_controlBar;
        HeroCluster *m_heroCluster;
        SimulationObject::ViewParameters m_viewParameters;

        bool m_paused;

    protected:
        void startRecording();
        void updateScreenSizeStability();
        bool readyToRecord();
        void stopRecording();
        void recordFrame();
        bool isRecording() const { return m_recording; }

        // VRUM: runtime engine switching. Every engine .mr defines its own
        // `main` node and assets/main.mr is only a wrapper that imports one of
        // them, so switching means regenerating that wrapper and recompiling.
        std::string workspacePath(const std::string &sub) const;
        void scanEngineFiles();
        void selectEngine(int index);

        static constexpr int ScreenResolutionHistoryLength = 5;
        int m_screenResolution[ScreenResolutionHistoryLength][2];
        int m_screenResolutionIndex;
        bool m_recording;

        ysVector m_background;
        ysVector m_foreground;
        ysVector m_shadow;
        ysVector m_highlight1;
        ysVector m_highlight2;

        ysVector m_pink;
        ysVector m_orange;
        ysVector m_yellow;
        ysVector m_red;
        ysVector m_green;
        ysVector m_blue;

        ysAudioBuffer *m_outputAudioBuffer;
        AudioBuffer m_audioBuffer;
        ysAudioSource *m_audioSource;
        WavWriter m_audioCapture;

        // Engine scripts discovered under assets/engines, stored relative to
        // assets/ so they can be written straight into an import statement.
        std::vector<std::string> m_engineFiles;
        int m_currentEngine = -1;
        // Non-zero means "cycle by this much once the UI walk has unwound".
        int m_pendingEngineCycle = 0;
        // Set from m_assetPath during initialize(); the literal here is only a
        // fallback for the build/-relative layout.
        std::string m_scriptPath = "../assets/main.mr";

        int m_oscillatorSampleOffset;
        int m_screen;

#ifdef ATG_ENGINE_SIM_VIDEO_CAPTURE
        atg_dtv::Encoder m_encoder;
#endif /* ATG_ENGINE_SIM_VIDEO_CAPTURE */
};

#endif /* ATG_ENGINE_SIM_ENGINE_SIM_APPLICATION_H */
