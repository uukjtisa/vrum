#ifndef ATG_ENGINE_SIM_COMPILER_H
#define ATG_ENGINE_SIM_COMPILER_H

#include "language_rules.h"

#include "engine_sim.h"

#include <string>
#include "piranha.h"

#include <vector>

namespace es_script {

    class Compiler {
    public:
        struct Output {
            Engine *engine = nullptr;
            Vehicle *vehicle = nullptr;
            Transmission *transmission = nullptr;
            Simulator::Parameters simulatorParameters;
            ApplicationSettings applicationSettings;

            std::vector<Function *> functions;
        };

    private:
        static Output *s_output;

    public:
        Compiler();
        ~Compiler();

        static Output *output();

        void initialize();

        // Adds an absolute directory to search for imports. The built-in paths
        // are relative to the working directory, which only works when the
        // binary is launched from the right folder; this lets the caller anchor
        // the search on the executable's own location instead.
        void addSearchPath(const std::string &path);
        bool compile(const piranha::IrPath &path);
        Output execute();
        void destroy();

    private:
        void printError(const piranha::CompilationError *err, std::ofstream &file) const;

    private:
        LanguageRules m_rules;
        piranha::Compiler *m_compiler;
        piranha::NodeProgram m_program;
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_COMPILER_H */