// libfpv - Betaflight configuration through Betaflight's own CLI: libfpv's
// base defaults, filtering of host/player commands and `diff all` import.
// Pure text processing; the runner executes the resulting scripts on the
// simulation thread.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <string>
#include <vector>

namespace fpv {

// Settings libfpv needs regardless of the tune: PWM (the only protocol
// SimITL drives), ARM on AUX1, no power-on grace, MSP receiver, OSD.
std::string betaflightBaseDefaults();
// The out-of-the-box tune for a fresh EEPROM: base defaults + modes on
// AUX2-4 + a readable OSD layout.
std::string betaflightFirstRunScript();
// For EEPROMs made before turtle mode existed: given the CLI's `aux`
// output, the `aux` line that puts turtle on AUX4, or "" when turtle is
// configured already or AUX4 is used by another mode.
std::string betaflightTurtleMigration(const std::string &auxDump);

struct CliFilterResult {
    std::string script;                 // what to run
    std::vector<std::string> refused;   // "<line>: <reason>"
    bool needsBaseDefaults = false;     // a "defaults" was rewritten
};

// Rewrites save/defaults/exit to their no-reboot forms and refuses
// commands that make no sense in a simulator (bootloader, passthrough...).
CliFilterResult filterCliCommands(const std::string &commands);

struct DiffImport {
    std::string script;                // full script to execute
    std::vector<std::string> applied;  // informational
    std::vector<std::string> skipped;  // "<line>  -> <reason>"
    std::vector<std::string> remapped; // e.g. "ANGLE: AUX3 -> AUX2"
    std::string boardName, firmware;   // from the diff header, if present
};

// Builds the import script for a real quad's `diff all` / `dump`.
DiffImport buildDiffImport(const std::string &diffText);
std::string formatImportReport(const DiffImport &imp, const std::string &cliOutput);

// Names of Betaflight modes by box id (as used by the `aux` command).
const char *betaflightModeName(int boxId);

}  // namespace fpv
